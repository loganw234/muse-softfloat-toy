/* ckpt_flash.c - board backend for core/toy_ckpt_slot.h.
 *
 * Reads are plain XIP reads (flash is memory-mapped; the slot
 * bytes are validated in place, then copied). Writes funnel
 * through one flash_safe_execute callback that erases and
 * programs under the SDK's interrupt/core safety protocol; the
 * low-level hardware/flash.h calls inside it handle leaving and
 * re-entering XIP per operation, so this file must never touch
 * QSPI directly. See ckpt_flash.h for the contract.
 */
#include <string.h>
#include "pico.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "ckpt_flash.h"

/* The pure geometry assumes the SDK's NOR units; refuse to build
 * against an SDK/board where that stops being true. */
_Static_assert(FLASH_SECTOR_SIZE == TOY_CKPT_SLOT_SECTOR_BYTES,
               "SDK FLASH_SECTOR_SIZE differs from toy_ckpt_slot.h");
_Static_assert(FLASH_PAGE_SIZE == TOY_CKPT_SLOT_PAGE_BYTES,
               "SDK FLASH_PAGE_SIZE differs from toy_ckpt_slot.h");

/* Generous: the callback itself is erase/program hardware time;
 * this bounds only entering/leaving the safe zone. */
#define TOY_CKPT_FLASH_SAFE_TIMEOUT_MS 1000u

static toy_ckpt_slot_layout_t g_layout;
static int g_layout_ready = 0;  /* 1 ok, -1 unresolvable */
static int g_last_safe_rc = 0;

const toy_ckpt_slot_layout_t *toy_ckpt_flash_layout(void)
{
    if (g_layout_ready == 0) {
#ifdef PICO_FLASH_SIZE_BYTES
        g_layout_ready =
            toy_ckpt_slot_layout_default(&g_layout,
                                         (uint32_t)PICO_FLASH_SIZE_BYTES, 0u)
                ? 1
                : -1;
#else
        g_layout_ready = -1;
#endif
    }
    return g_layout_ready == 1 ? &g_layout : NULL;
}

int toy_ckpt_flash_last_safe_rc(void)
{
    return g_last_safe_rc;
}

const unsigned char *toy_ckpt_flash_slot_xip(uint32_t index)
{
    const toy_ckpt_slot_layout_t *layout = toy_ckpt_flash_layout();
    uint32_t offset = 0;
    if (!layout || !toy_ckpt_slot_offset(layout, index, &offset))
        return NULL;
    return (const unsigned char *)(XIP_BASE + offset);
}

/* Scan every slot once; infos/cands arrays are layout-sized. */
static void scan_slots(const toy_ckpt_slot_layout_t *layout,
                       const toy_cart_t *expected,
                       toy_ckpt_slot_candidate_t *cands,
                       toy_ckpt_info_t *infos)
{
    for (uint32_t i = 0; i < layout->slot_count; i++) {
        const unsigned char *xip = toy_ckpt_flash_slot_xip(i);
        toy_ckpt_slot_probe(xip, layout->slot_bytes, expected, &cands[i],
                            &infos[i]);
    }
}

toy_ckpt_flash_err_t toy_ckpt_flash_find_newest(const toy_cart_t *expected,
                                                uint32_t *index_out,
                                                toy_ckpt_info_t *info_out)
{
    const toy_ckpt_slot_layout_t *layout = toy_ckpt_flash_layout();
    if (!layout)
        return TOY_CKPT_FLASH_ERR_LAYOUT;
    toy_ckpt_slot_candidate_t cands[TOY_CKPT_SLOT_MAX_COUNT];
    toy_ckpt_info_t infos[TOY_CKPT_SLOT_MAX_COUNT];
    scan_slots(layout, expected, cands, infos);
    int best = toy_ckpt_slot_newest(cands, layout->slot_count);
    if (best < 0)
        return TOY_CKPT_FLASH_ERR_NO_VALID;
    if (index_out)
        *index_out = (uint32_t)best;
    if (info_out)
        *info_out = infos[best];
    return TOY_CKPT_FLASH_OK;
}

toy_ckpt_flash_err_t toy_ckpt_flash_read_slot(uint32_t index,
                                              const toy_cart_t *expected,
                                              unsigned char *out,
                                              size_t out_cap,
                                              size_t *out_len,
                                              toy_ckpt_info_t *info_out)
{
    const toy_ckpt_slot_layout_t *layout = toy_ckpt_flash_layout();
    if (!layout)
        return TOY_CKPT_FLASH_ERR_LAYOUT;
    if (!out)
        return TOY_CKPT_FLASH_ERR_ARG;
    const unsigned char *xip = toy_ckpt_flash_slot_xip(index);
    if (!xip)
        return TOY_CKPT_FLASH_ERR_RANGE;
    toy_ckpt_slot_candidate_t cand;
    toy_ckpt_info_t info;
    if (toy_ckpt_slot_probe(xip, layout->slot_bytes, expected, &cand, &info) !=
        TOY_CKPT_OK)
        return TOY_CKPT_FLASH_ERR_NO_VALID;
    if (out_cap < cand.image_len)
        return TOY_CKPT_FLASH_ERR_RANGE;
    memcpy(out, xip, cand.image_len);
    if (out_len)
        *out_len = cand.image_len;
    if (info_out)
        *info_out = info;
    return TOY_CKPT_FLASH_OK;
}

toy_ckpt_flash_err_t toy_ckpt_flash_load_newest(const toy_cart_t *expected,
                                                unsigned char *out,
                                                size_t out_cap,
                                                size_t *out_len,
                                                uint32_t *index_out,
                                                toy_ckpt_info_t *info_out)
{
    uint32_t index = 0;
    toy_ckpt_flash_err_t err =
        toy_ckpt_flash_find_newest(expected, &index, NULL);
    if (err != TOY_CKPT_FLASH_OK)
        return err;
    err = toy_ckpt_flash_read_slot(index, expected, out, out_cap, out_len,
                                   info_out);
    if (err != TOY_CKPT_FLASH_OK)
        return err;
    if (index_out)
        *index_out = index;
    return TOY_CKPT_FLASH_OK;
}

/* ---- write path (one safe-execute callback) ----------------- */

typedef struct {
    uint32_t offset;            /* flash offset, sector-aligned */
    const unsigned char *data;  /* SRAM image, or NULL = erase only */
    uint32_t erase_bytes;       /* whole target slot(s) */
    uint32_t image_len;
    uint32_t page_bytes;
} write_params_t;

/* Runs with interrupts disabled / other core safe (flash.c).
 * Placed in SRAM (__not_in_flash_func) like the hardware_flash
 * routines it calls, so nothing here - code, the params struct,
 * the caller's SRAM image - needs flash while XIP is down. The
 * tail page is staged in a 256 B stack buffer padded with the
 * erased byte. */
static void __not_in_flash_func(write_callback)(void *param)
{
    write_params_t *p = (write_params_t *)param;
    flash_range_erase(p->offset, p->erase_bytes);
    if (!p->data || p->image_len == 0u)
        return;
    uint32_t full = p->image_len - (p->image_len % p->page_bytes);
    if (full > 0u)
        flash_range_program(p->offset, p->data, full);
    uint32_t tail = p->image_len - full;
    if (tail > 0u) {
        unsigned char page[TOY_CKPT_SLOT_PAGE_BYTES];
        for (uint32_t i = 0; i < p->page_bytes; i++)
            page[i] = (unsigned char)TOY_CKPT_SLOT_ERASED;
        for (uint32_t i = 0; i < tail; i++)
            page[i] = p->data[full + i];
        flash_range_program(p->offset + full, page, p->page_bytes);
    }
}

static toy_ckpt_flash_err_t run_write(write_params_t *params)
{
    g_last_safe_rc = flash_safe_execute(write_callback, params,
                                         TOY_CKPT_FLASH_SAFE_TIMEOUT_MS);
    if (g_last_safe_rc != PICO_OK)
        return TOY_CKPT_FLASH_ERR_SAFE;
    return TOY_CKPT_FLASH_OK;
}

/* A buffer in the XIP window cannot be read while programming. */
static int points_into_xip(const unsigned char *p)
{
    uintptr_t v = (uintptr_t)p;
    return v >= (uintptr_t)XIP_BASE && v < (uintptr_t)XIP_BASE + 0x10000000u;
}

toy_ckpt_flash_err_t toy_ckpt_flash_write_slot(uint32_t index,
                                               const unsigned char *image,
                                               size_t image_len)
{
    const toy_ckpt_slot_layout_t *layout = toy_ckpt_flash_layout();
    if (!layout)
        return TOY_CKPT_FLASH_ERR_LAYOUT;
    if (!image || points_into_xip(image))
        return TOY_CKPT_FLASH_ERR_ARG;
    uint32_t offset = 0;
    if (!toy_ckpt_slot_offset(layout, index, &offset))
        return TOY_CKPT_FLASH_ERR_RANGE;
    if (image_len == 0u || image_len > layout->slot_bytes ||
        image_len > TOY_CKPT_MAX_IMAGE_SIZE)
        return TOY_CKPT_FLASH_ERR_RANGE;
    if (toy_ckpt_validate(image, image_len, NULL, NULL, NULL) != TOY_CKPT_OK)
        return TOY_CKPT_FLASH_ERR_IMAGE;
    write_params_t params;
    params.offset = offset;
    params.data = image;
    params.erase_bytes = layout->slot_bytes;
    params.image_len = (uint32_t)image_len;
    params.page_bytes = layout->page_bytes;
    toy_ckpt_flash_err_t err = run_write(&params);
    if (err != TOY_CKPT_FLASH_OK)
        return err;
    /* Read back through XIP: full re-validation + byte compare. */
    const unsigned char *xip = toy_ckpt_flash_slot_xip(index);
    toy_ckpt_slot_candidate_t cand;
    if (!xip ||
        toy_ckpt_slot_probe(xip, layout->slot_bytes, NULL, &cand, NULL) !=
            TOY_CKPT_OK ||
        cand.image_len != image_len || memcmp(xip, image, image_len) != 0)
        return TOY_CKPT_FLASH_ERR_VERIFY;
    return TOY_CKPT_FLASH_OK;
}

toy_ckpt_flash_err_t toy_ckpt_flash_write_next(const unsigned char *image,
                                               size_t image_len,
                                               uint32_t *index_out)
{
    const toy_ckpt_slot_layout_t *layout = toy_ckpt_flash_layout();
    if (!layout)
        return TOY_CKPT_FLASH_ERR_LAYOUT;
    if (!image)
        return TOY_CKPT_FLASH_ERR_ARG;
    /* Rotation is scoped to the image's own cartridge: validate
     * once here to learn which cart that is, then scan for that
     * cart's newest slot. Another cart's higher step_count never
     * steers this write (toy_ckpt_slot.h selection caveat). */
    const toy_cart_t *cart = NULL;
    if (toy_ckpt_validate(image, image_len, NULL, NULL, &cart) != TOY_CKPT_OK ||
        !cart)
        return TOY_CKPT_FLASH_ERR_IMAGE;
    toy_ckpt_slot_candidate_t cands[TOY_CKPT_SLOT_MAX_COUNT];
    toy_ckpt_info_t infos[TOY_CKPT_SLOT_MAX_COUNT];
    scan_slots(layout, cart, cands, infos);
    int next = toy_ckpt_slot_next(cands, layout->slot_count);
    if (next < 0)
        return TOY_CKPT_FLASH_ERR_LAYOUT;
    toy_ckpt_flash_err_t err =
        toy_ckpt_flash_write_slot((uint32_t)next, image, image_len);
    if (err != TOY_CKPT_FLASH_OK)
        return err;
    if (index_out)
        *index_out = (uint32_t)next;
    return TOY_CKPT_FLASH_OK;
}

toy_ckpt_flash_err_t toy_ckpt_flash_erase_slot(uint32_t index)
{
    const toy_ckpt_slot_layout_t *layout = toy_ckpt_flash_layout();
    if (!layout)
        return TOY_CKPT_FLASH_ERR_LAYOUT;
    uint32_t offset = 0;
    if (!toy_ckpt_slot_offset(layout, index, &offset))
        return TOY_CKPT_FLASH_ERR_RANGE;
    write_params_t params;
    params.offset = offset;
    params.data = NULL;
    params.erase_bytes = layout->slot_bytes;
    params.image_len = 0u;
    params.page_bytes = layout->page_bytes;
    return run_write(&params);
}

toy_ckpt_flash_err_t toy_ckpt_flash_erase_all(void)
{
    const toy_ckpt_slot_layout_t *layout = toy_ckpt_flash_layout();
    if (!layout)
        return TOY_CKPT_FLASH_ERR_LAYOUT;
    write_params_t params;
    params.offset = layout->base_offset;
    params.data = NULL;
    params.erase_bytes = layout->total_bytes;
    params.image_len = 0u;
    params.page_bytes = layout->page_bytes;
    return run_write(&params);
}

#if TOY_CKPT_FLASH_SELFTEST
/* Round-trip scratch: statically allocated only in selftest
 * builds so the default firmware RAM cost stays zero. */
static unsigned char selftest_scratch[TOY_CKPT_MAX_IMAGE_SIZE];

toy_ckpt_flash_err_t toy_ckpt_flash_selftest_roundtrip(const unsigned char *image,
                                                       size_t image_len,
                                                       uint32_t *index_out)
{
    if (!image || image_len > sizeof selftest_scratch)
        return TOY_CKPT_FLASH_ERR_ARG;
    uint32_t written = 0;
    toy_ckpt_flash_err_t err =
        toy_ckpt_flash_write_next(image, image_len, &written);
    if (err != TOY_CKPT_FLASH_OK)
        return err;
    const toy_cart_t *cart = NULL;
    if (toy_ckpt_validate(image, image_len, NULL, NULL, &cart) !=
        TOY_CKPT_OK)
        return TOY_CKPT_FLASH_ERR_IMAGE;
    size_t back_len = 0;
    uint32_t back_index = 0;
    err = toy_ckpt_flash_load_newest(cart, selftest_scratch,
                                     sizeof selftest_scratch, &back_len,
                                     &back_index, NULL);
    if (err != TOY_CKPT_FLASH_OK)
        return err;
    if (back_index != written || back_len != image_len ||
        memcmp(selftest_scratch, image, image_len) != 0)
        return TOY_CKPT_FLASH_ERR_VERIFY;
    if (index_out)
        *index_out = written;
    return TOY_CKPT_FLASH_OK;
}
#endif

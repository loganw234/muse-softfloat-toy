/* toy_ckpt_slot.h - pure slot arithmetic for flash checkpoints.
 *
 * Header-only, no SDK includes, no storage access: every function
 * works on caller-supplied numbers and buffers, so the same logic
 * runs in the host fake-flash test (tools/flashslot_test.c) and in
 * the board backend (board/ckpt_flash.c). The image itself is
 * TOYCKPT1 (core/toy_ckpt.h); this file owns only where images sit
 * in flash and which of several slots wins.
 *
 * Layout (docs/study-checkpoint.md section 5.1, A/B form): slots
 * live at the END of flash, sized for the format cap rather than
 * today's largest cartridge, so the layout never moves when a
 * bigger cartridge lands. With the defaults below one slot is
 * ceil(65600 / 4096) * 4096 = 69,632 B (17 sectors) and the A/B
 * pair reserves 139,264 B. The image sits verbatim at slot offset
 * 0, padded with erased 0xFF bytes to a program-page multiple;
 * there is no wrapper around it. A torn write is therefore
 * detected exactly the way a torn file is: header/payload CRC or
 * state-hash mismatch in toy_ckpt_validate(). The study's wrapper
 * (slot_seq + commit word) exists to order writes across a
 * cartridge switch; v1 instead scopes selection to one running
 * cartridge and orders by the image's step_count (see below).
 *
 * Selection rule, defined here once: among slots whose image
 * passes toy_ckpt_validate() in full (magic, both CRCs, state
 * hash, registry checks) for the caller's expected cartridge (or
 * any cartridge if the caller validates with expected == NULL),
 * the valid slot with the highest header step_count is newest.
 * Blank (all-0xFF) and torn/corrupt slots are never candidates.
 * Ties (the same image saved twice) resolve to the lowest slot
 * index, which keeps the choice deterministic; the payloads are
 * interchangeable up to their bytes. Caveat, stated rather than
 * hidden: step_count is per-cartridge-run, so it decreases
 * legitimately when the cartridge switches or restarts. Callers
 * must therefore only compare slots naming the same cartridge
 * (validate with a non-NULL expected), and Friday's wiring
 * decides whether a cartridge switch erases the slots or simply
 * lets the next save rotate over the old cartridge's image.
 *
 * Rotation: the next write goes to the slot after the newest
 * valid one (wrapping), or slot 0 when nothing is valid. The
 * newest valid image is thus never the slot being erased: power
 * lost at any point during erase/program leaves the previous
 * checkpoint intact and selectable.
 */
#ifndef TOY_CKPT_SLOT_H
#define TOY_CKPT_SLOT_H

#include <stddef.h>
#include <stdint.h>
#include "toy_ckpt.h"

/* NOR geometry for the W25Q-class parts on the target boards
 * (Pico SDK FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; the board
 * backend static-asserts these against the SDK values). */
#define TOY_CKPT_SLOT_SECTOR_BYTES 4096u
#define TOY_CKPT_SLOT_PAGE_BYTES    256u
#define TOY_CKPT_SLOT_ERASED       0xffu
/* A/B pair (study section 5.1 v1 form). Wider rings are a
 * wear variant the functions below already generalise to. */
#define TOY_CKPT_SLOT_COUNT           2u
#define TOY_CKPT_SLOT_MAX_COUNT      16u

/* Resolved placement of the slot region. All offsets are flash
 * offsets (0 = start of the QSPI device), not XIP addresses. */
typedef struct {
    uint32_t flash_bytes;            /* usable flash (rounded down) */
    uint32_t firmware_reserve_bytes; /* low flash kept for firmware */
    uint32_t sector_bytes;
    uint32_t page_bytes;
    uint32_t max_image_bytes;        /* image cap the slot was sized for */
    uint32_t slot_bytes;             /* sector multiple >= max_image */
    uint32_t slot_count;
    uint32_t total_bytes;            /* slot_bytes * slot_count */
    uint32_t base_offset;            /* flash_bytes - total_bytes */
} toy_ckpt_slot_layout_t;

/* Per-slot parse outcome, the input to selection. Built by
 * toy_ckpt_slot_probe() over a slot buffer, or by hand from a
 * toy_ckpt_validate() result the caller already has. */
typedef struct {
    int            valid;      /* full validate passed */
    int            blank;      /* header area fully erased (0xFF) */
    toy_ckpt_err_t err;        /* validate/parse error when !valid */
    uint64_t       step_count; /* header step_count when valid */
    uint32_t       image_len;  /* 64 + state_size when valid */
} toy_ckpt_slot_candidate_t;

static inline int toy_ckpt_slot_is_pow2_u32(uint32_t v)
{
    return v != 0u && (v & (v - 1u)) == 0u;
}

/* Smallest sector multiple holding max_image_bytes, or 0 on bad
 * input (zero sizes, non-power-of-two sector, overflow). */
static inline uint32_t toy_ckpt_slot_size_for(uint32_t max_image_bytes,
                                              uint32_t sector_bytes)
{
    if (max_image_bytes == 0u || !toy_ckpt_slot_is_pow2_u32(sector_bytes))
        return 0u;
    uint64_t rounded =
        ((uint64_t)max_image_bytes + sector_bytes - 1u) /
        sector_bytes * sector_bytes;
    if (rounded > 0xffffffffull)
        return 0u;
    return (uint32_t)rounded;
}

/* Image length rounded up to a program-page multiple (the number
 * of bytes a slot write programs; the pad bytes stay 0xFF), or 0
 * on bad input. */
static inline uint32_t toy_ckpt_slot_program_bytes(uint32_t image_len,
                                                   uint32_t page_bytes)
{
    if (image_len == 0u || !toy_ckpt_slot_is_pow2_u32(page_bytes))
        return 0u;
    uint64_t rounded =
        ((uint64_t)image_len + page_bytes - 1u) / page_bytes * page_bytes;
    if (rounded > 0xffffffffull)
        return 0u;
    return (uint32_t)rounded;
}

/* Resolve a slot layout at the end of flash. flash_bytes is
 * rounded down to a whole sector; firmware_reserve_bytes is the
 * amount of low flash the firmware may occupy, rounded up to a
 * whole sector for the overlap check (0 = no explicit reserve;
 * the region still must fit in flash). Returns 1 and fills *out
 * on success; returns 0 (and zeroes *out) for any geometry that
 * does not fit: bad units, zero/oversized counts, a slot region
 * larger than flash, or a region overlapping the reserve. */
static inline int toy_ckpt_slot_layout_init(toy_ckpt_slot_layout_t *out,
                                            uint32_t flash_bytes,
                                            uint32_t firmware_reserve_bytes,
                                            uint32_t sector_bytes,
                                            uint32_t page_bytes,
                                            uint32_t max_image_bytes,
                                            uint32_t slot_count)
{
    if (!out)
        return 0;
    for (size_t i = 0; i < sizeof *out; i++)
        ((unsigned char *)out)[i] = 0;
    if (!toy_ckpt_slot_is_pow2_u32(sector_bytes) ||
        !toy_ckpt_slot_is_pow2_u32(page_bytes) ||
        page_bytes > sector_bytes || flash_bytes < sector_bytes ||
        slot_count == 0u || slot_count > TOY_CKPT_SLOT_MAX_COUNT)
        return 0;
    uint32_t slot_bytes = toy_ckpt_slot_size_for(max_image_bytes, sector_bytes);
    if (slot_bytes == 0u)
        return 0;
    uint64_t total = (uint64_t)slot_bytes * slot_count;
    uint32_t usable = flash_bytes - (flash_bytes % sector_bytes);
    if (total > usable)
        return 0;
    uint32_t base = usable - (uint32_t)total;
    uint64_t reserve_rounded =
        ((uint64_t)firmware_reserve_bytes + sector_bytes - 1u) /
        sector_bytes * sector_bytes;
    if ((uint64_t)base < reserve_rounded)
        return 0;
    out->flash_bytes = usable;
    out->firmware_reserve_bytes = firmware_reserve_bytes;
    out->sector_bytes = sector_bytes;
    out->page_bytes = page_bytes;
    out->max_image_bytes = max_image_bytes;
    out->slot_bytes = slot_bytes;
    out->slot_count = slot_count;
    out->total_bytes = (uint32_t)total;
    out->base_offset = base;
    return 1;
}

/* The default layout: 4 KiB sectors, 256 B pages, cap-sized
 * A/B slots for TOYCKPT1's maximum image. */
static inline int toy_ckpt_slot_layout_default(toy_ckpt_slot_layout_t *out,
                                               uint32_t flash_bytes,
                                               uint32_t firmware_reserve_bytes)
{
    return toy_ckpt_slot_layout_init(out, flash_bytes, firmware_reserve_bytes,
                                     TOY_CKPT_SLOT_SECTOR_BYTES,
                                     TOY_CKPT_SLOT_PAGE_BYTES,
                                     TOY_CKPT_MAX_IMAGE_SIZE,
                                     TOY_CKPT_SLOT_COUNT);
}

/* Flash offset of slot index. Returns 1 + *offset_out, or 0 for
 * a bad layout/index. Offsets are sector-aligned by construction. */
static inline int toy_ckpt_slot_offset(const toy_ckpt_slot_layout_t *layout,
                                       uint32_t index, uint32_t *offset_out)
{
    if (!layout || !offset_out || index >= layout->slot_count ||
        layout->slot_bytes == 0u)
        return 0;
    *offset_out = layout->base_offset + index * layout->slot_bytes;
    return 1;
}

/* 1 iff buf[0..len) is entirely the erased byte. len == 0 (or a
 * NULL buffer) is 0: "no evidence" is not "blank". */
static inline int toy_ckpt_slot_is_blank(const unsigned char *buf, size_t len)
{
    if (!buf || len == 0u)
        return 0;
    for (size_t i = 0; i < len; i++)
        if (buf[i] != TOY_CKPT_SLOT_ERASED)
            return 0;
    return 1;
}

/* Probe one slot buffer of slot_len bytes (a full slot, image at
 * offset 0): parse the header to learn the exact image length,
 * then run toy_ckpt_validate() over exactly that slice (the
 * 0xFF slot padding beyond it is not part of the image and must
 * not trip the exact-length check). Fills *cand (always, when
 * given) and *info_out on success; returns the validate/parse
 * error, TOY_CKPT_OK only when the slot holds a fully valid
 * image for `expected` (NULL = any registered cartridge). An
 * image claiming more bytes than the slot holds fails as
 * TOY_CKPT_ERR_LENGTH. Blank slots report cand->blank = 1 and
 * fail as TOY_CKPT_ERR_BAD_MAGIC, same as an erased file. */
static inline toy_ckpt_err_t toy_ckpt_slot_probe(
    const unsigned char *buf, size_t slot_len, const toy_cart_t *expected,
    toy_ckpt_slot_candidate_t *cand, toy_ckpt_info_t *info_out)
{
    toy_ckpt_slot_candidate_t sink;
    if (!cand)
        cand = &sink;
    cand->valid = 0;
    cand->blank = 0;
    cand->err = TOY_CKPT_ERR_NULL;
    cand->step_count = 0;
    cand->image_len = 0;
    if (!buf)
        return cand->err;
    if (slot_len >= TOY_CKPT_HEADER_SIZE)
        cand->blank = toy_ckpt_slot_is_blank(buf, TOY_CKPT_HEADER_SIZE);
    toy_ckpt_info_t info;
    toy_ckpt_err_t err = toy_ckpt_parse_header(buf, slot_len, &info);
    if (err != TOY_CKPT_OK) {
        cand->err = err;
        return err;
    }
    uint64_t image_len = (uint64_t)TOY_CKPT_HEADER_SIZE + info.state_size;
    if (image_len > slot_len || info.state_size > TOY_MAX_STATE_SIZE) {
        cand->err = TOY_CKPT_ERR_LENGTH;
        return cand->err;
    }
    err = toy_ckpt_validate(buf, (size_t)image_len, expected, &info, NULL);
    cand->err = err;
    if (err != TOY_CKPT_OK)
        return err;
    cand->valid = 1;
    cand->blank = 0;
    cand->step_count = info.step_count;
    cand->image_len = (uint32_t)image_len;
    if (info_out)
        *info_out = info;
    return TOY_CKPT_OK;
}

/* Build a candidate by hand from a validate result the caller
 * already computed (info only read when err == TOY_CKPT_OK). */
static inline toy_ckpt_slot_candidate_t toy_ckpt_slot_candidate_from(
    toy_ckpt_err_t err, const toy_ckpt_info_t *info, int blank)
{
    toy_ckpt_slot_candidate_t c;
    c.valid = err == TOY_CKPT_OK && info != NULL;
    c.blank = blank ? 1 : 0;
    c.err = err;
    c.step_count = c.valid ? info->step_count : 0u;
    c.image_len = c.valid ? TOY_CKPT_HEADER_SIZE + info->state_size : 0u;
    return c;
}

/* Newest valid slot index by step_count (ties: lowest index),
 * or -1 when no candidate is valid. */
static inline int toy_ckpt_slot_newest(const toy_ckpt_slot_candidate_t *cands,
                                       uint32_t count)
{
    if (!cands)
        return -1;
    int best = -1;
    for (uint32_t i = 0; i < count; i++) {
        if (!cands[i].valid)
            continue;
        if (best < 0 || cands[i].step_count > cands[best].step_count)
            best = (int)i;
    }
    return best;
}

/* Slot after `newest` (wrapping), or 0 when newest < 0 (nothing
 * valid: start at slot 0). Returns -1 only for count == 0. */
static inline int toy_ckpt_slot_next_after(int newest, uint32_t count)
{
    if (count == 0u)
        return -1;
    if (newest < 0)
        return 0;
    return (int)(((uint32_t)newest + 1u) % count);
}

/* Rotation in one call: the slot the next write must target so
 * the newest valid image survives a power loss mid-write. */
static inline int toy_ckpt_slot_next(const toy_ckpt_slot_candidate_t *cands,
                                     uint32_t count)
{
    return toy_ckpt_slot_next_after(toy_ckpt_slot_newest(cands, count), count);
}

#endif

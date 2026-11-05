/* flashslot_test.c - standalone host test for core/toy_ckpt_slot.h.
 *
 * Deliberately NOT part of tools/run.sh (mirroring ckpt_test.c):
 * build with the run.sh host flags - gcc -O2 -std=c11 -Wall
 * -Wextra, include dirs libs/muse-softfloat/include and core,
 * sources: every C file under core, this file, and the
 * muse-softfloat sources, output build/flashslot_test. Exit
 * status 0 iff every check passes.
 *
 * The device under test is pure slot arithmetic; flash itself is
 * a RAM fake with real NOR semantics: erase sets 0xFF per sector
 * (and counts cycles per slot), program only clears bits
 * (dst &= src) in whole 256 B pages, and a bug in the fake
 * (unaligned or out-of-region op) aborts the test rather than
 * being silently absorbed. Writes in the tests go through the
 * same erase -> page-program (0xFF-padded tail) sequence
 * board/ckpt_flash.c performs on device, and slot scans use
 * toy_ckpt_slot_probe() exactly as the backend does. Torn writes
 * are simulated by stopping the program sequence at a chosen
 * byte count - the power-cut shape the A/B design exists for.
 *
 * Coverage: geometry for 2/4/16 MiB parts + rejection cases,
 * blank device, rotation across wrap with balanced erases,
 * newest-valid fallback after a torn newest write (mid-header,
 * mid-payload, final-page), AND-only reprogramming without
 * erase, step_count ordering rules incl. ties, 4-slot ring
 * generalisation, and page-rounding arithmetic.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "toy_cart.h"
#include "toy_ckpt.h"
#include "toy_ckpt_slot.h"

static int failures = 0;
static int checks = 0;

static void check(int ok, const char *what)
{
    checks++;
    printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) failures++;
}

static void *xcalloc(size_t n)
{
    void *p = calloc(1, n ? n : 1);
    if (!p) { fprintf(stderr, "oom\n"); exit(2); }
    return p;
}

/* ---- RAM fake NOR flash ------------------------------------- */

/* Sized for the widest layout under test (4 cap slots). */
#define FAKE_MAX_SLOTS 4u
static unsigned char fake_flash[FAKE_MAX_SLOTS * 69632u];
static uint32_t fake_erases[FAKE_MAX_SLOTS];
static toy_ckpt_slot_layout_t fake_layout;

static void fake_fatal(const char *what)
{
    fprintf(stderr, "fake flash misuse: %s\n", what);
    exit(2);
}

/* (Re)initialise the fake for a flash size + slot count: the
 * whole fake buffer becomes the slot region view (tests address
 * slots relative to the region base, mirroring XIP reads). */
static void fake_init(uint32_t flash_bytes, uint32_t slot_count)
{
    if (!toy_ckpt_slot_layout_init(&fake_layout, flash_bytes, 0u,
                                   TOY_CKPT_SLOT_SECTOR_BYTES,
                                   TOY_CKPT_SLOT_PAGE_BYTES,
                                   TOY_CKPT_MAX_IMAGE_SIZE, slot_count))
        fake_fatal("layout init");
    if (fake_layout.total_bytes > sizeof fake_flash)
        fake_fatal("region larger than fake buffer");
    memset(fake_flash, 0xff, sizeof fake_flash);
    memset(fake_erases, 0, sizeof fake_erases);
}

static unsigned char *fake_slot(uint32_t index)
{
    if (index >= fake_layout.slot_count)
        fake_fatal("slot index");
    return fake_flash + index * fake_layout.slot_bytes;
}

/* NOR erase: whole slot to 0xFF, one cycle per sector counted
 * as one per slot erase. */
static void fake_erase_slot(uint32_t index)
{
    memset(fake_slot(index), 0xff, fake_layout.slot_bytes);
    fake_erases[index]++;
}

/* NOR program: page-aligned, page multiples, bits only 1 -> 0. */
static void fake_program(uint32_t index, uint32_t slot_off,
                         const unsigned char *data, uint32_t len)
{
    if ((slot_off % TOY_CKPT_SLOT_PAGE_BYTES) != 0u ||
        (len % TOY_CKPT_SLOT_PAGE_BYTES) != 0u ||
        slot_off + len > fake_layout.slot_bytes)
        fake_fatal("program alignment/range");
    unsigned char *dst = fake_slot(index) + slot_off;
    for (uint32_t i = 0; i < len; i++)
        dst[i] = (unsigned char)(dst[i] & data[i]);
}

/* The board write sequence over the fake: erase the slot, then
 * program full pages + a 0xFF-padded tail page. When torn_at >=
 * 0, "power" is cut after that many image bytes have been
 * programmed (page granular, the partial page's unwritten tail
 * stays 0xFF); when erase is 0 the erase is skipped on purpose
 * (the AND-semantics test). */
static void fake_write(uint32_t index, const unsigned char *image,
                       uint32_t image_len, long torn_at, int erase)
{
    if (erase)
        fake_erase_slot(index);
    uint32_t limit = image_len;
    if (torn_at >= 0 && (uint32_t)torn_at < limit)
        limit = (uint32_t)torn_at;
    uint32_t off = 0;
    while (off < limit) {
        unsigned char page[TOY_CKPT_SLOT_PAGE_BYTES];
        memset(page, 0xff, sizeof page);
        uint32_t chunk = limit - off;
        if (chunk > TOY_CKPT_SLOT_PAGE_BYTES)
            chunk = TOY_CKPT_SLOT_PAGE_BYTES;
        memcpy(page, image + off, chunk);
        fake_program(index, off, page, TOY_CKPT_SLOT_PAGE_BYTES);
        off += TOY_CKPT_SLOT_PAGE_BYTES;
    }
}

/* Probe all fake slots the way board/ckpt_flash.c scans XIP. */
static int fake_newest(const toy_cart_t *expected,
                       toy_ckpt_slot_candidate_t *cands_out)
{
    toy_ckpt_slot_candidate_t cands[FAKE_MAX_SLOTS];
    for (uint32_t i = 0; i < fake_layout.slot_count; i++)
        toy_ckpt_slot_probe(fake_slot(i), fake_layout.slot_bytes, expected,
                            &cands[i], NULL);
    if (cands_out)
        memcpy(cands_out, cands,
               fake_layout.slot_count * sizeof cands[0]);
    return toy_ckpt_slot_newest(cands, fake_layout.slot_count);
}

static int fake_next(const toy_cart_t *expected)
{
    toy_ckpt_slot_candidate_t cands[FAKE_MAX_SLOTS];
    fake_newest(expected, cands);
    return toy_ckpt_slot_next(cands, fake_layout.slot_count);
}

/* ---- image factory (real serialiser, real carts) ------------ */

static unsigned char img[TOY_CKPT_MAX_IMAGE_SIZE];

static size_t make_image(const char *cart_name, uint64_t steps)
{
    const toy_cart_t *cart = toy_cart_by_name(cart_name);
    if (!cart)
        fake_fatal("unknown cart");
    void *st = xcalloc(cart->state_size);
    cart->reset(st);
    for (uint64_t i = 0; i < steps; i++)
        cart->step(st);
    size_t len = 0;
    if (toy_ckpt_serialize(cart, st, steps, img, sizeof img, &len) !=
        TOY_CKPT_OK)
        fake_fatal("serialize");
    free(st);
    return len;
}

/* ---- (a) geometry -------------------------------------------- */

static void test_geometry(void)
{
    printf("geometry (cap-sized slots at flash end)\n");
    check(TOY_CKPT_MAX_IMAGE_SIZE == 65600u, "max image is 65,600 B");
    check(toy_ckpt_slot_size_for(65600u, 4096u) == 69632u,
          "cap slot = 17 sectors = 69,632 B");
    check(toy_ckpt_slot_size_for(14024u, 4096u) == 16384u,
          "fig8-sized slot would be 16,384 B (not used: sized for cap)");
    check(toy_ckpt_slot_size_for(4096u, 4096u) == 4096u &&
          toy_ckpt_slot_size_for(4097u, 4096u) == 8192u,
          "sector rounding");
    check(toy_ckpt_slot_size_for(0u, 4096u) == 0u &&
          toy_ckpt_slot_size_for(100u, 0u) == 0u &&
          toy_ckpt_slot_size_for(100u, 3000u) == 0u,
          "bad slot-size inputs give 0");
    check(toy_ckpt_slot_program_bytes(14024u, 256u) == 14080u,
          "fig8 image programs 55 pages = 14,080 B");
    check(toy_ckpt_slot_program_bytes(7192u, 256u) == 7424u,
          "pendulum image programs 29 pages = 7,424 B");
    check(toy_ckpt_slot_program_bytes(65600u, 256u) == 65792u,
          "cap image programs 257 pages = 65,792 B");
    check(toy_ckpt_slot_program_bytes(256u, 256u) == 256u &&
          toy_ckpt_slot_program_bytes(0u, 256u) == 0u,
          "page rounding edges");

    toy_ckpt_slot_layout_t lay;
    uint32_t off = 0;
    check(toy_ckpt_slot_layout_default(&lay, 2u * 1024u * 1024u, 0u) == 1 &&
          lay.slot_bytes == 69632u && lay.slot_count == 2u &&
          lay.total_bytes == 139264u && lay.base_offset == 0x1de000u &&
          toy_ckpt_slot_offset(&lay, 0u, &off) == 1 && off == 0x1de000u &&
          toy_ckpt_slot_offset(&lay, 1u, &off) == 1 && off == 0x1ef000u &&
          off + lay.slot_bytes == 0x200000u,
          "2 MiB: slots at 0x1DE000 / 0x1EF000, end at flash end");
    check(toy_ckpt_slot_layout_default(&lay, 16u * 1024u * 1024u, 0u) == 1 &&
          lay.base_offset == 0xfde000u &&
          toy_ckpt_slot_offset(&lay, 1u, &off) == 1 && off == 0xfef000u &&
          off + lay.slot_bytes == 0x1000000u,
          "16 MiB: slots at 0xFDE000 / 0xFEF000");
    check(toy_ckpt_slot_layout_default(&lay, 4u * 1024u * 1024u, 0u) == 1 &&
          lay.base_offset == 0x3de000u,
          "4 MiB (pico2 header default): base 0x3DE000");
    check(toy_ckpt_slot_offset(&lay, 2u, &off) == 0,
          "slot index past count rejected");

    check(toy_ckpt_slot_layout_default(&lay, 128u * 1024u, 0u) == 0,
          "flash smaller than the A/B pair rejected");
    check(toy_ckpt_slot_layout_default(&lay, 2u * 1024u * 1024u,
                                       2u * 1024u * 1024u - 100000u) == 0,
          "firmware reserve overlapping the slots rejected");
    check(toy_ckpt_slot_layout_default(&lay, 2u * 1024u * 1024u,
                                       1024u * 1024u) == 1,
          "1 MiB firmware reserve still fits on 2 MiB");
    check(toy_ckpt_slot_layout_init(&lay, 2u * 1024u * 1024u, 0u, 4096u, 256u,
                                    TOY_CKPT_MAX_IMAGE_SIZE, 0u) == 0 &&
          toy_ckpt_slot_layout_init(&lay, 2u * 1024u * 1024u, 0u, 4096u, 256u,
                                    TOY_CKPT_MAX_IMAGE_SIZE, 17u) == 0 &&
          toy_ckpt_slot_layout_init(&lay, 2u * 1024u * 1024u, 0u, 4096u, 8192u,
                                    TOY_CKPT_MAX_IMAGE_SIZE, 2u) == 0 &&
          toy_ckpt_slot_layout_init(&lay, 0u, 0u, 4096u, 256u,
                                    TOY_CKPT_MAX_IMAGE_SIZE, 2u) == 0 &&
          toy_ckpt_slot_layout_init(NULL, 2u * 1024u * 1024u, 0u, 4096u, 256u,
                                    TOY_CKPT_MAX_IMAGE_SIZE, 2u) == 0,
          "bad geometries rejected (count 0/huge, page>sector, no flash)");
    check(toy_ckpt_slot_layout_init(&lay, 2000000u, 0u, 4096u, 256u,
                                    TOY_CKPT_MAX_IMAGE_SIZE, 2u) == 1 &&
          (lay.flash_bytes % 4096u) == 0u &&
          lay.base_offset + lay.total_bytes == lay.flash_bytes,
          "non-sector-multiple flash size rounds down, slots at usable end");
    check(toy_ckpt_slot_layout_init(&lay, 16u * 1024u * 1024u, 0u, 4096u, 256u,
                                    TOY_CKPT_MAX_IMAGE_SIZE, 4u) == 1 &&
          lay.total_bytes == 278528u && lay.base_offset == 0xfbc000u,
          "4-slot ring variant: 278,528 B at 0xFBC000 on 16 MiB");
}

/* ---- (b) blank device + first saves -------------------------- */

static void test_blank_and_first(void)
{
    printf("blank device, first saves\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    fake_init(2u * 1024u * 1024u, 2u);
    toy_ckpt_slot_candidate_t cands[FAKE_MAX_SLOTS];
    check(fake_newest(fig8, cands) == -1 &&
          toy_ckpt_slot_next(cands, 2u) == 0,
          "blank device: no newest, next = slot 0");
    check(cands[0].blank == 1 && cands[1].blank == 1 &&
          cands[0].valid == 0 && cands[0].err == TOY_CKPT_ERR_BAD_MAGIC,
          "blank slots report blank + BAD_MAGIC");
    check(toy_ckpt_slot_is_blank(fake_slot(0), fake_layout.slot_bytes) == 1 &&
          toy_ckpt_slot_is_blank(fake_slot(0), 0u) == 0 &&
          toy_ckpt_slot_is_blank(NULL, 64u) == 0,
          "is_blank semantics");

    size_t len = make_image("fig8", 100u);
    fake_write(0u, img, (uint32_t)len, -1, 1);
    check(fake_newest(fig8, cands) == 0 && cands[0].step_count == 100u &&
          cands[0].image_len == len && fake_next(fig8) == 1,
          "first save @100: newest 0, next 1");
    check(memcmp(fake_slot(0), img, len) == 0 &&
          toy_ckpt_slot_is_blank(fake_slot(0) + len,
                                 (size_t)toy_ckpt_slot_program_bytes(
                                     (uint32_t)len, 256u) - len) == 1,
          "image bytes verbatim, page pad stays 0xFF");

    len = make_image("fig8", 200u);
    fake_write(1u, img, (uint32_t)len, -1, 1);
    check(fake_newest(fig8, cands) == 1 && cands[1].step_count == 200u &&
          fake_next(fig8) == 0,
          "second save @200: newest 1, next wraps to 0");
}

/* ---- (c) rotation across wrap -------------------------------- */

static void test_rotation(void)
{
    printf("rotation across wrap (A/B and 4-ring)\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");

    fake_init(2u * 1024u * 1024u, 2u);
    static const int want_slot[6] = { 0, 1, 0, 1, 0, 1 };
    int order_ok = 1, newest_ok = 1;
    for (int k = 0; k < 6; k++) {
        int target = fake_next(fig8);
        if (target != want_slot[k])
            order_ok = 0;
        size_t len = make_image("fig8", (uint64_t)(100 * (k + 1)));
        fake_write((uint32_t)target, img, (uint32_t)len, -1, 1);
        if (fake_newest(fig8, NULL) != target)
            newest_ok = 0;
    }
    check(order_ok && newest_ok,
          "6 saves rotate 0,1,0,1,0,1 and each is newest after writing");
    check(fake_erases[0] == 3u && fake_erases[1] == 3u,
          "erase cycles balance across slots (3 + 3 after 6 saves)");

    fake_init(16u * 1024u * 1024u, 4u);
    static const int want4[6] = { 0, 1, 2, 3, 0, 1 };
    order_ok = 1;
    for (int k = 0; k < 6; k++) {
        int target = fake_next(fig8);
        if (target != want4[k])
            order_ok = 0;
        size_t len = make_image("fig8", (uint64_t)(50 * (k + 1)));
        fake_write((uint32_t)target, img, (uint32_t)len, -1, 1);
    }
    check(order_ok && fake_newest(fig8, NULL) == 1 &&
          fake_erases[0] == 2u && fake_erases[1] == 2u &&
          fake_erases[2] == 1u && fake_erases[3] == 1u,
          "4-slot ring rotates 0,1,2,3,0,1 with spread erases");
}

/* ---- (d) torn newest falls back ------------------------------ */

static void test_torn(void)
{
    printf("torn newest write falls back to previous valid\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    fake_init(2u * 1024u * 1024u, 2u);
    size_t len100 = make_image("fig8", 100u);
    unsigned char img100[TOY_CKPT_MAX_IMAGE_SIZE];
    memcpy(img100, img, len100);
    fake_write(0u, img100, (uint32_t)len100, -1, 1);

    /* Power cuts during the @300 write to slot 1, at three
     * points; each time the @100 image must stay newest. */
    size_t len300 = make_image("fig8", 300u);
    static const long cuts[4] = { 0, 32, 7000, 14000 };
    static const char *cut_names[4] = {
        "cut before any program (erased target)",
        "cut mid-header (32 B programmed)",
        "cut mid-payload (7,000 B programmed)",
        "cut in final page (14,000 of 14,024 B)",
    };
    for (int k = 0; k < 4; k++) {
        fake_write(1u, img, (uint32_t)len300, cuts[k], 1);
        toy_ckpt_slot_candidate_t cands[FAKE_MAX_SLOTS];
        int best = fake_newest(fig8, cands);
        check(best == 0 && cands[0].step_count == 100u &&
              cands[1].valid == 0 &&
              memcmp(fake_slot(0), img100, len100) == 0,
              cut_names[k]);
    }

    /* The next real save reuses the torn slot (rotation target
     * is still slot 1) and then wins. */
    check(fake_next(fig8) == 1, "after torn writes, next is still the torn slot");
    fake_write(1u, img, (uint32_t)len300, -1, 1);
    check(fake_newest(fig8, NULL) == 1, "completed @300 write becomes newest");

    /* A torn erase-but-never-programmed newest slot (blank)
     * beside a valid older slot. */
    fake_erase_slot(1u);
    check(fake_newest(fig8, NULL) == 0,
          "erased-only slot skipped, older valid wins");
}

/* ---- (e) NOR semantics: no erase means AND garbage ----------- */

static void test_and_semantics(void)
{
    printf("AND-only programming (erase is mandatory)\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    fake_init(2u * 1024u * 1024u, 2u);
    size_t len_a = make_image("fig8", 100u);
    unsigned char a[TOY_CKPT_MAX_IMAGE_SIZE];
    memcpy(a, img, len_a);
    fake_write(0u, a, (uint32_t)len_a, -1, 1);

    /* Reprogram the same bytes without erase: x & x == x, the
     * one harmless rewrite. */
    fake_write(0u, a, (uint32_t)len_a, -1, 0);
    check(fake_newest(fig8, NULL) == 0,
          "reprogramming identical bytes without erase is a no-op");

    /* Program a different image over it without erase: the
     * bitwise AND of the two images must fail validation. */
    size_t len_b = make_image("fig8", 500u);
    fake_write(0u, img, (uint32_t)len_b, -1, 0);
    toy_ckpt_slot_candidate_t cands[FAKE_MAX_SLOTS];
    check(fake_newest(fig8, cands) == -1 && cands[0].valid == 0,
          "AND of two images without erase rejects (no valid slot)");

    /* Programming all-0xFF changes nothing (bits only clear). */
    fake_erase_slot(0u);
    fake_write(0u, a, (uint32_t)len_a, -1, 1);
    unsigned char ff[256];
    memset(ff, 0xff, sizeof ff);
    fake_program(0u, 0u, ff, 256u);
    check(fake_newest(fig8, NULL) == 0,
          "programming 0xFF over an image changes nothing");
}

/* ---- (f) ordering rules -------------------------------------- */

static void test_ordering(void)
{
    printf("step_count ordering rules\n");
    /* Hand-built candidates: the selection function itself. */
    toy_ckpt_slot_candidate_t c[3];
    memset(c, 0, sizeof c);
    check(toy_ckpt_slot_newest(c, 3u) == -1 && toy_ckpt_slot_next(c, 3u) == 0,
          "no valid candidates: newest -1, next 0");
    check(toy_ckpt_slot_newest(NULL, 3u) == -1 &&
          toy_ckpt_slot_next_after(0, 0u) == -1 &&
          toy_ckpt_slot_next_after(-1, 2u) == 0 &&
          toy_ckpt_slot_next_after(1, 2u) == 0,
          "degenerate counts / next_after wrap");
    c[1].valid = 1; c[1].step_count = 50u;
    c[0].valid = 1; c[0].step_count = 200u;
    check(toy_ckpt_slot_newest(c, 3u) == 0,
          "higher step_count in the lower slot wins (order, not recency)");
    c[2].valid = 1; c[2].step_count = 200u;
    check(toy_ckpt_slot_newest(c, 3u) == 0,
          "step_count tie resolves to the lowest slot index");
    c[0].valid = 0;
    check(toy_ckpt_slot_newest(c, 3u) == 2 && toy_ckpt_slot_next(c, 3u) == 0,
          "invalid high-water slot skipped; next wraps past newest");

    toy_ckpt_info_t info;
    memset(&info, 0, sizeof info);
    info.step_count = 42u;
    info.state_size = 100u;
    toy_ckpt_slot_candidate_t m =
        toy_ckpt_slot_candidate_from(TOY_CKPT_OK, &info, 0);
    check(m.valid == 1 && m.step_count == 42u && m.image_len == 164u,
          "candidate_from(OK) carries step_count + image length");
    m = toy_ckpt_slot_candidate_from(TOY_CKPT_ERR_PAYLOAD_CRC, &info, 0);
    check(m.valid == 0 && m.step_count == 0u && m.image_len == 0u &&
          m.err == TOY_CKPT_ERR_PAYLOAD_CRC,
          "candidate_from(error) is invalid and carries the error");

    /* The cart-switch caveat, pinned as behaviour: scoped scans
     * (expected != NULL) ignore the other cart entirely; an
     * unscoped scan picks the higher step_count across carts.
     * Board write rotation always scopes to the image's cart. */
    const toy_cart_t *pendulum = toy_cart_by_name("pendulum");
    fake_init(2u * 1024u * 1024u, 2u);
    size_t len = make_image("pendulum", 900u);
    fake_write(0u, img, (uint32_t)len, -1, 1);
    len = make_image("fig8", 10u);
    fake_write(1u, img, (uint32_t)len, -1, 1);
    check(fake_newest(pendulum, NULL) == 0 &&
          fake_newest(toy_cart_by_name("fig8"), NULL) == 1,
          "cart-scoped scans each find their own cart's slot");
    check(fake_newest(NULL, NULL) == 0,
          "unscoped scan follows raw step_count (900 > 10): the documented caveat");

    /* A payload-corrupted slot with a huge (forged-by-damage)
     * header must lose to an intact modest one: probe validates
     * payload CRC + hash, never trusts the header alone. */
    fake_init(2u * 1024u * 1024u, 2u);
    len = make_image("fig8", 100u);
    fake_write(1u, img, (uint32_t)len, -1, 1);
    len = make_image("fig8", 999u);
    fake_write(0u, img, (uint32_t)len, -1, 1);
    fake_slot(0)[500] = (unsigned char)(fake_slot(0)[500] ^ 0x40u);
    toy_ckpt_slot_candidate_t cands[FAKE_MAX_SLOTS];
    check(fake_newest(toy_cart_by_name("fig8"), cands) == 1 &&
          cands[0].valid == 0 && cands[0].err == TOY_CKPT_ERR_PAYLOAD_CRC &&
          cands[1].step_count == 100u,
          "@999 slot with flipped payload byte loses to intact @100");
}

/* ---- (g) probe edges ----------------------------------------- */

static void test_probe_edges(void)
{
    printf("probe edges\n");
    fake_init(2u * 1024u * 1024u, 2u);
    size_t len = make_image("pendulum", 7u);
    toy_ckpt_slot_candidate_t cand;
    toy_ckpt_info_t info;
    check(toy_ckpt_slot_probe(img, len, NULL, &cand, &info) == TOY_CKPT_OK &&
          cand.valid == 1 && cand.image_len == len &&
          strcmp(info.cart_name, "pendulum") == 0,
          "probe over exactly-image-sized buffer validates");
    check(toy_ckpt_slot_probe(img, len - 1u, NULL, &cand, NULL) ==
              TOY_CKPT_ERR_LENGTH && cand.valid == 0,
          "probe with slot one byte short: LENGTH, not a read past the end");
    check(toy_ckpt_slot_probe(img, 10u, NULL, &cand, NULL) ==
              TOY_CKPT_ERR_TOO_SMALL,
          "probe over 10-byte buffer: TOO_SMALL");
    check(toy_ckpt_slot_probe(NULL, len, NULL, &cand, NULL) ==
              TOY_CKPT_ERR_NULL && cand.valid == 0,
          "probe NULL buffer: NULL error, candidate invalid");
    check(toy_ckpt_slot_probe(img, len, toy_cart_by_name("fig8"), &cand,
                              NULL) == TOY_CKPT_ERR_CART_MISMATCH,
          "probe with wrong expected cart: CART_MISMATCH");
}

int main(void)
{
    test_geometry();
    test_blank_and_first();
    test_rotation();
    test_torn();
    test_and_semantics();
    test_ordering();
    test_probe_edges();
    printf("flashslot_test: %d checks, %d failures - %s\n",
           checks, failures, failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}

/* ckpt_test.c - standalone host test for core/toy_ckpt.c.
 *
 * Deliberately NOT part of tools/run.sh (mirroring fft_test.c):
 * build and run it separately with the run.sh host flags -
 * gcc -O2 -std=c11 -Wall -Wextra, include dirs
 * libs/muse-softfloat/include and core, sources: every C
 * file under core, this file, and the muse-softfloat sources,
 * output build/ckpt_test. Exit status 0 iff every check passes.
 *
 * Coverage (docs/study-checkpoint.md section 8 A/B, host level):
 *  (a) CRC-32 against published check values.
 *  (b) Registry rules: every cart name fits the 16-byte field,
 *      every state_size fits the cap.
 *  (c) Golden header: 64 literal bytes computed independently
 *      of this code (Python binascii.crc32 + the FNV definition)
 *      must parse to the known fields, validate over an all-zero
 *      fig8 payload, and be reproduced byte-for-byte by the
 *      serialiser - a writer+reader pair sharing an offset or
 *      endianness bug cannot pass.
 *  (d) Round-trip of every registered cart at several step
 *      counts, including save -> load -> save byte identity.
 *  (e) Resume continuity: checkpoint at N, load into a fresh
 *      state, continue M steps; the hash stream must equal an
 *      uninterrupted N+M run (all carts short form; fig8 and
 *      the largest-state carts with a full reported stream).
 *  (f) Rejection matrix: each corruption must fail with the
 *      specific error the check order implies, torn writes at
 *      every prefix length must reject cleanly, and a rejected
 *      load must leave the destination state untouched.
 *  (g) Synthetic feed carts: checkpoint at N, load, seek a
 *      fresh proof source to N * samples_per_step and resume -
 *      the hash stream must equal the uninterrupted fed run
 *      (gpend, heartbeat, seismo deep; every feed cart short),
 *      and a TOYBANK1 bank of gpend written by a feeding loop
 *      verifies by replay while a CRC-repaired tampered
 *      keyframe fails with TOY_BANK_ERR_REPLAY_HASH. A feed
 *      descriptor with a non-SYNTHETIC proof kind still
 *      refuses to bank.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "toy_cart.h"
#include "toy_ckpt.h"
#include "toy_bank.h"

static int failures = 0;
static int checks = 0;

static void check(int ok, const char *what)
{
    checks++;
    printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) failures++;
}

static void expect_err(const char *what, toy_ckpt_err_t got, toy_ckpt_err_t want)
{
    checks++;
    int ok = got == want;
    printf("  [%s] %s (got %s, want %s)\n", ok ? "ok" : "FAIL", what,
           toy_ckpt_strerror(got), toy_ckpt_strerror(want));
    if (!ok) failures++;
}

static void *xcalloc(size_t n)
{
    void *p = calloc(1, n ? n : 1);
    if (!p) { fprintf(stderr, "oom\n"); exit(2); }
    return p;
}

static unsigned char img_a[TOY_CKPT_MAX_IMAGE_SIZE + 16];
static unsigned char img_b[TOY_CKPT_MAX_IMAGE_SIZE + 16];

static void put_u32le(unsigned char *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) p[i] = (unsigned char)(v >> (8 * i));
}
static void put_u64le(unsigned char *p, uint64_t v)
{
    for (int i = 0; i < 8; i++) p[i] = (unsigned char)(v >> (8 * i));
}
static uint32_t get_u32le(const unsigned char *p)
{
    uint32_t v = 0;
    for (int i = 3; i >= 0; i--) v = (v << 8) | (uint32_t)p[i];
    return v;
}
/* Repair the header CRC after a deliberate header mutation, so
 * the loader reaches the check the mutation is aimed at. */
static void fix_header_crc(unsigned char *img)
{
    put_u32le(img + 60, toy_ckpt_crc32(img, 60));
}

static void run_steps(const toy_cart_t *cart, void *st, uint64_t n)
{
    for (uint64_t i = 0; i < n; i++) cart->step(st);
}

/* ---- (a) CRC-32 published check values ---------------------- */

static void test_crc(void)
{
    printf("crc32 published check values\n");
    check(toy_ckpt_crc32("", 0) == 0x00000000u, "crc32(\"\") == 00000000");
    check(toy_ckpt_crc32("123456789", 9) == 0xcbf43926u,
          "crc32(\"123456789\") == cbf43926");
    const unsigned char zero[1] = { 0 };
    check(toy_ckpt_crc32(zero, 1) == 0xd202ef8du, "crc32(\\x00) == d202ef8d");
    const char *fox = "The quick brown fox jumps over the lazy dog";
    check(toy_ckpt_crc32(fox, strlen(fox)) == 0x414fa339u,
          "crc32(quick brown fox) == 414fa339");
}

/* ---- (b) registry rules ------------------------------------- */

static void test_registry(void)
{
    printf("registry rules (name cap, state cap, version 0)\n");
    for (int i = 0; i < toy_cart_count; i++) {
        const toy_cart_t *c = toy_carts[i];
        size_t n = strlen(c->name);
        int ascii = 1;
        for (size_t k = 0; k < n; k++)
            if ((unsigned char)c->name[k] < 0x21u || (unsigned char)c->name[k] > 0x7eu)
                ascii = 0;
        char what[128];
        snprintf(what, sizeof what, "%.24s: name %zu chars ascii, state %zu B, version %u",
                 c->name, n, c->state_size, c->state_version);
        check(n >= 1 && n <= TOY_CKPT_NAME_CAP && ascii &&
              c->state_size >= 1 && c->state_size <= TOY_MAX_STATE_SIZE &&
              c->state_version == 0 &&
              toy_ckpt_image_size(c) == TOY_CKPT_HEADER_SIZE + c->state_size, what);
    }
}

/* ---- (c) golden header -------------------------------------- */

/* fig8, state_size 13960, state_version 0, step_count 1000,
 * payload = 13960 zero bytes. state_hash dbbaebe4103b1fe3,
 * payload_crc32 0f8b2be1, header_crc32 862bf0bb - computed with
 * Python (binascii.crc32, FNV-1a per core/toy_carts.c), not by
 * running the code under test. */
static const unsigned char golden_header[64] = {
    0x54, 0x4f, 0x59, 0x43, 0x4b, 0x50, 0x54, 0x31, 0x01, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x66, 0x69, 0x67, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x88, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xe3, 0x1f, 0x3b, 0x10, 0xe4, 0xeb, 0xba, 0xdb, 0xe1, 0x2b, 0x8b, 0x0f, 0xbb, 0xf0, 0x2b, 0x86,
};

static void test_golden(void)
{
    printf("golden header (fig8, zero payload, step 1000)\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    check(fig8 && fig8->state_size == 13960, "fig8 state_size is 13960");
    if (!fig8) return;

    toy_ckpt_info_t info;
    expect_err("golden header parses",
               toy_ckpt_parse_header(golden_header, 64, &info), TOY_CKPT_OK);
    check(strcmp(info.cart_name, "fig8") == 0, "golden name == fig8");
    check(info.format_version == 1 && info.header_size == 64 && info.flags == 0,
          "golden version/header_size/flags");
    check(info.state_size == 13960 && info.state_version == 0, "golden sizes/versions");
    check(info.step_count == 1000, "golden step_count == 1000");
    check(info.state_hash == 0xdbbaebe4103b1fe3ull, "golden state_hash");
    check(info.payload_crc32 == 0x0f8b2be1u, "golden payload_crc32");
    check(info.header_crc32 == 0x862bf0bbu, "golden header_crc32");

    /* Full golden image: header + zero payload. */
    size_t total = TOY_CKPT_HEADER_SIZE + fig8->state_size;
    memset(img_a, 0, total);
    memcpy(img_a, golden_header, 64);
    const toy_cart_t *cart = NULL;
    expect_err("golden image validates",
               toy_ckpt_validate(img_a, total, NULL, &info, &cart), TOY_CKPT_OK);
    check(cart == fig8, "golden image resolves to fig8 descriptor");

    /* The serialiser must reproduce those exact bytes. */
    void *zero_state = xcalloc(fig8->state_size);
    size_t out_len = 0;
    expect_err("serialise zero state",
               toy_ckpt_serialize(fig8, zero_state, 1000, img_b, sizeof img_b, &out_len),
               TOY_CKPT_OK);
    check(out_len == total && memcmp(img_a, img_b, total) == 0,
          "serialiser output == golden image byte-for-byte");

    /* And loading it installs zeros + the step count. */
    void *st = xcalloc(fig8->state_size);
    memset(st, 0xa5, fig8->state_size);
    uint64_t steps = 0;
    expect_err("load golden image",
               toy_ckpt_load(img_a, total, NULL, st, fig8->state_size, NULL, &steps),
               TOY_CKPT_OK);
    check(steps == 1000 && memcmp(st, zero_state, fig8->state_size) == 0,
          "loaded state is zeros at step 1000");
    free(zero_state);
    free(st);
}

/* ---- (d) round-trip every cart ------------------------------ */

static void test_roundtrip(void)
{
    printf("round-trip every cart (steps 0, 1, 17, 100)\n");
    static const uint64_t step_counts[] = { 0, 1, 17, 100 };
    for (int i = 0; i < toy_cart_count; i++) {
        const toy_cart_t *c = toy_carts[i];
        void *a = xcalloc(c->state_size);
        void *b = xcalloc(c->state_size);
        for (size_t k = 0; k < sizeof step_counts / sizeof step_counts[0]; k++) {
            uint64_t n = step_counts[k];
            memset(a, 0, c->state_size);
            c->reset(a);
            run_steps(c, a, n);
            size_t len_a = 0, len_b = 0;
            toy_ckpt_err_t e1 = toy_ckpt_serialize(c, a, n, img_a, sizeof img_a, &len_a);
            memset(b, 0xa5, c->state_size);
            const toy_cart_t *loaded = NULL;
            uint64_t steps = 0;
            toy_ckpt_err_t e2 = toy_ckpt_load(img_a, len_a, c, b, c->state_size,
                                              &loaded, &steps);
            toy_ckpt_err_t e3 = toy_ckpt_serialize(c, b, steps, img_b, sizeof img_b, &len_b);
            char what[160];
            snprintf(what, sizeof what,
                     "%s @%llu: round-trip + re-save identity, hash %016llx",
                     c->name, (unsigned long long)n,
                     (unsigned long long)toy_hash_state(a, c->state_size));
            check(e1 == TOY_CKPT_OK && e2 == TOY_CKPT_OK && e3 == TOY_CKPT_OK &&
                  loaded == c && steps == n &&
                  len_a == TOY_CKPT_HEADER_SIZE + c->state_size && len_b == len_a &&
                  memcmp(a, b, c->state_size) == 0 &&
                  memcmp(img_a, img_b, len_a) == 0, what);
        }
        free(a);
        free(b);
    }
}

/* ---- (e) resume continuity ---------------------------------- */

/* Uninterrupted N+M vs checkpoint-at-N + resume for M. Compares
 * the hash at every `every` boundary after N and at the end.
 * Returns 1 on equality; prints the stream when verbose. */
static int resume_stream_equal(const toy_cart_t *c, uint64_t n, uint64_t m,
                               uint64_t every, int verbose)
{
    void *ref = xcalloc(c->state_size);
    void *res = xcalloc(c->state_size);
    c->reset(ref);
    run_steps(c, ref, n);

    size_t len = 0;
    if (toy_ckpt_serialize(c, ref, n, img_a, sizeof img_a, &len) != TOY_CKPT_OK) return 0;
    memset(res, 0xa5, c->state_size);
    uint64_t steps = 0;
    if (toy_ckpt_load(img_a, len, NULL, res, c->state_size, NULL, &steps) != TOY_CKPT_OK)
        return 0;
    int ok = steps == n && memcmp(ref, res, c->state_size) == 0;
    if (verbose)
        printf("    %s checkpoint @%llu hash %016llx\n", c->name,
               (unsigned long long)n,
               (unsigned long long)toy_hash_state(res, c->state_size));
    for (uint64_t s = 1; s <= m; s++) {
        c->step(ref);
        c->step(res);
        uint64_t abs_step = n + s;
        if ((every && (abs_step % every) == 0) || s == m) {
            uint64_t hr = toy_hash_state(ref, c->state_size);
            uint64_t hs = toy_hash_state(res, c->state_size);
            if (hr != hs) ok = 0;
            if (verbose)
                printf("    step %llu uninterrupted %016llx resumed %016llx %s\n",
                       (unsigned long long)abs_step,
                       (unsigned long long)hr, (unsigned long long)hs,
                       hr == hs ? "==" : "!=");
        }
    }
    free(ref);
    free(res);
    return ok;
}

static void test_resume(void)
{
    printf("resume continuity: all carts (37 + 63 vs 100)\n");
    for (int i = 0; i < toy_cart_count; i++) {
        const toy_cart_t *c = toy_carts[i];
        char what[128];
        snprintf(what, sizeof what, "%s: resumed stream == uninterrupted", c->name);
        check(resume_stream_equal(c, 37, 63, 0, 0), what);
    }
    printf("resume continuity: full streams (fig8, lorenz, cloud; 300 + 200, every 50)\n");
    static const char *deep[] = { "fig8", "lorenz", "cloud" };
    for (size_t i = 0; i < sizeof deep / sizeof deep[0]; i++) {
        const toy_cart_t *c = toy_cart_by_name(deep[i]);
        char what[128];
        snprintf(what, sizeof what, "%s: 500-step stream suffix-equal after resume @300",
                 deep[i]);
        check(c && resume_stream_equal(c, 300, 200, 50, 1), what);
    }
}

/* ---- (f) rejection matrix ----------------------------------- */

static size_t base_len;
static unsigned char base_img[TOY_CKPT_MAX_IMAGE_SIZE + 16];

static toy_ckpt_err_t validate_mut(void)
{
    return toy_ckpt_validate(img_b, base_len, NULL, NULL, NULL);
}
#define FRESH() memcpy(img_b, base_img, base_len)

static void test_rejection(void)
{
    printf("rejection matrix (fig8 @50)\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    const toy_cart_t *pendulum = toy_cart_by_name("pendulum");
    void *st = xcalloc(fig8->state_size);
    fig8->reset(st);
    run_steps(fig8, st, 50);
    if (toy_ckpt_serialize(fig8, st, 50, base_img, sizeof base_img, &base_len)
            != TOY_CKPT_OK) {
        check(0, "build base image");
        free(st);
        return;
    }
    FRESH();
    expect_err("base image is valid", validate_mut(), TOY_CKPT_OK);

    FRESH(); img_b[0] ^= 0x01u;
    expect_err("bad magic", validate_mut(), TOY_CKPT_ERR_BAD_MAGIC);

    FRESH(); img_b[8] = 0; /* version 0; version is checked before CRC */
    expect_err("format_version 0", validate_mut(), TOY_CKPT_ERR_FORMAT_VERSION);
    FRESH(); img_b[8] = 2;
    expect_err("format_version 2", validate_mut(), TOY_CKPT_ERR_FORMAT_VERSION);

    FRESH(); img_b[10] = 63; /* header_size checked before CRC */
    expect_err("header_size 63", validate_mut(), TOY_CKPT_ERR_HEADER_SIZE);

    FRESH(); img_b[40] ^= 0x01u; /* step_count bit, header CRC now stale */
    expect_err("header field flipped", validate_mut(), TOY_CKPT_ERR_HEADER_CRC);
    FRESH(); img_b[60] ^= 0x01u; /* stored header CRC */
    expect_err("stored header_crc32 flipped", validate_mut(), TOY_CKPT_ERR_HEADER_CRC);

    FRESH(); img_b[100] ^= 0x40u; /* payload bit */
    expect_err("payload bit flipped", validate_mut(), TOY_CKPT_ERR_PAYLOAD_CRC);

    /* Same payload damage with the payload CRC repaired: intact
     * transport, wrong meaning - must reach the FNV check. */
    FRESH(); img_b[100] ^= 0x40u;
    put_u32le(img_b + 56, toy_ckpt_crc32(img_b + 64, fig8->state_size));
    fix_header_crc(img_b);
    expect_err("payload flipped, CRC repaired", validate_mut(), TOY_CKPT_ERR_STATE_HASH);

    FRESH(); img_b[48] ^= 0x01u; fix_header_crc(img_b); /* stored state_hash */
    expect_err("stored state_hash flipped", validate_mut(), TOY_CKPT_ERR_STATE_HASH);

    FRESH(); img_b[56] ^= 0x01u; fix_header_crc(img_b); /* stored payload_crc32 */
    expect_err("stored payload_crc32 flipped", validate_mut(), TOY_CKPT_ERR_PAYLOAD_CRC);

    expect_err("truncated to 0", toy_ckpt_validate(base_img, 0, NULL, NULL, NULL),
               TOY_CKPT_ERR_TOO_SMALL);
    expect_err("truncated mid-header (32)",
               toy_ckpt_validate(base_img, 32, NULL, NULL, NULL), TOY_CKPT_ERR_TOO_SMALL);
    expect_err("truncated at 63", toy_ckpt_validate(base_img, 63, NULL, NULL, NULL),
               TOY_CKPT_ERR_TOO_SMALL);
    expect_err("header only (64)", toy_ckpt_validate(base_img, 64, NULL, NULL, NULL),
               TOY_CKPT_ERR_LENGTH);
    expect_err("truncated mid-payload",
               toy_ckpt_validate(base_img, base_len / 2, NULL, NULL, NULL),
               TOY_CKPT_ERR_LENGTH);
    expect_err("truncated by one byte",
               toy_ckpt_validate(base_img, base_len - 1, NULL, NULL, NULL),
               TOY_CKPT_ERR_LENGTH);
    expect_err("one trailing byte",
               toy_ckpt_validate(base_img, base_len + 1, NULL, NULL, NULL),
               TOY_CKPT_ERR_LENGTH);

    /* Every proper prefix of a valid image rejects (torn write). */
    {
        int all_rejected = 1;
        for (size_t p = 0; p < base_len; p++)
            if (toy_ckpt_validate(base_img, p, NULL, NULL, NULL) == TOY_CKPT_OK)
                all_rejected = 0;
        check(all_rejected, "every prefix truncation (0..len-1) rejects");
    }

    memset(img_b, 0xff, base_len); /* erased flash */
    expect_err("all-0xFF image", validate_mut(), TOY_CKPT_ERR_BAD_MAGIC);
    memset(img_b, 0x00, base_len);
    expect_err("all-0x00 image", validate_mut(), TOY_CKPT_ERR_BAD_MAGIC);

    FRESH(); memcpy(img_b + 16, "nope", 4); img_b[20] = 0; fix_header_crc(img_b);
    expect_err("unknown cart name", validate_mut(), TOY_CKPT_ERR_UNKNOWN_CART);

    FRESH(); memset(img_b + 16, 'a', 16); fix_header_crc(img_b);
    expect_err("name with no NUL in field", validate_mut(), TOY_CKPT_ERR_BAD_NAME);

    expect_err("fig8 image, pendulum expected",
               toy_ckpt_validate(base_img, base_len, pendulum, NULL, NULL),
               TOY_CKPT_ERR_CART_MISMATCH);

    FRESH(); img_b[12] = 1; fix_header_crc(img_b);
    expect_err("flags = 1", validate_mut(), TOY_CKPT_ERR_FLAGS);

    FRESH(); put_u32le(img_b + 32, (uint32_t)fig8->state_size + 8u); fix_header_crc(img_b);
    expect_err("state_size != descriptor", validate_mut(), TOY_CKPT_ERR_STATE_SIZE);
    FRESH(); put_u32le(img_b + 32, 0u); fix_header_crc(img_b);
    expect_err("state_size = 0", validate_mut(), TOY_CKPT_ERR_STATE_SIZE);
    FRESH(); put_u32le(img_b + 32, TOY_MAX_STATE_SIZE + 8u); fix_header_crc(img_b);
    expect_err("state_size = cap + 8", validate_mut(), TOY_CKPT_ERR_STATE_SIZE);

    FRESH(); put_u32le(img_b + 36, 1u); fix_header_crc(img_b);
    expect_err("cart_state_version bumped in header", validate_mut(),
               TOY_CKPT_ERR_STATE_VERSION);

    /* Version-bump simulation at the writer: a descriptor copy
     * claiming layout version 1 writes an image this registry
     * (version 0) must reject on version, not load by size. */
    {
        toy_cart_t future = *fig8;
        future.state_version = 1;
        size_t flen = 0;
        toy_ckpt_err_t e = toy_ckpt_serialize(&future, st, 50, img_b, sizeof img_b, &flen);
        check(e == TOY_CKPT_OK && get_u32le(img_b + 36) == 1u,
              "serialiser stamps descriptor state_version (fake v1)");
        expect_err("v1 image read by v0 registry",
                   toy_ckpt_validate(img_b, flen, NULL, NULL, NULL),
                   TOY_CKPT_ERR_STATE_VERSION);
    }

    /* Serialise-side guards. */
    expect_err("serialise into short buffer",
               toy_ckpt_serialize(fig8, st, 0, img_b, base_len - 1, NULL),
               TOY_CKPT_ERR_OUT_TOO_SMALL);
    expect_err("serialise null cart",
               toy_ckpt_serialize(NULL, st, 0, img_b, sizeof img_b, NULL),
               TOY_CKPT_ERR_NULL);
    {
        toy_cart_t badname = *fig8;
        badname.name = "0123456789abcdef"; /* 16 chars: no room for the NUL */
        expect_err("serialise 16-char cart name",
                   toy_ckpt_serialize(&badname, st, 0, img_b, sizeof img_b, NULL),
                   TOY_CKPT_ERR_BAD_NAME);
        toy_cart_t badsize = *fig8;
        badsize.state_size = 0;
        expect_err("serialise zero state_size",
                   toy_ckpt_serialize(&badsize, st, 0, img_b, sizeof img_b, NULL),
                   TOY_CKPT_ERR_STATE_SIZE);
    }

    /* A rejected load leaves the destination state untouched. */
    {
        void *dst = xcalloc(fig8->state_size);
        void *ref = xcalloc(fig8->state_size);
        memset(dst, 0xa5, fig8->state_size);
        memset(ref, 0xa5, fig8->state_size);
        FRESH(); img_b[100] ^= 0x40u; /* payload damage */
        expect_err("load of damaged image",
                   toy_ckpt_load(img_b, base_len, NULL, dst, fig8->state_size, NULL, NULL),
                   TOY_CKPT_ERR_PAYLOAD_CRC);
        check(memcmp(dst, ref, fig8->state_size) == 0,
              "destination untouched after rejected load");
        expect_err("load into short state buffer",
                   toy_ckpt_load(base_img, base_len, NULL, dst,
                                 fig8->state_size - 1, NULL, NULL),
                   TOY_CKPT_ERR_OUT_TOO_SMALL);
        check(memcmp(dst, ref, fig8->state_size) == 0,
              "destination untouched after short-buffer load");
        free(dst);
        free(ref);
    }
    free(st);
}

/* ---- (g) synthetic feed carts: resume + bank ---------------- */

static int16_t feed_frame[TOY_SIG_MAX_FRAME_VALUES];

/* One runner boundary for absolute step s: feed -> apply -> step. */
static int feed_boundary(const toy_cart_t *c, void *st, toy_sig_source_t *src,
                         uint32_t *cursor, uint64_t s)
{
    if (toy_feed_step(c, st, src, feed_frame, TOY_SIG_MAX_FRAME_VALUES) != 0)
        return 0;
    toy_apply_proof_at(c, st, (uint32_t)s, cursor);
    c->step(st);
    return 1;
}

/* Uninterrupted fed N+M vs checkpoint-at-N + load + seek fresh
 * source to N * samples_per_step + resume M fed steps. */
static int feed_resume_equal(const toy_cart_t *c, uint64_t n, uint64_t m,
                             int verbose)
{
    void *ref = xcalloc(c->state_size);
    void *res = xcalloc(c->state_size);
    toy_sig_source_t src_ref, src_res;
    int ok = 1;
    if (toy_sig_proof_source_init(c, &src_ref) != 1) ok = 0;
    c->reset(ref);
    uint32_t cur_ref = 0, cur_res = 0;
    toy_apply_proof_at(c, ref, 0, &cur_ref);
    for (uint64_t s = 1; ok && s <= n; s++)
        ok = feed_boundary(c, ref, &src_ref, &cur_ref, s);

    size_t len = 0;
    if (toy_ckpt_serialize(c, ref, n, img_a, sizeof img_a, &len) != TOY_CKPT_OK)
        ok = 0;
    memset(res, 0xa5, c->state_size);
    uint64_t steps = 0;
    if (toy_ckpt_load(img_a, len, NULL, res, c->state_size, NULL, &steps)
            != TOY_CKPT_OK)
        ok = 0;
    if (toy_sig_proof_source_init(c, &src_res) != 1 ||
        toy_sig_seek_samples(&src_res,
                             steps * (uint64_t)c->sig_spec.samples_per_step) != 0)
        ok = 0;
    while (c->proof_inputs && cur_res < c->proof_input_count &&
           (uint64_t)c->proof_inputs[cur_res].step <= steps)
        cur_res++;
    if (steps != n || memcmp(ref, res, c->state_size) != 0) ok = 0;
    if (verbose)
        printf("    %s fed checkpoint @%llu hash %016llx (per_step %u, "
               "seek to sample %llu)\n", c->name, (unsigned long long)n,
               (unsigned long long)toy_hash_state(res, c->state_size),
               c->sig_spec.samples_per_step,
               (unsigned long long)(n * c->sig_spec.samples_per_step));
    for (uint64_t s = n + 1; ok && s <= n + m; s++) {
        ok = feed_boundary(c, ref, &src_ref, &cur_ref, s) &&
             feed_boundary(c, res, &src_res, &cur_res, s);
        uint64_t hr = toy_hash_state(ref, c->state_size);
        uint64_t hs = toy_hash_state(res, c->state_size);
        if (hr != hs) ok = 0;
        if (verbose && (s == n + m || (s % 50) == 0))
            printf("    step %llu uninterrupted %016llx resumed %016llx %s\n",
                   (unsigned long long)s, (unsigned long long)hr,
                   (unsigned long long)hs, hr == hs ? "==" : "!=");
    }
    free(ref);
    free(res);
    return ok;
}

static void test_feed_resume(void)
{
    printf("synthetic feed resume: every feed cart (37 + 63 fed steps)\n");
    int feed_carts = 0;
    for (int i = 0; i < toy_cart_count; i++) {
        const toy_cart_t *c = toy_carts[i];
        if (!c->feed) continue;
        feed_carts++;
        char what[160];
        snprintf(what, sizeof what,
                 "%s: seek-resumed fed stream == uninterrupted", c->name);
        check(c->sig_spec.proof_kind == TOY_SIG_KIND_SYNTHETIC &&
              feed_resume_equal(c, 37, 63, 0), what);
    }
    check(feed_carts == 4, "registry has the 4 feed carts");
    printf("synthetic feed resume: deep streams (300 + 200 fed steps)\n");
    static const char *deep[] = { "gpend", "heartbeat", "seismo" };
    for (size_t i = 0; i < sizeof deep / sizeof deep[0]; i++) {
        const toy_cart_t *c = toy_cart_by_name(deep[i]);
        char what[160];
        snprintf(what, sizeof what,
                 "%s: 500-step fed stream suffix-equal after resume @300",
                 deep[i]);
        check(c && feed_resume_equal(c, 300, 200, 1), what);
    }
    /* seismo at 10 samples/step: resuming at step 290 puts the
     * desk tap (sample 3000) inside the resumed segment. */
    {
        const toy_cart_t *c = toy_cart_by_name("seismo");
        check(c && feed_resume_equal(c, 290, 60, 1),
              "seismo: resume @290 crosses the tap exactly");
    }
}

static unsigned char bank_buf[TOY_BANK_BUDGET_BYTES];

/* Write a bank with the emu's feeding loop shape into bank_buf. */
static size_t build_feed_bank(const toy_cart_t *c, uint64_t total,
                              uint32_t every)
{
    uint32_t count = toy_bank_keyframe_count(total, every);
    size_t est = toy_bank_estimate_size(c, count);
    toy_bank_writer_t w;
    toy_sig_source_t src;
    size_t out = 0;
    void *st = xcalloc(c->state_size);
    if (!est || toy_bank_begin(&w, c, total, every, 0, 0, 0,
                               bank_buf, est) != TOY_BANK_OK)
        goto done;
    if (toy_sig_proof_source_init(c, &src) != 1) goto done;
    c->reset(st);
    {
        uint32_t cur = 0, next_kf = 0;
        toy_apply_proof_at(c, st, 0, &cur);
        for (uint64_t s = 0;; s++) {
            if (next_kf < count &&
                s == toy_bank_keyframe_step(total, every, next_kf)) {
                if (toy_bank_add_keyframe(&w, st, s, 0) != TOY_BANK_OK)
                    goto done;
                next_kf++;
            }
            if (s >= total) break;
            if (!feed_boundary(c, st, &src, &cur, s + 1)) goto done;
        }
    }
    if (toy_bank_finish(&w, 0, TOY_BANK_END_COMPLETE, &out) != TOY_BANK_OK)
        out = 0;
done:
    free(st);
    return out;
}

static void test_feed_bank(void)
{
    printf("synthetic feed banking (gpend 200 steps every 50, heartbeat too)\n");
    static const char *names[] = { "gpend", "heartbeat" };
    for (size_t t = 0; t < sizeof names / sizeof names[0]; t++) {
        const toy_cart_t *c = toy_cart_by_name(names[t]);
        size_t len = c ? build_feed_bank(c, 200, 50) : 0;
        char what[160];
        snprintf(what, sizeof what, "%s bank written by feeding loop (%zu bytes)",
                 names[t], len);
        check(len > 0, what);
        if (!len) continue;
        void *scratch = xcalloc(TOY_MAX_STATE_SIZE);
        uint32_t verified = 0;
        toy_bank_err_t e = toy_bank_verify_replay(bank_buf, len, scratch,
                                                  TOY_MAX_STATE_SIZE,
                                                  &verified, 0, 0, 0, 0);
        uint64_t hfinal = toy_hash_state(scratch, c->state_size);
        printf("    %s bank replay: %u keyframes verified, final hash %016llx\n",
               c->name, verified, (unsigned long long)hfinal);
        snprintf(what, sizeof what, "%s bank verifies by replay", names[t]);
        check(e == TOY_BANK_OK && verified == 5, what);
        /* Untampered bank must equal the uninterrupted fed run's
         * final hash - replay is re-derivation, not self-trust. */
        {
            void *ref = xcalloc(c->state_size);
            toy_sig_source_t src;
            uint32_t cur = 0;
            int ok = toy_sig_proof_source_init(c, &src) == 1;
            c->reset(ref);
            toy_apply_proof_at(c, ref, 0, &cur);
            for (uint64_t s = 1; ok && s <= 200; s++)
                ok = feed_boundary(c, ref, &src, &cur, s);
            snprintf(what, sizeof what,
                     "%s bank final == uninterrupted fed run", names[t]);
            check(ok && toy_hash_state(ref, c->state_size) == hfinal, what);
            free(ref);
        }
        free(scratch);
    }

    /* Tamper: flip a payload byte in keyframe 2 and repair
     * every CRC/hash wrapper, so the bank parses but re-stepped
     * states cannot match its (recomputed) keyframe hash. */
    {
        const toy_cart_t *gpend = toy_cart_by_name("gpend");
        size_t len = build_feed_bank(gpend, 200, 50);
        toy_bank_info_t info;
        toy_bank_keyframe_t kf;
        int shape = len > 0 &&
            toy_bank_parse(bank_buf, len, NULL, &info, NULL) == TOY_BANK_OK &&
            toy_bank_keyframe(bank_buf, len, &info, 2, &kf) == TOY_BANK_OK;
        check(shape, "gpend bank parses before tamper");
        unsigned char *img = bank_buf + kf.image_offset;
        unsigned char *payload = img + TOY_CKPT_HEADER_SIZE;
        payload[10] ^= 0x40u;
        uint64_t new_hash = toy_hash_state(payload, gpend->state_size);
        put_u64le(img + 48, new_hash);
        put_u32le(img + 56, toy_ckpt_crc32(payload, gpend->state_size));
        put_u32le(img + 60, toy_ckpt_crc32(img, 60));
        put_u64le(bank_buf + TOY_BANK_HEADER_SIZE + 2 * TOY_BANK_INDEX_ENTRY_SIZE + 8,
                  new_hash);
        put_u32le(bank_buf + 80,
                  toy_ckpt_crc32(bank_buf + TOY_BANK_HEADER_SIZE,
                                 (size_t)info.keyframe_count *
                                     TOY_BANK_INDEX_ENTRY_SIZE));
        put_u32le(bank_buf + 92, toy_ckpt_crc32(bank_buf, 92));
        void *scratch = xcalloc(TOY_MAX_STATE_SIZE);
        uint32_t verified = 0, fail_idx = 99;
        uint64_t fail_step = 0, want = 0, got = 0;
        toy_bank_err_t e = toy_bank_verify_replay(bank_buf, len, scratch,
                                                  TOY_MAX_STATE_SIZE,
                                                  &verified, &fail_idx,
                                                  &fail_step, &want, &got);
        printf("    tampered gpend bank: %s at keyframe %u step %llu "
               "want %016llx got %016llx (verified %u before)\n",
               toy_bank_strerror(e), fail_idx,
               (unsigned long long)fail_step, (unsigned long long)want,
               (unsigned long long)got, verified);
        check(toy_bank_parse(bank_buf, len, NULL, NULL, NULL) == TOY_BANK_OK &&
              e == TOY_BANK_ERR_REPLAY_HASH && fail_idx == 2 &&
              fail_step == 100 && want == new_hash && got != new_hash &&
              verified == 2,
              "CRC-repaired tamper still fails replay-hash at keyframe 2");
        free(scratch);
    }

    /* Wrong-kind descriptors stay refused at begin. */
    {
        const toy_cart_t *gpend = toy_cart_by_name("gpend");
        toy_cart_t live = *gpend;
        toy_bank_writer_t w;
        live.sig_spec.proof_kind = TOY_SIG_KIND_LIVE;
        check(toy_bank_begin(&w, &live, 10, 5, 0, 0, 0,
                             bank_buf, sizeof bank_buf) ==
                  TOY_BANK_ERR_FEED_CART,
              "LIVE-proof-kind feed cart refuses to bank");
        live.sig_spec.proof_kind = TOY_SIG_KIND_REPLAY;
        check(toy_bank_begin(&w, &live, 10, 5, 0, 0, 0,
                             bank_buf, sizeof bank_buf) ==
                  TOY_BANK_ERR_FEED_CART,
              "REPLAY-proof-kind feed cart refuses to bank");
    }
}

int main(void)
{
    test_crc();
    test_registry();
    test_golden();
    test_roundtrip();
    test_resume();
    test_rejection();
    test_feed_resume();
    test_feed_bank();
    printf("ckpt_test: %d checks, %d failures - %s\n",
           checks, failures, failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}



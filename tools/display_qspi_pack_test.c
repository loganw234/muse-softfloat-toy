/* display_qspi_pack_test.c - host test for the PIO TX word
 * packer (board/display_qspi_pio.h, toy5-flush).
 *
 * The packer is the only host-compilable part of the RP2350
 * transport: display_qspi_pio.c's sink feeds every cmd/pixel
 * byte through display_qspi_pack_feed with a put_word that
 * does pio_sm_put_blocking. Here a capturing put_word records
 * the emitted words so the EXACT word sequence can be pinned:
 * word values, cross-call partial carry, every 1..3-byte tail
 * shape, and - the bit-identity contract - an independent
 * OSR simulation (shift left, nibbles MSB-first, 32 bits per
 * full word, tail_bits for the final short word) that must
 * reproduce the input byte stream byte for byte. That stream
 * is the same one the old one-byte-per-word feed produced by
 * construction (word = byte << 24, 8 bits consumed), which is
 * also simulated and compared nibble for nibble.
 *
 * Build (from repo root):
 *   gcc -O2 -std=c11 -Wall -Wextra -Werror -Iboard \
 *       tools/display_qspi_pack_test.c \
 *       -o build/display_qspi_pack_test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "display_qspi_pio.h"

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
            failures++;                                                      \
        }                                                                    \
    } while (0)

#define CAP_WORDS 4096u

typedef struct {
    uint32_t words[CAP_WORDS];
    size_t nwords;
    int overflow;
} cap_t;

static cap_t cap;
static display_qspi_pack_t pack;

static void cap_put(void *ctx, uint32_t word)
{
    cap_t *c = (cap_t *)ctx;
    if (c->nwords < CAP_WORDS)
        c->words[c->nwords] = word;
    else
        c->overflow = 1;
    c->nwords++;
}

static void cap_reset(void)
{
    memset(&cap, 0, sizeof cap);
    display_qspi_pack_reset(&pack);
}

static void feed(const uint8_t *bytes, size_t len)
{
    display_qspi_pack_feed(&pack, bytes, len, cap_put, &cap);
}

/* Independent OSR model: shift `bits` of `word` out to the
 * left, one nibble at a time (bits 31..28 first), appending
 * each nibble to out[]. Returns nibbles written. */
static size_t osr_shift_out(uint32_t word, uint32_t bits, uint8_t *out)
{
    size_t n = 0;
    for (uint32_t b = 0; b < bits; b += 4)
        out[n++] = (uint8_t)((word << b) >> 28);
    return n;
}

/* Reconstruct the wire byte stream from the captured words
 * plus the pending tail exactly as flush_pack_tail emits it
 * (tail word consumed for tail_bits). Must equal the fed
 * bytes - this is the bit-identity pin. */
static void check_wire_identity(const uint8_t *want, size_t want_len)
{
    static uint8_t nib[2 * 8192];
    size_t nnib = 0;
    CHECK(!cap.overflow);
    for (size_t i = 0; i < cap.nwords; i++)
        nnib += osr_shift_out(cap.words[i], 32, nib + nnib);
    nnib += osr_shift_out(pack.word, display_qspi_pack_tail_bits(&pack),
                          nib + nnib);
    CHECK(nnib == 2 * want_len);
    if (nnib != 2 * want_len)
        return;
    int same = 1;
    for (size_t i = 0; i < want_len; i++) {
        uint8_t b = (uint8_t)((nib[2 * i] << 4) | nib[2 * i + 1]);
        if (b != want[i])
            same = 0;
    }
    CHECK(same);
    /* And the OLD feed (byte << 24, 8 bits per word) must
     * produce the identical nibble stream. */
    size_t on = 0;
    int osame = 1;
    for (size_t i = 0; i < want_len; i++) {
        uint8_t onib[2];
        osr_shift_out((uint32_t)want[i] << 24, 8, onib);
        if (onib[0] != nib[on] || onib[1] != nib[on + 1])
            osame = 0;
        on += 2;
    }
    CHECK(osame);
}

static void test_word4(void)
{
    static const uint8_t b[4] = { 0x12, 0x34, 0x56, 0x78 };
    CHECK(display_qspi_pack_word4(b) == 0x12345678u);
    static const uint8_t z[4] = { 0xff, 0x00, 0x80, 0x01 };
    CHECK(display_qspi_pack_word4(z) == 0xff008001u);
}

/* Exact expectations for single feeds of 0..5 bytes over the
 * sequence 11 22 33 44 55: emitted words, pending tail word
 * (real bytes on top, zeros below) and tail bit count. */
static void test_short_feeds_exact(void)
{
    static const uint8_t seq[5] = { 0x11, 0x22, 0x33, 0x44, 0x55 };
    struct {
        size_t len;
        size_t nwords;
        uint32_t word0;
        uint8_t tail_len;
        uint32_t tail_word;
        uint32_t tail_bits;
    } cases[] = {
        { 0, 0, 0u,          0, 0x00000000u, 0  },
        { 1, 0, 0u,          1, 0x11000000u, 8  },
        { 2, 0, 0u,          2, 0x11220000u, 16 },
        { 3, 0, 0u,          3, 0x11223300u, 24 },
        { 4, 1, 0x11223344u, 0, 0x00000000u, 0  },
        { 5, 1, 0x11223344u, 1, 0x55000000u, 8  },
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        cap_reset();
        feed(seq, cases[i].len);
        CHECK(cap.nwords == cases[i].nwords);
        if (cases[i].nwords && cap.nwords >= 1)
            CHECK(cap.words[0] == cases[i].word0);
        CHECK(pack.len == cases[i].tail_len);
        CHECK(pack.word == cases[i].tail_word);
        CHECK(display_qspi_pack_tail_bits(&pack) == cases[i].tail_bits);
        check_wire_identity(seq, cases[i].len);
    }
}

/* Push the same 13 bytes one at a time and in irregular
 * split calls: identical words and tail every time (chunk
 * boundaries invisible), plus wire identity. */
static void test_split_invariance(void)
{
    static const uint8_t seq[13] = { 0xde, 0xad, 0xbe, 0xef, 0x01,
                                     0x23, 0x45, 0x67, 0x89, 0xab,
                                     0xcd, 0xef, 0x42 };
    static const uint32_t want_words[3] = { 0xdeadbeefu, 0x01234567u,
                                            0x89abcdefu };

    cap_reset();
    for (size_t i = 0; i < sizeof seq; i++)
        feed(seq + i, 1);
    CHECK(cap.nwords == 3);
    if (cap.nwords == 3)
        CHECK(memcmp(cap.words, want_words, sizeof want_words) == 0);
    CHECK(pack.len == 1 && pack.word == 0x42000000u);
    check_wire_identity(seq, sizeof seq);

    cap_reset();
    static const size_t splits[] = { 3, 1, 4, 2, 3 };
    size_t off = 0;
    for (size_t i = 0; i < sizeof splits / sizeof splits[0]; i++) {
        feed(seq + off, splits[i]);
        off += splits[i];
    }
    CHECK(off == sizeof seq);
    CHECK(cap.nwords == 3);
    if (cap.nwords == 3)
        CHECK(memcmp(cap.words, want_words, sizeof want_words) == 0);
    CHECK(pack.len == 1 && pack.word == 0x42000000u);
    check_wire_identity(seq, sizeof seq);

    /* Whole-buffer feed: same again. */
    cap_reset();
    feed(seq, sizeof seq);
    CHECK(cap.nwords == 3);
    if (cap.nwords == 3)
        CHECK(memcmp(cap.words, want_words, sizeof want_words) == 0);
    CHECK(pack.len == 1 && pack.word == 0x42000000u);
    check_wire_identity(seq, sizeof seq);
}

/* Every length 0..17 of a counting sequence: word count is
 * len/4, tail is len%4, wire identity holds. Covers all
 * residues and the 4/8/12/16 exact multiples. */
static void test_all_lengths(void)
{
    uint8_t seq[17];
    for (size_t i = 0; i < sizeof seq; i++)
        seq[i] = (uint8_t)(0x10u + 0x11u * i);
    for (size_t len = 0; len <= sizeof seq; len++) {
        cap_reset();
        feed(seq, len);
        CHECK(cap.nwords == len / 4);
        CHECK(pack.len == len % 4);
        check_wire_identity(seq, len);
    }
}

/* Deterministic pseudo-random stream, pseudo-random call
 * splits (LCG, fixed seed): 1000 bytes = 250 whole words,
 * then 999 bytes leaves a 3-byte tail. Wire identity both. */
static uint32_t lcg_state;
static uint8_t lcg_byte(void)
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (uint8_t)(lcg_state >> 24);
}

static void test_lcg_stream(void)
{
    static uint8_t seq[1000];
    lcg_state = 0x5eed1234u;
    for (size_t i = 0; i < sizeof seq; i++)
        seq[i] = lcg_byte();

    cap_reset();
    size_t off = 0;
    while (off < sizeof seq) {
        size_t n = (size_t)(lcg_byte() % 9u); /* 0..8, incl. empty */
        if (n > sizeof seq - off)
            n = sizeof seq - off;
        feed(seq + off, n);
        off += n;
        if (n == 0) { /* guarantee progress past empty calls */
            feed(seq + off, 1);
            off += 1;
        }
    }
    CHECK(cap.nwords == 250);
    CHECK(pack.len == 0);
    check_wire_identity(seq, sizeof seq);

    cap_reset();
    feed(seq, sizeof seq - 1);
    CHECK(cap.nwords == 249);
    CHECK(pack.len == 3);
    check_wire_identity(seq, sizeof seq - 1);
}

/* Command path: each logical cmd byte expands to 4 wire bytes
 * and must pack into EXACTLY one word whose nibbles carry the
 * byte MSB-first on SI0 with SI1..SI3 low (the decode mirrors
 * tools/display_test.c's), and must never leave a tail -
 * the property flush_pack_tail's reachability note relies on. */
static void test_cmd_expansion_packs_exact(void)
{
    for (int b = 0; b < 256; b++) {
        uint8_t expanded[4];
        co5300_expand_cmd_byte((uint8_t)b, expanded);
        cap_reset();
        feed(expanded, sizeof expanded);
        CHECK(cap.nwords == 1);
        CHECK(pack.len == 0);
        if (cap.nwords != 1)
            continue;
        uint8_t nib[8];
        osr_shift_out(cap.words[0], 32, nib);
        uint8_t v = 0;
        for (int k = 0; k < 8; k++) {
            CHECK((nib[k] & 0xeu) == 0); /* SI1..SI3 stay low */
            v = (uint8_t)((v << 1) | (nib[k] & 1u));
        }
        CHECK(v == (uint8_t)b);
    }
}

/* A whole register-write header {02 00 2A 00} then its
 * parameter bytes, fed as the driver calls them (two sink
 * calls after per-byte expansion): 8 logical bytes = 8 words,
 * no tail, wire bytes after nibble decode spell the header in
 * 1-wire bit order. Spot-check the first word exactly:
 * 0x02 = 0000_0010b -> nibbles 0,0,0,0,0,0,1,0 ->
 * word 0x00000010. */
static void test_cmd_header_sequence(void)
{
    static const uint8_t hdr[4] = { 0x02, 0x00, 0x2a, 0x00 };
    static const uint8_t par[4] = { 0x00, 0x06, 0x01, 0xd7 };
    cap_reset();
    for (int i = 0; i < 4; i++) {
        uint8_t expanded[4];
        co5300_expand_cmd_byte(hdr[i], expanded);
        feed(expanded, sizeof expanded);
    }
    for (int i = 0; i < 4; i++) {
        uint8_t expanded[4];
        co5300_expand_cmd_byte(par[i], expanded);
        feed(expanded, sizeof expanded);
    }
    CHECK(cap.nwords == 8);
    CHECK(pack.len == 0);
    if (cap.nwords >= 1)
        CHECK(cap.words[0] == 0x00000010u);
}

int main(void)
{
    test_word4();
    test_short_feeds_exact();
    test_split_invariance();
    test_all_lengths();
    test_lcg_stream();
    test_cmd_expansion_packs_exact();
    test_cmd_header_sequence();
    if (failures) {
        printf("display_qspi_pack_test: FAIL (%d failures)\n", failures);
        return EXIT_FAILURE;
    }
    printf("display_qspi_pack_test: PASS\n");
    return EXIT_SUCCESS;
}

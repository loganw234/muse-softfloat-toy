/* clock_test.c - standalone host test for the time layer v1:
 * core/toy_clock.h (runner-owned clock API) + core/toy_bank.c
 * (TOYBANK1 sparse-keyframe banking / verified replay).
 *
 * Deliberately NOT part of tools/run.sh (mirroring ckpt_test.c):
 * build and run it separately with the run.sh host flags -
 * gcc -O2 -std=c11 -Wall -Wextra, include dirs
 * libs/muse-softfloat/include and core, sources: every C file
 * under core, this file, and the muse-softfloat sources (NOT
 * emu sources: this file provides its own controllable clock
 * implementation, the fake runner clock the fake-clock test
 * posture of study-timebank section 2.4 calls for), output
 * build/clock_test. Exit status 0 iff every check passes.
 *
 * Coverage:
 *  (a) Clock set/unset behaviour: unset is first-class (invalid
 *      snapshot, schedule always-active), set/clear round-trip,
 *      zone-offset minute-of-day arithmetic, civil time
 *      advancing off the monotonic anchor, schedule windows
 *      incl. midnight wrap, warp/pacing arithmetic.
 *  (b) Banking round-trip on fig8 and on the largest-state
 *      registered cart: bank -> parse -> verified replay (hash
 *      equality at EVERY keyframe by re-stepping), plus an
 *      independent uninterrupted run agreeing on the end hash.
 *  (c) Tampered keyframes rejected: payload bit flip (embedded
 *      checkpoint CRC), index flip (index CRC), and the strong
 *      case - a fully CRC-repaired wrong keyframe state that
 *      parses clean and is caught only by replay re-stepping.
 *  (d) Budget: keyframe counts past the fixed 8 MiB budget
 *      refuse at estimate/begin; schedule arithmetic pinned.
 *  (e) Morning report v1 fields with the clock set (wall span,
 *      expected 1x steps) and unset (no invented times).
 *  (f) THE DETERMINISM FIREWALL (study section 7.1 acceptance
 *      test): identical step streams under unset / set /
 *      mid-run-jumping-backwards / cleared clocks produce
 *      identical cartridge hashes. The clock is polled between
 *      steps exactly as a runner would; state never sees it.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "toy_cart.h"
#include "toy_ckpt.h"
#include "toy_bank.h"
#define TOY_CLOCK_ALLOW 1
#include "toy_clock.h"

/* ---- the fake runner clock under test control --------------- */

static int      fake_valid;
static int64_t  fake_base_utc_s;
static uint64_t fake_base_mono_ms;
static uint64_t fake_mono_ms;
static int32_t  fake_tz_min;

toy_time_t toy_clock_now(void)
{
    toy_time_t t;
    t.utc_seconds = 0;
    t.tz_offset_min = fake_tz_min;
    t.valid = fake_valid != 0;
    if (fake_valid)
        t.utc_seconds = fake_base_utc_s +
                        (int64_t)((fake_mono_ms - fake_base_mono_ms) / 1000ull);
    return t;
}
uint64_t toy_clock_mono_ms(void) { return fake_mono_ms; }
bool toy_clock_set(int64_t utc_seconds, int32_t tz_offset_min)
{
    fake_base_utc_s = utc_seconds;
    fake_base_mono_ms = fake_mono_ms;
    fake_tz_min = tz_offset_min;
    fake_valid = 1;
    return true;
}
void toy_clock_clear(void) { fake_valid = 0; }

/* ---- tiny test harness (ckpt_test.c idiom) ------------------ */

static int failures = 0;
static int checks = 0;

static void check(int ok, const char *what)
{
    checks++;
    printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
    if (!ok) failures++;
}

static void expect_bank_err(const char *what, toy_bank_err_t got,
                            toy_bank_err_t want)
{
    checks++;
    int ok = got == want;
    printf("  [%s] %s (got %s, want %s)\n", ok ? "ok" : "FAIL", what,
           toy_bank_strerror(got), toy_bank_strerror(want));
    if (!ok) failures++;
}

static void *xcalloc(size_t n)
{
    void *p = calloc(1, n ? n : 1);
    if (!p) { fprintf(stderr, "oom\n"); exit(2); }
    return p;
}

static unsigned char bank_buf[TOY_BANK_BUDGET_BYTES];
static unsigned char bank_copy[TOY_BANK_BUDGET_BYTES];

static void put_u32le(unsigned char *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) p[i] = (unsigned char)(v >> (8 * i));
}
static void put_u64le(unsigned char *p, uint64_t v)
{
    for (int i = 0; i < 8; i++) p[i] = (unsigned char)(v >> (8 * i));
}

/* Step like every runner: proof entries at absolute steps via
 * the shared helpers, step 0 after reset. */
static uint64_t run_proof_style(const toy_cart_t *cart, void *st, uint64_t total)
{
    uint32_t cur = 0;
    toy_apply_proof_at(cart, st, 0, &cur);
    for (uint64_t s = 1; s <= total; s++) {
        toy_apply_proof_at(cart, st, (uint32_t)s, &cur);
        cart->step(st);
    }
    return toy_hash_state(st, cart->state_size);
}

/* Build a bank exactly as emu `bank` does. Returns length or 0. */
static size_t build_bank(const toy_cart_t *cart, uint64_t total, uint32_t every,
                         int64_t wall_start, int wall_valid, int32_t tz,
                         int64_t wall_end)
{
    uint32_t count = toy_bank_keyframe_count(total, every);
    size_t est = toy_bank_estimate_size(cart, count);
    if (est == 0) return 0;
    toy_bank_writer_t w;
    if (toy_bank_begin(&w, cart, total, every, wall_start,
                       wall_valid ? 1u : 0u, tz, bank_buf, sizeof bank_buf)
            != TOY_BANK_OK)
        return 0;
    void *st = xcalloc(cart->state_size);
    cart->reset(st);
    uint32_t cur = 0;
    toy_apply_proof_at(cart, st, 0, &cur);
    uint32_t next_kf = 0;
    size_t out_len = 0;
    int ok = 1;
    for (uint64_t s = 0;; s++) {
        if (next_kf < count &&
            s == toy_bank_keyframe_step(total, every, next_kf)) {
            if (toy_bank_add_keyframe(&w, st, s, wall_start) != TOY_BANK_OK)
                ok = 0;
            next_kf++;
        }
        if (s >= total) break;
        toy_apply_proof_at(cart, st, (uint32_t)(s + 1), &cur);
        cart->step(st);
    }
    if (toy_bank_finish(&w, wall_end, TOY_BANK_END_COMPLETE, &out_len)
            != TOY_BANK_OK)
        ok = 0;
    free(st);
    return ok ? out_len : 0;
}

/* ---- (a) clock behaviour ------------------------------------ */

static void test_clock(void)
{
    printf("clock set/unset, schedule, warp arithmetic\n");
    fake_mono_ms = 5000;
    toy_clock_clear();
    toy_time_t t = toy_clock_now();
    check(!t.valid, "starts unset: valid == false");

    toy_schedule_t sched = toy_schedule_default();
    check(sched.live_warp_x100 == 100 && sched.playback_warp_x100 == 1600 &&
          sched.keyframe_interval_s == 60 && sched.banking_enabled == 1,
          "defaults: live 1.00x, playback 16x, keyframes 60 s, banking on");
    check(toy_clock_is_active(t, &sched),
          "unset clock: schedule degrades to always-active");

    /* Pick a round local noon without relying on a calendar:
     * with tz +330, local = utc + 19800 s. Choose utc so local
     * is exactly 12:00 on some day: local_seconds % 86400 =
     * 43200. Minute-of-day checks below verify the arithmetic. */
    int64_t utc = 1760000000LL; /* arbitrary anchor */
    int64_t rem = ((utc + 330 * 60) % 86400 + 86400) % 86400;
    utc += (43200 - rem + 86400) % 86400;
    check(toy_clock_set(utc, 330), "toy_clock_set accepted");
    t = toy_clock_now();
    check(t.valid && t.utc_seconds == utc && t.tz_offset_min == 330,
          "set snapshot round-trips (utc + tz)");
    check(toy_clock_minutes_of_day(t) == 720,
          "local minute-of-day = 720 (12:00 at UTC+5:30)");
    check(toy_clock_is_active(t, &sched), "12:00 inside 07:00-23:00 window");

    fake_mono_ms += 65u * 1000u; /* 65 monotonic seconds pass */
    t = toy_clock_now();
    check(t.utc_seconds == utc + 65,
          "civil time advances off the monotonic anchor (+65 s)");

    check(toy_clock_set(utc, 0), "re-set at tz 0");
    t = toy_clock_now();
    check(toy_clock_minutes_of_day(t) == (720 - 330 + 1440) % 1440,
          "same instant at UTC+0 is 06:30 local");
    check(!toy_clock_is_active(t, &sched), "06:30 outside 07:00-23:00 window");

    /* Midnight-crossing window 22:00 -> 06:00 (active = night). */
    toy_schedule_t night = sched;
    night.active_start_min = 22u * 60u;
    night.active_end_min = 6u * 60u;
    check(!toy_clock_is_active(t, &night),
          "06:30 outside midnight-crossing 22:00-06:00 window");
    /* 23:30 local at tz 0: shift the anchor by +17 h. */
    check(toy_clock_set(utc + 17 * 3600, 0), "re-set to 23:30 local");
    t = toy_clock_now();
    check(toy_clock_minutes_of_day(t) == 1410 &&
          toy_clock_is_active(t, &night) && !toy_clock_is_active(t, &sched),
          "23:30 inside night window, outside day window");

    toy_clock_clear();
    t = toy_clock_now();
    check(!t.valid, "clear returns to unset");
    check(toy_clock_is_active(t, &night),
          "unset is always-active even for a night window");

    /* Warp/pacing arithmetic (runner-side counts only). */
    check(toy_clock_steps_due(1000, 4, 100) == 120,
          "fig8 demo 4spf: 1000 ms at 1.00x = 120 steps");
    check(toy_clock_steps_due(1000, 4, 1600) == 1920,
          "fig8 at playback 16x: 1000 ms = 1920 steps");
    check(toy_clock_steps_due(1000, 30, 100) == 900,
          "pendulum demo 30spf: 1000 ms at 1.00x = 900 steps");
    check(toy_keyframe_every_steps(4, 60) == 7200,
          "fig8 60 s keyframe interval = 7200 steps (study 3.3)");
    check(toy_keyframe_every_steps(30, 60) == 54000,
          "pendulum 60 s keyframe interval = 54000 steps");
    check(toy_keyframe_every_steps(4, 0) == 0,
          "interval 0 = start+final only");

    uint64_t m0 = toy_clock_mono_ms();
    fake_mono_ms += 7;
    check(toy_clock_mono_ms() == m0 + 7, "mono advances independently of set");
}

/* ---- (b)+(d) banking round-trip & budget -------------------- */

static void roundtrip_one(const toy_cart_t *cart, uint64_t total, uint32_t every)
{
    char what[192];
    size_t len = build_bank(cart, total, every, 0, 0, 0, 0);
    snprintf(what, sizeof what, "%s: bank %llu steps every %u built (%zu B)",
             cart->name, (unsigned long long)total, every, len);
    check(len > 0, what);
    if (!len) return;

    toy_bank_info_t info;
    const toy_cart_t *parsed = NULL;
    expect_bank_err("bank parses",
                    toy_bank_parse(bank_buf, len, cart, &info, &parsed),
                    TOY_BANK_OK);
    check(parsed == cart && info.total_steps == total &&
          info.keyframe_count == toy_bank_keyframe_count(total, every) &&
          info.keyframe_every == every && info.wall_valid == 0,
          "header fields: cart/steps/count/every, wall unset");
    printf("    %s bank: %zu bytes, %u keyframes, state %zu B\n",
           cart->name, len, info.keyframe_count, cart->state_size);
    for (uint32_t i = 0; i < info.keyframe_count; i++) {
        toy_bank_keyframe_t kf;
        toy_bank_keyframe(bank_buf, len, &info, i, &kf);
        printf("    keyframe %u step %llu hash %016llx\n", i,
               (unsigned long long)kf.step_index,
               (unsigned long long)kf.state_hash);
    }

    void *scratch = xcalloc(TOY_MAX_STATE_SIZE);
    uint32_t verified = 0;
    expect_bank_err("verified replay (restep hash at every keyframe)",
                    toy_bank_verify_replay(bank_buf, len, scratch,
                                           TOY_MAX_STATE_SIZE, &verified,
                                           NULL, NULL, NULL, NULL),
                    TOY_BANK_OK);
    snprintf(what, sizeof what, "%s: all %u keyframes verified by re-stepping",
             cart->name, info.keyframe_count);
    check(verified == info.keyframe_count, what);

    /* Independent oracle: an uninterrupted run must land on the
     * same final hash the bank claims. */
    void *ref = xcalloc(cart->state_size);
    cart->reset(ref);
    uint64_t href = run_proof_style(cart, ref, total);
    toy_morning_report_t rep;
    toy_bank_morning_report(bank_buf, len, 1u, &rep, NULL);
    snprintf(what, sizeof what,
             "%s: uninterrupted hash %016llx == bank hash_end %016llx",
             cart->name, (unsigned long long)href,
             (unsigned long long)rep.hash_end);
    check(href == rep.hash_end, what);
    check(toy_hash_state(scratch, cart->state_size) == rep.hash_end,
          "replay scratch ends on hash_end byte-stream");
    free(scratch);
    free(ref);
}

static void test_banking(void)
{
    printf("banking round-trip: fig8 + largest-state cart\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    check(fig8 != NULL, "fig8 registered");
    roundtrip_one(fig8, 5000, 700);   /* steps 0,700,...,4900,5000 */
    roundtrip_one(fig8, 1234, 0);     /* start+final density */
    roundtrip_one(fig8, 0, 100);      /* degenerate: step 0 only */

    const toy_cart_t *big = toy_carts[0];
    for (int i = 1; i < toy_cart_count; i++)
        if (toy_carts[i]->state_size > big->state_size) big = toy_carts[i];
    printf("    largest-state cart: %s (%zu B, budget allows %u keyframes)\n",
           big->name, big->state_size, toy_bank_max_keyframes(big));
    roundtrip_one(big, 300, 100);
}

static void test_budget(void)
{
    printf("keyframe schedule + fixed budget arithmetic\n");
    check(toy_bank_keyframe_count(0, 100) == 1, "count(0, E) = 1");
    check(toy_bank_keyframe_count(100, 0) == 2, "count(T, 0) = 2");
    check(toy_bank_keyframe_count(100, 30) == 5, "count(100,30) = 5 {0,30,60,90,100}");
    check(toy_bank_keyframe_count(90, 30) == 4, "count(90,30) = 4 {0,30,60,90}");
    check(toy_bank_keyframe_step(100, 30, 0) == 0 &&
          toy_bank_keyframe_step(100, 30, 3) == 90 &&
          toy_bank_keyframe_step(100, 30, 4) == 100,
          "schedule steps {0,...,90,100}");
    check(toy_bank_keyframe_step(100, 0, 0) == 0 &&
          toy_bank_keyframe_step(100, 0, 1) == 100,
          "start+final schedule {0,100}");

    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    uint32_t maxk = toy_bank_max_keyframes(fig8);
    size_t at_max = toy_bank_estimate_size(fig8, maxk);
    printf("    fig8: max %u keyframes in %u B budget (bank %zu B)\n",
           maxk, (unsigned)TOY_BANK_BUDGET_BYTES, at_max);
    check(maxk >= 480 && at_max > 0 && at_max <= TOY_BANK_BUDGET_BYTES,
          "fig8 480-keyframe night fits the fixed budget");
    check(toy_bank_estimate_size(fig8, maxk + 1) == 0,
          "one keyframe past budget: estimate refuses (0)");
    toy_bank_writer_t w;
    uint64_t huge_total = (uint64_t)(maxk + 5) * 10u;
    expect_bank_err("writer begin past budget refuses",
                    toy_bank_begin(&w, fig8, huge_total, 10, 0, 0, 0,
                                   bank_buf, sizeof bank_buf),
                    TOY_BANK_ERR_COUNT);
    expect_bank_err("writer begin into short buffer refuses",
                    toy_bank_begin(&w, fig8, 100, 10, 0, 0, 0, bank_buf, 64),
                    TOY_BANK_ERR_OUT_TOO_SMALL);
}

/* ---- (c) tamper rejection ----------------------------------- */

static void test_tamper(void)
{
    printf("tampered keyframe rejection\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    size_t len = build_bank(fig8, 900, 300, 0, 0, 0, 0);
    check(len > 0, "base bank built (900 steps, every 300)");
    if (!len) return;
    toy_bank_info_t info;
    toy_bank_parse(bank_buf, len, NULL, &info, NULL);
    toy_bank_keyframe_t kf1;
    toy_bank_keyframe(bank_buf, len, &info, 1, &kf1);
    void *scratch = xcalloc(TOY_MAX_STATE_SIZE);

    /* 1: payload bit flip inside keyframe 1's checkpoint. */
    memcpy(bank_copy, bank_buf, len);
    bank_copy[kf1.image_offset + TOY_CKPT_HEADER_SIZE + 10] ^= 0x40u;
    expect_bank_err("payload bit flip: embedded ckpt rejects",
                    toy_bank_parse(bank_copy, len, NULL, NULL, NULL),
                    TOY_BANK_ERR_KEYFRAME_CKPT);

    /* 2: index hash flip. */
    memcpy(bank_copy, bank_buf, len);
    bank_copy[TOY_BANK_HEADER_SIZE + TOY_BANK_INDEX_ENTRY_SIZE + 8] ^= 0x01u;
    expect_bank_err("index byte flip: index CRC rejects",
                    toy_bank_parse(bank_copy, len, NULL, NULL, NULL),
                    TOY_BANK_ERR_INDEX_CRC);

    /* 3: truncated bank (torn write). */
    expect_bank_err("truncated by one byte rejects",
                    toy_bank_parse(bank_buf, len - 1, NULL, NULL, NULL),
                    TOY_BANK_ERR_LENGTH);
    expect_bank_err("header-only prefix rejects",
                    toy_bank_parse(bank_buf, TOY_BANK_HEADER_SIZE, NULL,
                                   NULL, NULL),
                    TOY_BANK_ERR_LENGTH);

    /* 4: the strong case. Replace keyframe 1's state with wrong
     * bytes and repair EVERY checksum (ckpt payload CRC + FNV +
     * ckpt header CRC, bank index hash + CRC + bank header
     * CRC): intact transport, wrong meaning. Parse must accept;
     * only re-stepping from keyframe 0 can catch it, at
     * keyframe 1. */
    memcpy(bank_copy, bank_buf, len);
    unsigned char *img = bank_copy + kf1.image_offset;
    img[TOY_CKPT_HEADER_SIZE + 10] ^= 0x40u;
    uint64_t wrong_hash = toy_hash_state(img + TOY_CKPT_HEADER_SIZE,
                                         fig8->state_size);
    put_u32le(img + 56, toy_ckpt_crc32(img + TOY_CKPT_HEADER_SIZE,
                                       fig8->state_size));
    put_u64le(img + 48, wrong_hash);
    put_u32le(img + 60, toy_ckpt_crc32(img, 60));
    unsigned char *en1 = bank_copy + TOY_BANK_HEADER_SIZE +
                         TOY_BANK_INDEX_ENTRY_SIZE;
    put_u64le(en1 + 8, wrong_hash);
    put_u32le(bank_copy + 80, toy_ckpt_crc32(bank_copy + TOY_BANK_HEADER_SIZE,
                                             (size_t)info.keyframe_count *
                                                 TOY_BANK_INDEX_ENTRY_SIZE));
    put_u32le(bank_copy + 92, toy_ckpt_crc32(bank_copy, 92));
    expect_bank_err("fully-repaired wrong keyframe still parses",
                    toy_bank_parse(bank_copy, len, NULL, NULL, NULL),
                    TOY_BANK_OK);
    uint32_t fail_idx = 999;
    uint64_t fail_step = 0, want = 0, got = 0;
    expect_bank_err("replay re-stepping catches the wrong keyframe",
                    toy_bank_verify_replay(bank_copy, len, scratch,
                                           TOY_MAX_STATE_SIZE, NULL,
                                           &fail_idx, &fail_step, &want, &got),
                    TOY_BANK_ERR_REPLAY_HASH);
    printf("    replay failure: keyframe %u step %llu want %016llx got %016llx\n",
           fail_idx, (unsigned long long)fail_step,
           (unsigned long long)want, (unsigned long long)got);
    check(fail_idx == 1 && fail_step == 300 && want == wrong_hash && got != want,
          "failure names keyframe 1 at step 300 with both hashes");

    /* 5: untouched bank still verifies (copy games left it be). */
    expect_bank_err("original bank verifies after tamper copies",
                    toy_bank_verify_replay(bank_buf, len, scratch,
                                           TOY_MAX_STATE_SIZE, NULL, NULL,
                                           NULL, NULL, NULL),
                    TOY_BANK_OK);
    free(scratch);
}

/* ---- (e) morning report ------------------------------------- */

static void test_report(void)
{
    printf("morning report v1 (clock set vs unset)\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    /* A simulated 2 h wall span at fig8's 120 steps/s demo rate
     * expects 864000 steps at 1x; we bank 5000 - the report must
     * expose the shortfall honestly, not hide it. */
    int64_t w0 = 1760000000LL, w1 = w0 + 7200;
    size_t len = build_bank(fig8, 5000, 1000, w0, 1, 330, w1);
    check(len > 0, "labelled bank built");
    toy_morning_report_t r;
    expect_bank_err("report from labelled bank",
                    toy_bank_morning_report(bank_buf, len, 1u, &r, NULL),
                    TOY_BANK_OK);
    check(strcmp(r.cart_name, "fig8") == 0 && r.steps_at_start == 0 &&
          r.steps_at_end == 5000 && r.steps_banked == 5000,
          "report step range 0 -> 5000");
    check(r.keyframe_count == 6 && r.keyframe_every == 1000 &&
          r.hash_start != r.hash_end,
          "report keyframes + distinct start/end hashes");
    printf("    report hashes: start %016llx end %016llx\n",
           (unsigned long long)r.hash_start, (unsigned long long)r.hash_end);
    check(r.wall_valid == 1 && r.wall_start_s == w0 && r.wall_end_s == w1 &&
          r.tz_offset_min == 330,
          "report wall span + zone carried as labels");
    check(r.steps_expected_1x == 7200ull * 4ull * 30ull,
          "expected 1x steps = 7200 s x 120/s = 864000");
    check(r.playback_verified == 1 && r.end_reason == TOY_BANK_END_COMPLETE &&
          r.bank_bytes == len,
          "report verified flag, end reason, byte count");

    len = build_bank(fig8, 500, 100, 0, 0, 0, 0);
    expect_bank_err("report from unlabelled bank",
                    toy_bank_morning_report(bank_buf, len, 0u, &r, NULL),
                    TOY_BANK_OK);
    check(r.wall_valid == 0 && r.steps_expected_1x == 0 &&
          r.playback_verified == 0,
          "unset clock: no wall span, no invented expectation");
}

/* ---- (f) determinism firewall ------------------------------- */

static uint64_t firewall_run(const toy_cart_t *cart, int mode)
{
    void *st = xcalloc(cart->state_size);
    cart->reset(st);
    uint32_t cur = 0;
    toy_apply_proof_at(cart, st, 0, &cur);
    toy_schedule_t sched = toy_schedule_default();
    volatile int sink = 0; /* runner-side clock polls land here */
    const uint64_t total = 2000;
    if (mode == 1) toy_clock_set(1760000000LL, 330);
    if (mode == 2) toy_clock_set(1899999999LL, -480);
    if (mode == 0) toy_clock_clear();
    for (uint64_t s = 1; s <= total; s++) {
        if (mode != 0 && (s % 100) == 0) {
            toy_time_t now = toy_clock_now();
            sink += toy_clock_is_active(now, &sched) ? 1 : 0;
            sink += (int)(toy_clock_mono_ms() & 1u);
            fake_mono_ms += 12345; /* wall time races ahead */
        }
        if (mode == 2 && s == 1000)
            toy_clock_set(1000000000LL, 840); /* jump backwards + rezone */
        if (mode == 2 && s == 1500)
            toy_clock_clear();                /* unset mid-run */
        toy_apply_proof_at(cart, st, (uint32_t)s, &cur);
        cart->step(st);
    }
    uint64_t h = toy_hash_state(st, cart->state_size);
    free(st);
    (void)sink;
    return h;
}

static void test_firewall(void)
{
    printf("determinism firewall: identical steps, hostile clocks\n");
    const toy_cart_t *fig8 = toy_cart_by_name("fig8");
    fake_mono_ms = 100000;
    uint64_t h_unset = firewall_run(fig8, 0);
    uint64_t h_set = firewall_run(fig8, 1);
    uint64_t h_chaos = firewall_run(fig8, 2);

    /* Clock-free reference: never touches the clock API at all. */
    void *st = xcalloc(fig8->state_size);
    fig8->reset(st);
    uint64_t h_ref = run_proof_style(fig8, st, 2000);
    free(st);

    printf("    fig8 @2000  clock-free %016llx\n", (unsigned long long)h_ref);
    printf("    fig8 @2000  unset      %016llx\n", (unsigned long long)h_unset);
    printf("    fig8 @2000  set+330    %016llx\n", (unsigned long long)h_set);
    printf("    fig8 @2000  jump/clear %016llx\n", (unsigned long long)h_chaos);
    check(h_unset == h_ref, "unset-clock run == clock-free reference");
    check(h_set == h_ref, "set-clock run == clock-free reference");
    check(h_chaos == h_ref,
          "backwards-jump/rezone/mid-run-unset == clock-free reference");
    /* Sanity that the fuzz actually changed clock readings: the
     * chaos run ended unset, the set run ended set. */
    toy_clock_set(1760000000LL, 330);
    check(toy_clock_now().valid, "clock still settable after fuzz");
    toy_clock_clear();
    check(!toy_clock_now().valid, "and clearable after fuzz");
}

int main(void)
{
    test_clock();
    test_budget();
    test_banking();
    test_tamper();
    test_report();
    test_firewall();
    printf("clock_test: %d checks, %d failures - %s\n",
           checks, failures, failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}

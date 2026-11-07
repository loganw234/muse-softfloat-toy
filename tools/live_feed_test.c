/* live_feed_test.c - standalone host test for the
 * toy5-livefeed pure seam: board/live_feed.h (axis
 * transform, latest-sample hold accounting, source
 * selection, LIVE info) plus the core labelling path a
 * live runner stamps through (toy_sig_label_feed).
 *
 * Deliberately NOT part of tools/run.sh (mirroring
 * product_input_test.c / sensor_drivers_test.c): build and
 * run with
 *   gcc -O2 -std=c11 -Wall -Wextra -Werror -Iboard -Icore \
 *       tools/live_feed_test.c core/toy_signal.c \
 *       -o build/live_feed_test
 * Exit status 0 iff every check passes.
 *
 * Oracles are stated from the pinned definitions, not from
 * the code under test: the synthetic rest pose (az
 * +15864 desk / +16384 chest counts flat screen-up,
 * core/toy_signal.c) against the hardware rest pose (az
 * ~ -1000 mg = ~-16384 counts flat screen-up,
 * board/SENSORS.md); hold counts by hand from the
 * fresh/repeated definition in live_feed.h; the synthetic
 * comparison stream from core's own generator.
 */
#include <stdio.h>
#include <string.h>

#include "live_feed.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, what)                                              \
    do {                                                               \
        checks++;                                                      \
        if (!(cond)) {                                                 \
            failures++;                                                \
            printf("  FAIL %s (line %d): %s\n", __func__, __LINE__,    \
                   what);                                              \
        }                                                              \
    } while (0)

static void test_select(void)
{
    CHECK(toy_live_feed_select(0, TOY_SIG_DEVICE_IMU6, 1) ==
              TOY_LIVE_FEED_NONE,
          "signal-deaf cart needs no source");
    CHECK(toy_live_feed_select(0, 0, 0) == TOY_LIVE_FEED_NONE,
          "no feed, no device: none");
    CHECK(toy_live_feed_select(1, TOY_SIG_DEVICE_IMU6, 1) ==
              TOY_LIVE_FEED_LIVE,
          "IMU6 feed + IMU up: live");
    CHECK(toy_live_feed_select(1, TOY_SIG_DEVICE_IMU6, 0) ==
              TOY_LIVE_FEED_SYNTHETIC,
          "IMU6 feed + IMU down: synthetic fallback");
    CHECK(toy_live_feed_select(1, TOY_SIG_DEVICE_MIC1, 1) ==
              TOY_LIVE_FEED_SYNTHETIC,
          "MIC feed has no live device: synthetic");
    CHECK(toy_live_feed_select(1, TOY_SIG_DEVICE_MIC1, 0) ==
              TOY_LIVE_FEED_SYNTHETIC,
          "MIC feed + IMU down: synthetic");
}

static void test_transform(void)
{
    int16_t out[6];
    /* Flat screen-up rest, hardware pose: az -16384 counts
     * (-1000 mg). Must land on the synthetic rest pose:
     * az +16384 (chest) / +15864-class (desk). */
    const int16_t flat[6] = { 0, 0, -16384, 0, 0, 0 };
    toy_live_feed_transform(flat, out);
    CHECK(out[2] == 16384, "flat screen-up az -16384 -> +16384");
    CHECK(out[0] == 0 && out[1] == 0, "flat in-plane stays zero");

    const int16_t v[6] = { 100, -200, -300, 10, -20, 30 };
    toy_live_feed_transform(v, out);
    CHECK(out[0] == 100 && out[1] == -200 && out[2] == 300,
          "accel: ax/ay pass, az flips");
    CHECK(out[3] == 10 && out[4] == -20 && out[5] == -30,
          "gyro takes per-axis accel signs");

    /* Saturation corner: negating int16 min must clamp,
     * never wrap back negative. */
    const int16_t sat[6] = { 0, 0, (int16_t)-32768, 0, 0,
                             (int16_t)-32768 };
    toy_live_feed_transform(sat, out);
    CHECK(out[2] == 32767 && out[5] == 32767,
          "-(-32768) saturates to 32767");

    /* In-place aliasing (note_sample transforms into its
     * own buffer via a separate array, but the transform
     * itself promises alias safety). */
    int16_t inplace[6] = { 1, 2, -3, 4, 5, -6 };
    toy_live_feed_transform(inplace, inplace);
    CHECK(inplace[0] == 1 && inplace[2] == 3 && inplace[5] == 6,
          "in-place transform is alias-safe");
}

static void test_hold_accounting(void)
{
    toy_live_feed_t f;
    toy_live_feed_init(&f);
    int16_t frame[5 * 6];
    uint32_t flags = 0xdead;

    CHECK(toy_live_feed_fill(&f, frame, 5, 1000, &flags) == 0,
          "no good sample yet: fill refuses (nothing fabricated)");
    CHECK(flags == 0, "refused fill sets no flags");

    const int16_t a[6] = { 11, 22, -16384, 1, 2, 3 };
    toy_live_feed_note_sample(&f, a, 1000);
    CHECK(toy_live_feed_fill(&f, frame, 5, 1000, &flags) == 5,
          "first fill serves the step");
    CHECK(f.fresh_w == 1 && f.rep_w == 4,
          "n=5 on one new reading: 1 fresh + 4 repeated");
    CHECK(frame[0] == 11 && frame[2] == 16384 &&
              frame[4 * 6 + 0] == 11 && frame[4 * 6 + 2] == 16384,
          "frame repeats the transformed held sample");

    CHECK(toy_live_feed_fill(&f, frame, 5, 1016, &flags) == 5,
          "second fill still serves (hold)");
    CHECK(f.fresh_w == 1 && f.rep_w == 9,
          "no new reading: 5 more repeated, none fresh");
    CHECK(f.max_age_w == 16, "held age tracked: 1016-1000=16 ms");

    const int16_t b[6] = { -7, 8, -16000, 0, 0, 0 };
    toy_live_feed_note_sample(&f, b, 1020);
    CHECK(toy_live_feed_fill(&f, frame, 1, 1020, &flags) == 1,
          "new reading fills");
    CHECK(f.fresh_w == 2 && f.rep_w == 9,
          "new reading counts fresh once");
    CHECK(frame[0] == -7 && frame[2] == 16000,
          "frame carries the new reading, transformed");

    /* Error posture: hold last good, count, flag once. */
    toy_live_feed_note_error(&f);
    CHECK(toy_live_feed_fill(&f, frame, 1, 1030, &flags) == 1,
          "after error the held sample still feeds");
    CHECK((flags & TOY_SIG_FLAG_UNDERRUN) != 0,
          "error since last fill flags UNDERRUN (gap counted)");
    CHECK(f.err_w == 1 && f.err_tot == 1, "error counted w+tot");
    CHECK(frame[0] == -7, "held values unchanged by the error");
    CHECK(toy_live_feed_fill(&f, frame, 1, 1031, &flags) == 1 &&
              flags == 0,
          "flag clears after one flagged fill");

    /* Window reset keeps totals + hold; run reset marks the
     * held sample already-fed. */
    uint64_t ft = f.fresh_tot, rt = f.rep_tot;
    toy_live_feed_reset_window(&f);
    CHECK(f.fresh_w == 0 && f.rep_w == 0 && f.err_w == 0 &&
              f.max_age_w == 0,
          "window reset zeroes window counters");
    CHECK(f.fresh_tot == ft && f.rep_tot == rt && f.have,
          "window reset keeps totals and the held sample");
    toy_live_feed_reset_run(&f);
    CHECK(toy_live_feed_fill(&f, frame, 3, 1040, &flags) == 3 &&
              f.fresh_w == 0 && f.rep_w == 3,
          "after run reset the pre-start sample is repeated, not fresh");

    /* Backwards clock never reports a huge age. */
    toy_live_feed_note_sample(&f, a, 2000);
    CHECK(toy_live_feed_fill(&f, frame, 1, 1500, &flags) == 1 &&
              f.max_age_w == 20,
          "backwards clock: age ignored (still 1040-1020 window max 20)");
}

static void test_info_and_label(void)
{
    /* gpend's pinned spec (core/cart_gpend.c). */
    const toy_sig_spec_t spec = { TOY_SIG_DEVICE_IMU6,
                                  TOY_SIG_KIND_SYNTHETIC, 1u, 100u, 1u,
                                  UINT64_C(0x5eed) };
    toy_sig_info_t live, live2;
    toy_live_feed_info(&spec, &live);
    toy_live_feed_info(&spec, &live2);
    CHECK(live.kind == TOY_SIG_KIND_LIVE &&
              live.orig_kind == TOY_SIG_KIND_LIVE,
          "live info kind/orig are LIVE");
    CHECK(live.device == TOY_SIG_DEVICE_IMU6 && live.channels == 6,
          "live info device/channels from spec");
    CHECK(live.sample_rate_hz == 100, "info rate is the cart's spec rate");
    CHECK(live.scale_num == 16384 && live.scale2_num == 128,
          "info scales are the pinned driver/synthetic scales");
    CHECK(live.seed == 0 && live.scenario_id == 0 && live.n_samples == 0,
          "live carries no seed/scenario, unbounded");
    CHECK(live.stream_id != 0 && live.stream_id == live2.stream_id,
          "live stream id set and stable");

    /* A live run can never compare equal to the synthetic
     * proof stream for the same spec. */
    toy_sig_source_t synth;
    memset(&synth, 0, sizeof synth);
    CHECK(toy_sig_source_synth_imu_scenario(&synth, spec.seed,
                                            spec.sample_rate_hz,
                                            spec.scenario_id) == 0,
          "synthetic comparison source builds");
    CHECK(synth.info.kind == TOY_SIG_KIND_SYNTHETIC &&
              synth.info.stream_id != live.stream_id,
          "synthetic and live stream ids differ (kind inside the hash)");

    /* The stamping path every feed cart uses: the label in
     * hashed state takes the LIVE kind from info, which is
     * what each cart's provenance() renders. */
    toy_sig_label_t lab;
    toy_sig_label_reset(&lab);
    CHECK(lab.kind == TOY_SIG_KIND_NONE, "label starts NONE (NO SIG)");
    toy_sig_label_feed(&lab, &live, 5, 0);
    CHECK(lab.kind == TOY_SIG_KIND_LIVE &&
              lab.stream_id == live.stream_id && lab.samples_seen == 5 &&
              lab.gap_count == 0,
          "live feed stamps LIVE, advances count, no gap");
    toy_sig_label_feed(&lab, &live, 5, TOY_SIG_FLAG_UNDERRUN);
    CHECK(lab.gap_count == 1 && lab.samples_seen == 10,
          "UNDERRUN feed counts a gap in hashed state");

    /* Core refuses to reconstruct a live stream position:
     * the checkpoint/bank seek stays synthetic-only. */
    toy_sig_source_t as_live;
    memset(&as_live, 0, sizeof as_live);
    as_live.info = live;
    CHECK(toy_sig_seek_samples(&as_live, 100) == -1,
          "toy_sig_seek_samples refuses LIVE (bank/ckpt stay refused)");
    uint32_t pflags = 0;
    int16_t scratch[6];
    CHECK(toy_sig_poll(&as_live, NULL, 0, scratch, 1, &pflags) == 0,
          "toy_sig_poll has no LIVE generator (runner feeds directly)");
}

int main(void)
{
    test_select();
    test_transform();
    test_hold_accounting();
    test_info_and_label();
    printf("live_feed_test: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}

/* sigcore_test.c - standalone proof that the SIG-1 signal seam works.
 *
 * Own main, NOT wired into tools/run.sh. Defines a minimal
 * signal-consuming test cartridge in this file only (never
 * registered) and drives it through the exact shared helpers
 * the runners use (toy_sig_proof_source_init + toy_feed_step,
 * feed -> apply -> step), checking with printed hashes:
 *   1. twice-identical state hashes with the IMU feed
 *   2. a different synthetic seed changes the hash
 *   3. stream kind + stream_id are recorded in state (NONE
 *      before the first feed, SYNTHETIC + param id after)
 *   4. frames-per-step is exact every step (fixed count, no
 *      step-0 feed), and feed runs before apply at a boundary
 *   5. dial derivation emits the expected milliturn totals for
 *      a known synthetic constant-rate rotation, in both
 *      directions, past a full turn, with remainder kept
 *   6. a signal-deaf cart (fig8) is untouched by the helpers
 *   7. descriptor validation and the proof underrun refusal,
 *      including a TRUE underrun: a finite source that matches
 *      the spec but is exhausted must yield rc 2 (not the rc 1
 *      mismatch), feed() uncalled and state untouched
 *
 * Build (from repo root, Git Bash):
 *   gcc -O2 -std=c11 -Wall -Wextra -Ilibs/muse-softfloat/include
 *     -Icore tools/sigcore_test.c <all core and lib .c files>
 *     -o build/sigcore_test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "toy_cart.h"
#include "toy_math.h"
#include "toy_util.h"

static int failures;
static void expect(int ok, const char *what)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

#define TEST_STEPS 2000u /* 4000 samples at 2/step: crosses the t=30 s tap */
#define TEST_SEED UINT64_C(0x5eed)

/* --- minimal signal-consuming test cart (this file only) --- */

typedef struct {
    toy_sig_label_t label; /* kind + stream_id inside hashed state */
    toy_dial_t dial;       /* derived from the feed, in state too   */
    f64 acc;               /* softfloat sum of az, stepped          */
    int64_t sum[6];        /* per-axis integer sums of all samples  */
    uint64_t steps;
    uint64_t feed_calls;
    uint64_t samples_seen_at_apply; /* feed-before-apply witness    */
    int32_t last[6];       /* newest sample, all step() may read    */
    uint32_t fresh;        /* samples fed since last step           */
    uint32_t bad_fresh;    /* steps whose fresh != samples_per_step */
    uint32_t apply_seen;
} sig_state;

static void sig_reset(void *st_)
{
    sig_state *s = (sig_state *)st_;
    toy_memset(s, 0, sizeof *s);
    toy_sig_label_reset(&s->label);       /* NONE until first feed */
    /* Desk-scenario scales (info at first feed would be too late
     * to matter here: scale/rate are static in the descriptor). */
    toy_dial_reset(&s->dial, 128u, 100u);
}

static void sig_feed(void *st_, const int16_t *samples, uint32_t n,
                     uint32_t channels, const toy_sig_info_t *info,
                     uint32_t flags)
{
    sig_state *s = (sig_state *)st_;
    toy_sig_label_feed(&s->label, info, n, flags);
    s->feed_calls++;
    s->fresh += n;
    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t a = 0; a < 6u; a++) {
            int32_t v = samples[(size_t)i * channels + a];
            s->sum[a] += v;
            s->last[a] = v;
        }
    }
    (void)toy_dial_feed(&s->dial, samples, n, channels);
}

static void sig_apply(void *st_, const toy_event_t *ev)
{
    sig_state *s = (sig_state *)st_;
    s->apply_seen++;
    /* Whatever the event, the feed for this boundary must
     * already have landed (samples_seen includes this step's
     * frames). step 0 is excluded: no feed there. */
    if (ev->type == TOY_EV_POINTER_DOWN)
        s->samples_seen_at_apply = s->label.samples_seen;
}

static void sig_step(void *st_)
{
    sig_state *s = (sig_state *)st_;
    if (s->fresh != 2u) s->bad_fresh++;
    s->fresh = 0;
    /* step() reads state only: the newest az the feed buffered. */
    s->acc = f64_add(s->acc, (f64)toy_f64_bits_from_i32(s->last[2]));
    s->steps++;
}

static void sig_render(const void *st_, toy_fb_t *fb)
{
    (void)st_;
    toy_fb_clear(fb, 0);
}

static const toy_proof_input_t sig_proof[] = {
    { 10, { TOY_EV_POINTER_DOWN, 0, 0, 0, 0, 0, 0 } },
};

static const toy_cart_t sig_cart = {
    "sigtest", "Signal Test", "test only",
    sizeof(sig_state), 1, TEST_STEPS, 500,
    sig_reset, sig_step, sig_render,
    sig_apply, 0, 0, sig_proof, 1, TOY_CAP_POINTER,
    0, /* state_version */
    sig_feed,
    { TOY_SIG_DEVICE_IMU6, TOY_SIG_KIND_SYNTHETIC, 1u, 100u, 2u, TEST_SEED }
};

/* Drive the test cart exactly as a proof runner does. When
 * seed_override is nonzero, build the source by hand with that
 * seed instead of the descriptor spec (same feed helper). */
static int16_t frame_buf[TOY_SIG_MAX_FRAME_VALUES];
static uint64_t run_sig_cart(uint64_t seed_override, sig_state *out,
                             toy_sig_info_t *info_out)
{
    sig_state s;
    toy_sig_source_t src;
    memset(&s, 0, sizeof s);
    sig_reset(&s);
    if (seed_override) {
        toy_sig_source_synth_imu(&src, seed_override, 100u);
    } else if (toy_sig_proof_source_init(&sig_cart, &src) != 1) {
        fprintf(stderr, "proof source init failed\n");
        exit(2);
    }
    if (info_out) *info_out = src.info;
    uint32_t cur = 0;
    toy_apply_proof_at(&sig_cart, &s, 0, &cur); /* step 0: apply only */
    for (uint32_t step = 1; step <= TEST_STEPS; step++) {
        int rc = toy_feed_step(&sig_cart, &s, &src, frame_buf,
                               TOY_SIG_MAX_FRAME_VALUES);
        if (rc) { fprintf(stderr, "feed rc=%d at %u\n", rc, step); exit(2); }
        toy_apply_proof_at(&sig_cart, &s, step, &cur);
        sig_step(&s);
    }
    if (out) *out = s;
    return toy_hash_state(&s, sizeof s);
}

/* --- dial test: analytic constant-rate rotation ---
 * The desk scenario's gyro is tremor-only (gyro-coherent
 * attitude is a later study parcel), so the known rotation is
 * built as an explicit synthetic frame stream here: constant
 * gz counts, everything else zero. At 128 LSB/(deg/s) and
 * 100 Hz, gz=11520 is 90 deg/s: 400 samples (4 s) is exactly
 * one turn = 1000 milliturns; 2.5 milliturns per sample, so
 * per-call deltas alternate and only the kept remainder makes
 * the total exact. */
static void make_rotation(int16_t *frames, uint32_t n, int16_t gz)
{
    for (uint32_t i = 0; i < n * 6u; i++) frames[i] = 0;
    for (uint32_t i = 0; i < n; i++) frames[i * 6u + 5u] = gz;
}

static void dial_checks(void)
{
    static int16_t rot[400 * 6];
    toy_dial_t d;
    toy_event_t ev;
    int64_t total = 0;
    int events_ok = 1;

    make_rotation(rot, 400, 11520); /* +90 deg/s, 4 s = 1 turn */
    toy_dial_reset(&d, 128u, 100u);
    for (uint32_t c = 0; c < 200u; c++) { /* fed 2 samples per step */
        if (toy_dial_feed_event(&d, rot + c * 12u, 2, 6, &ev)) {
            if (ev.type != TOY_EV_DIAL || ev.id != 0u || ev.x || ev.y ||
                ev.flags || ev.bits)
                events_ok = 0;
            total += ev.v;
        }
    }
    printf("dial +90dps 400 samples: total %lld milliturns\n",
           (long long)total);
    expect(events_ok, "dial events are clean TOY_EV_DIAL id 0");
    expect(total == 1000 && d.total_milliturns == 1000,
           "one full synthetic turn = 1000 milliturns exactly");

    /* A second identical turn must read 2000, never wrap to 0. */
    total = 0;
    for (uint32_t c = 0; c < 400u; c++)
        total += toy_dial_feed(&d, rot + (c % 400u) * 6u, 1, 6);
    printf("dial second turn: delta %lld, running total %lld\n",
           (long long)total, (long long)d.total_milliturns);
    expect(total == 1000 && d.total_milliturns == 2000,
           "rotation past a full turn does not wrap");

    /* Reverse: -90 deg/s for 1 s = -250 milliturns. */
    make_rotation(rot, 100, -11520);
    toy_dial_reset(&d, 128u, 100u);
    total = toy_dial_feed(&d, rot, 100, 6);
    printf("dial -90dps 100 samples: total %lld milliturns\n",
           (long long)total);
    expect(total == -250 && d.total_milliturns == -250,
           "reverse rotation is symmetric (-250)");

    /* Sub-milliturn remainder is kept: 1 deg/s (128 LSB) for
     * 1 s is 2.77.. milliturns -> 2 emitted; 260 more samples
     * land exactly on 10 (360 samples = 3.6 s = 3.6 deg). */
    make_rotation(rot, 360, 128);
    toy_dial_reset(&d, 128u, 100u);
    int32_t first = toy_dial_feed(&d, rot, 100, 6);
    int32_t rest = toy_dial_feed(&d, rot + 100 * 6, 260, 6);
    printf("dial 1dps: first 100 -> %d, next 260 -> %d\n", first, rest);
    expect(first == 2 && rest == 8 && d.total_milliturns == 10,
           "sub-milliturn remainder accumulates exactly");

    /* Still device: no event, no drift. Wrong layout: ignored. */
    make_rotation(rot, 100, 0);
    toy_dial_reset(&d, 128u, 100u);
    expect(toy_dial_feed_event(&d, rot, 100, 6, &ev) == 0 &&
           d.total_milliturns == 0,
           "zero gyro-z produces no dial event");
    make_rotation(rot, 100, 11520);
    expect(toy_dial_feed(&d, rot, 100, 1) == 0 &&
           d.total_milliturns == 0,
           "non-IMU6 channel layout integrates nothing");
}

int main(void)
{
    printf("sizeof toy_sig_label_t = %zu, toy_dial_t = %zu\n",
           sizeof(toy_sig_label_t), sizeof(toy_dial_t));
    expect(toy_cart_check_signal(&sig_cart) == 0 &&
           toy_cart_check_input(&sig_cart) == 0,
           "test cart descriptor valid (input + signal)");

    /* Labelling before any feed: explicit no-signal state. */
    {
        sig_state s;
        memset(&s, 0, sizeof s);
        sig_reset(&s);
        expect(s.label.kind == TOY_SIG_KIND_NONE &&
               s.label.stream_id == 0 && s.label.samples_seen == 0,
               "after reset the label says NO SIGNAL (kind NONE)");
    }

    /* 1-4. Fed runs through the runner helpers. */
    sig_state run1, run_seed2;
    toy_sig_info_t info, info2;
    uint64_t h1 = run_sig_cart(0, &run1, &info);
    uint64_t h2 = run_sig_cart(0, 0, 0);
    uint64_t h_seed2 = run_sig_cart(TEST_SEED + 1u, &run_seed2, &info2);
    printf("sig seed 0x%llx run A %016llx run B %016llx\n",
           (unsigned long long)TEST_SEED,
           (unsigned long long)h1, (unsigned long long)h2);
    printf("sig seed 0x%llx run %016llx\n",
           (unsigned long long)(TEST_SEED + 1u),
           (unsigned long long)h_seed2);
    printf("stream identity: kind %s id %016llx (seed2 %016llx)\n",
           toy_sig_kind_name(info.kind),
           (unsigned long long)info.stream_id,
           (unsigned long long)info2.stream_id);
    expect(h1 == h2, "fed run is twice-identical");
    expect(h_seed2 != h1, "different seed changes the fed hash");
    expect(run1.label.kind == TOY_SIG_KIND_SYNTHETIC &&
           run1.label.orig_kind == TOY_SIG_KIND_SYNTHETIC &&
           run1.label.device == TOY_SIG_DEVICE_IMU6 &&
           run1.label.stream_id == info.stream_id &&
           info.stream_id != 0 &&
           run1.label.stream_id == toy_sig_param_stream_id(&info),
           "kind + stream_id recorded in state from the feed");
    expect(run_seed2.label.stream_id == info2.stream_id &&
           info2.stream_id != info.stream_id,
           "different seed records a different stream_id");
    expect(run1.feed_calls == TEST_STEPS &&
           run1.label.samples_seen == 2ull * TEST_STEPS &&
           run1.steps == TEST_STEPS && run1.bad_fresh == 0 &&
           run1.label.gap_count == 0,
           "exactly 2 frames before every step, no step-0 feed");
    expect(run1.apply_seen == 1 && run1.samples_seen_at_apply == 20ull,
           "feed for step 10 landed before apply (seen=20)");
    /* The desk stream really flowed: az sums near 1 g/sample,
     * tap included; gyro tremor keeps sums small vs 4000*90. */
    expect(run1.sum[2] > 4000ll * 14000 && run1.sum[2] < 4000ll * 17500,
           "accel samples actually entered state (az sum sane)");
    printf("dial-in-state total on desk stream: %lld milliturns\n",
           (long long)run1.dial.total_milliturns);
    expect(run1.dial.total_milliturns > -50 &&
           run1.dial.total_milliturns < 50,
           "desk gyro tremor integrates to ~no dial rotation");

    /* 5. Dial derivation on the known rotation. */
    dial_checks();

    /* 6. Deaf path: helpers must not touch fig8; every
     * registered cart must be signal-valid; carts on the feed
     * allowlist below must instantiate their synthetic proof
     * source, and every other cart must stay signal-deaf. The
     * allowlist is the reviewed change when a feed cart lands;
     * the registry count itself is never hardcoded here (it
     * went stale twice: 13 after julia, 15 after gravity). */
    {
        static const char *const feed_names[] = { "gpend", "seismo",
                                                  "attitude", "heartbeat" };
        const toy_cart_t *fig8 = toy_cart_by_name("fig8");
        int registry_ok = fig8 != 0 && fig8->feed == 0;
        int feed_carts = 0;
        for (int i = 0; i < toy_cart_count; i++) {
            const toy_cart_t *c = toy_carts[i];
            toy_sig_source_t unused;
            int created = toy_sig_proof_source_init(c, &unused);
            int listed = 0;
            for (size_t k = 0; k < sizeof feed_names / sizeof *feed_names; k++)
                if (strcmp(c->name, feed_names[k]) == 0) listed = 1;
            if (toy_cart_check_signal(c) != 0) registry_ok = 0;
            if (c->feed) {
                feed_carts++;
                if (!listed || created != 1) registry_ok = 0;
            } else if (listed || created != 0) {
                registry_ok = 0;
            }
        }
        printf("registered carts: %d (feed carts: %d)\n",
               toy_cart_count, feed_carts);
        expect(registry_ok && feed_carts >= 1,
               "registered carts signal-deaf or allowlisted feed carts, all valid");
        void *a = calloc(1, fig8->state_size);
        void *b = calloc(1, fig8->state_size);
        if (!a || !b) return 2;
        fig8->reset(a);
        fig8->reset(b);
        for (uint32_t s = 0; s < 500u; s++) {
            if (toy_feed_step(fig8, b, 0, frame_buf,
                              TOY_SIG_MAX_FRAME_VALUES) != 0)
                registry_ok = 0;
            fig8->step(a);
            fig8->step(b);
        }
        uint64_t ha = toy_hash_state(a, fig8->state_size);
        uint64_t hb = toy_hash_state(b, fig8->state_size);
        printf("fig8 plain %016llx via-feed-helper %016llx\n",
               (unsigned long long)ha, (unsigned long long)hb);
        expect(registry_ok && ha == hb,
               "deaf fig8 hash untouched by feed helpers");
        free(a);
        free(b);
    }

    /* 7. Validation + proof underrun refusal. */
    {
        toy_cart_t bad = sig_cart;
        bad.feed = 0; /* spec without feed */
        expect(toy_cart_check_signal(&bad) != 0,
               "spec without feed rejected");
        bad = sig_cart;
        bad.sig_spec.samples_per_step = 0;
        expect(toy_cart_check_signal(&bad) != 0,
               "zero samples-per-step rejected");
        bad = sig_cart;
        bad.sig_spec.proof_kind = TOY_SIG_KIND_LIVE;
        expect(toy_cart_check_signal(&bad) != 0,
               "LIVE proof kind rejected in v1");
        bad = sig_cart;
        bad.sig_spec.device = 99u;
        expect(toy_cart_check_signal(&bad) != 0,
               "unknown device rejected");

        /* Scenario allowlist is {1, 2} (Wave T3 added IMU6
         * "chest"): 0 and >=3 are rejected at validation... */
        bad = sig_cart;
        bad.sig_spec.scenario_id = 3u;
        expect(toy_cart_check_signal(&bad) != 0,
               "scenario 3 rejected (allowlist is 1 and 2)");
        bad = sig_cart;
        bad.sig_spec.scenario_id = 0u;
        /* scenario 0 zeroes that field but the rest of the spec
         * is still set, so this is a spec - and a bad one. */
        expect(toy_cart_check_signal(&bad) != 0,
               "scenario 0 rejected");

        /* ...while scenario 2 validates, instantiates through
         * the proof-source dispatch, and feeds sanely: az rests
         * near +1 g (flat on sternum), unlike the desk tilt. */
        toy_cart_t chest = sig_cart;
        chest.sig_spec.scenario_id = 2u;
        expect(toy_cart_check_signal(&chest) == 0,
               "scenario 2 (chest) accepted by validation");
        {
            sig_state s;
            toy_sig_source_t src;
            int created = toy_sig_proof_source_init(&chest, &src);
            expect(created == 1 && src.info.scenario_id == 2u &&
                   src.info.kind == TOY_SIG_KIND_SYNTHETIC,
                   "scenario 2 proof source instantiates");
            memset(&s, 0, sizeof s);
            sig_reset(&s);
            int feed_ok = created == 1;
            for (int step = 0; feed_ok && step < 200; step++) {
                if (toy_feed_step(&chest, &s, &src, frame_buf,
                                  TOY_SIG_MAX_FRAME_VALUES) != 0)
                    feed_ok = 0;
                sig_step(&s);
            }
            printf("scenario 2 feed: samples %llu az sum %lld "
                   "stream %016llx\n",
                   (unsigned long long)s.label.samples_seen,
                   (long long)s.sum[2],
                   (unsigned long long)s.label.stream_id);
            expect(feed_ok && s.feed_calls == 200u &&
                   s.label.samples_seen == 400u && s.bad_fresh == 0 &&
                   s.label.stream_id == src.info.stream_id &&
                   s.sum[2] > 400ll * 15800 && s.sum[2] < 400ll * 17000,
                   "scenario 2 feeds: 2/step exact, az rests near +1 g");
        }

        /* A device/scenario combo with no generator (MIC
         * scenario 2) passes validation but fails loud at
         * proof-source init - never a silently wrong stream. */
        {
            toy_cart_t mic2 = sig_cart;
            toy_sig_source_t unused;
            mic2.sig_spec.device = TOY_SIG_DEVICE_MIC1;
            mic2.sig_spec.scenario_id = 2u;
            expect(toy_cart_check_signal(&mic2) == 0 &&
                   toy_sig_proof_source_init(&mic2, &unused) == -1,
                   "MIC scenario 2 refused at proof-source init");
        }

        /* An absent source must fail loud WITHOUT calling feed:
         * no samples are invented into a proof. */
        sig_state s;
        toy_sig_source_t none;
        memset(&s, 0, sizeof s);
        memset(&none, 0, sizeof none); /* kind NONE */
        sig_reset(&s);
        uint64_t h_before = toy_hash_state(&s, sizeof s);
        int rc = toy_feed_step(&sig_cart, &s, &none, frame_buf,
                               TOY_SIG_MAX_FRAME_VALUES);
        printf("feed from NONE source rc=%d hash %016llx\n", rc,
               (unsigned long long)toy_hash_state(&s, sizeof s));
        expect(rc != 0 && s.feed_calls == 0 &&
               toy_hash_state(&s, sizeof s) == h_before,
               "proof feed refuses a mismatched source, state untouched");

        /* Mismatched rate is refused the same way (no resampling). */
        toy_sig_source_t wrong_rate;
        toy_sig_source_synth_imu(&wrong_rate, TEST_SEED, 200u);
        rc = toy_feed_step(&sig_cart, &s, &wrong_rate, frame_buf,
                           TOY_SIG_MAX_FRAME_VALUES);
        expect(rc != 0 && s.feed_calls == 0,
               "proof feed refuses a rate-mismatched source");

        /* TRUE underrun: a finite (2-sample TSIG replay) source
         * whose device/channels/rate MATCH the spec, drained to
         * exhaustion directly through toy_sig_poll (direct poll
         * below proves the source really held 2 samples and is
         * now at end). A subsequent toy_feed_step must return
         * rc 2 - distinct from the rc 1 mismatches above - with
         * feed() uncalled and the state bytes untouched. Honest
         * limit: toy_feed_step polls replay sources with NULL
         * bytes by design (proof helper is synthetic-only), so
         * a FRESH replay source yields rc 2 through it too; this
         * case pins the rc2/feed-uncalled/state-untouched
         * contract for a matched-but-unfeedable source, and the
         * direct poll pins that exhaustion is genuine. */
        {
            int16_t vals[12] = { 0 };
            vals[2] = 16384; vals[8] = 16384; /* az = 1 g, both samples */
            toy_sig_info_t ri;
            memset(&ri, 0, sizeof ri);
            ri.kind = TOY_SIG_KIND_REPLAY;
            ri.orig_kind = TOY_SIG_KIND_SYNTHETIC;
            ri.device = TOY_SIG_DEVICE_IMU6;
            ri.channels = 6u;
            ri.sample_rate_hz = 100u;
            ri.scale_num = 16384u;
            ri.scale2_num = 128u;
            ri.scenario_id = 1u;
            ri.seed = TEST_SEED;
            ri.n_samples = 2u;
            uint8_t tsig[TOY_SIG_HEADER_SIZE + sizeof vals];
            toy_sig_write_header(tsig, &ri,
                                 toy_sig_hash_samples(vals, 2u, 6u));
            toy_sig_write_payload(tsig + TOY_SIG_HEADER_SIZE, vals, 2u, 6u);
            toy_sig_source_t finite;
            uint32_t pflags = 99u;
            uint32_t drained = 99u;
            if (toy_sig_source_replay(&finite, tsig, sizeof tsig) == 0)
                drained = toy_sig_poll(&finite, tsig, sizeof tsig,
                                       frame_buf, 2u, &pflags);
            memset(&s, 0, sizeof s);
            sig_reset(&s);
            h_before = toy_hash_state(&s, sizeof s);
            rc = toy_feed_step(&sig_cart, &s, &finite, frame_buf,
                               TOY_SIG_MAX_FRAME_VALUES);
            printf("exhausted matched replay: direct poll got %u "
                   "flags %u, feed rc=%d hash %016llx\n",
                   drained, pflags, rc,
                   (unsigned long long)toy_hash_state(&s, sizeof s));
            expect(drained == 2u && pflags == 0u && rc == 2 &&
                   s.feed_calls == 0 &&
                   toy_hash_state(&s, sizeof s) == h_before,
                   "true underrun from exhausted matched source: rc 2, feed uncalled, state untouched");
        }
    }

    printf(failures ? "sigcore_test: FAIL (%d)\n" : "sigcore_test: PASS\n",
           failures);
    return failures ? 1 : 0;
}

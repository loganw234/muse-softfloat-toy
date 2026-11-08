/* prod_ckpt_test.c - standalone host test for the
 * toy5-flashprod pure seam: board/product_ckpt.h (save /
 * skip-live decision, cadence timer, boot-resume decision
 * table over slot scan x cart registry).
 *
 * Deliberately NOT part of tools/run.sh (mirroring
 * live_feed_test.c / product_input_test.c): build and run
 * with
 *   gcc -O2 -std=c11 -Wall -Wextra -Werror -Iboard -Icore \
 *       tools/prod_ckpt_test.c -o build/prod_ckpt_test
 * (single TU on purpose: the seam is header-only over
 * descriptor fields, so no core sources need linking).
 * Exit status 0 iff every check passes.
 *
 * Slot CHOICE among valid images of one cart is core's
 * rule and is already pinned by tools/flashslot_test.c;
 * this file pins only the product glue around it. Oracles
 * are stated from the product contract in
 * board/product_ckpt.h / board/PRODUCT.md: checkpoint
 * every TOY_PROD_CKPT_SECONDS while RUNNING + on clean
 * exit, never for a LIVE feed run, resume the newest valid
 * slot whose cart is still registered, anything else
 * opens the gallery.
 */
#include <stdio.h>
#include <string.h>

#include "product_ckpt.h"

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

static void test_constants_and_save_decision(void)
{
    CHECK(TOY_PROD_CKPT_SECONDS == 30u,
          "default cadence is the named 30 s constant");
    CHECK(TOY_PROD_CKPT_INTERVAL_MS == 30000ull,
          "interval ms derives from the seconds constant");

    CHECK(toy_prod_ckpt_save_decision(TOY_LIVE_FEED_NONE) ==
              TOY_PROD_CKPT_SAVE,
          "signal-deaf cart: save");
    CHECK(toy_prod_ckpt_save_decision(TOY_LIVE_FEED_SYNTHETIC) ==
              TOY_PROD_CKPT_SAVE,
          "synthetic feed: save (seek restores the source)");
    CHECK(toy_prod_ckpt_save_decision(TOY_LIVE_FEED_LIVE) ==
              TOY_PROD_CKPT_SKIP_LIVE,
          "LIVE feed: skip (core refuses LIVE reconstruction)");
}

static void test_timer(void)
{
    toy_prod_ckpt_timer_t t;
    toy_prod_ckpt_timer_init(&t);
    CHECK(!toy_prod_ckpt_due(&t, 100000u, 30000u),
          "unarmed timer is never due");

    toy_prod_ckpt_arm(&t, 1000u);
    CHECK(!toy_prod_ckpt_due(&t, 1000u, 30000u), "not due at arm time");
    CHECK(!toy_prod_ckpt_due(&t, 30999u, 30000u),
          "not due 1 ms before the interval");
    CHECK(toy_prod_ckpt_due(&t, 31000u, 30000u),
          "due exactly at arm + interval");
    CHECK(toy_prod_ckpt_due(&t, 99999u, 30000u),
          "still due long after (no missed-window swallow)");

    /* A completed (or failed) save re-arms from its own
     * completion time: spacing is between saves, so a long
     * flash stall never causes an immediate catch-up save. */
    toy_prod_ckpt_note(&t, 50000u);
    CHECK(!toy_prod_ckpt_due(&t, 79999u, 30000u),
          "note re-arms: not due before note + interval");
    CHECK(toy_prod_ckpt_due(&t, 80000u, 30000u),
          "note re-arms: due at note + interval");

    CHECK(!toy_prod_ckpt_due(&t, 40000u, 30000u),
          "backwards clock reads as not due");
    CHECK(!toy_prod_ckpt_due(&t, 90000u, 0u),
          "interval 0 disables cadence saves");

    toy_prod_ckpt_disarm(&t);
    CHECK(!toy_prod_ckpt_due(&t, 90000u, 30000u),
          "disarmed timer (cart stopped) is never due");

    toy_prod_ckpt_arm(&t, 0u);
    CHECK(toy_prod_ckpt_due(&t, TOY_PROD_CKPT_INTERVAL_MS,
                            TOY_PROD_CKPT_INTERVAL_MS),
          "boot-time arm at 0 is due one default interval later");
}

/* Fake registry: descriptor names only - the decision
 * reads nothing else. */
static const toy_cart_t reg_fig8 = { .name = "fig8" };
static const toy_cart_t reg_pendulum = { .name = "pendulum" };
static const toy_cart_t reg_gpend = { .name = "gpend" };
static const toy_cart_t *const reg_carts[3] = { &reg_fig8, &reg_pendulum,
                                                &reg_gpend };

static toy_prod_ckpt_slot_view_t view(int header_ok, int valid,
                                      const char *name, uint64_t step)
{
    toy_prod_ckpt_slot_view_t v;
    memset(&v, 0, sizeof v);
    v.header_ok = header_ok;
    v.valid = valid;
    if (name) {
        size_t n = strlen(name);
        if (n > sizeof v.cart_name - 1u)
            n = sizeof v.cart_name - 1u;
        memcpy(v.cart_name, name, n);
    }
    v.step_count = step;
    return v;
}

static toy_prod_ckpt_resume_t decide2(toy_prod_ckpt_slot_view_t a,
                                      toy_prod_ckpt_slot_view_t b)
{
    toy_prod_ckpt_slot_view_t slots[2] = { a, b };
    toy_prod_ckpt_resume_t out;
    memset(&out, 0x55, sizeof out);
    toy_prod_ckpt_resume_decide(slots, 2u, reg_carts, 3, &out);
    return out;
}

static void test_resume_decide(void)
{
    toy_prod_ckpt_resume_t r;

    r = decide2(view(0, 0, NULL, 0), view(0, 0, NULL, 0));
    CHECK(r.action == TOY_PROD_RESUME_NONE && r.slot == -1 &&
              r.cart_index == -1,
          "blank device: gallery, quiet NONE");

    r = decide2(view(0, 0, NULL, 0), view(1, 1, "fig8", 100u));
    CHECK(r.action == TOY_PROD_RESUME_CART && r.slot == 1 &&
              r.cart_index == 0,
          "one valid slot resumes its cart");

    r = decide2(view(1, 1, "pendulum", 100u),
                view(1, 1, "pendulum", 250u));
    CHECK(r.action == TOY_PROD_RESUME_CART && r.slot == 1 &&
              r.cart_index == 1,
          "same cart both slots: higher step_count wins");
    r = decide2(view(1, 1, "pendulum", 250u),
                view(1, 1, "pendulum", 100u));
    CHECK(r.action == TOY_PROD_RESUME_CART && r.slot == 0,
          "higher step_count in the lower slot wins");
    r = decide2(view(1, 1, "fig8", 100u), view(1, 1, "fig8", 100u));
    CHECK(r.action == TOY_PROD_RESUME_CART && r.slot == 0,
          "step_count tie resolves to the lowest slot (core rule)");

    /* The power-cut shape: newest write torn beside an
     * intact older slot. The torn slot's header step field
     * is meaningless and must never outrank validity. */
    r = decide2(view(1, 1, "fig8", 100u), view(1, 0, "fig8", 9999u));
    CHECK(r.action == TOY_PROD_RESUME_CART && r.slot == 0 &&
              r.cart_index == 0,
          "torn newest (header ok, invalid) falls back to older valid");
    r = decide2(view(0, 0, NULL, 0), view(1, 0, "fig8", 9999u));
    CHECK(r.action == TOY_PROD_RESUME_NO_VALID && r.slot == -1,
          "only a torn known-cart image: NO_VALID gallery");

    r = decide2(view(1, 0, "nosuch", 50u), view(0, 0, NULL, 0));
    CHECK(r.action == TOY_PROD_RESUME_UNKNOWN_CART && r.slot == 0 &&
              r.cart_index == -1,
          "header names a cart the registry dropped: UNKNOWN gallery");
    r = decide2(view(1, 1, "nosuch", 50u), view(0, 0, NULL, 0));
    CHECK(r.action == TOY_PROD_RESUME_UNKNOWN_CART,
          "even a self-valid image never resumes an unregistered cart");

    r = decide2(view(1, 0, "nosuch", 9999u), view(1, 1, "gpend", 10u));
    CHECK(r.action == TOY_PROD_RESUME_CART && r.slot == 1 &&
              r.cart_index == 2,
          "a resumable valid slot beats an unknown neighbour");

    /* Mixed carts can only arrive from foreign/older
     * firmware (the product clears slots on fresh starts);
     * the backend's raw highest-step rule is pinned as the
     * documented behaviour, not endorsed as comparable. */
    r = decide2(view(1, 1, "pendulum", 900u), view(1, 1, "fig8", 10u));
    CHECK(r.action == TOY_PROD_RESUME_CART && r.slot == 0 &&
              r.cart_index == 1,
          "mixed carts: raw highest step wins (documented caveat)");

    toy_prod_ckpt_resume_t z;
    memset(&z, 0x55, sizeof z);
    toy_prod_ckpt_resume_decide(NULL, 0u, reg_carts, 3, &z);
    CHECK(z.action == TOY_PROD_RESUME_NONE && z.slot == -1 &&
              z.cart_index == -1,
          "zero slots: NONE");
    toy_prod_ckpt_slot_view_t one[1] = { view(1, 1, "gpend", 7u) };
    memset(&z, 0x55, sizeof z);
    toy_prod_ckpt_resume_decide(one, 1u, reg_carts, 0, &z);
    CHECK(z.action == TOY_PROD_RESUME_UNKNOWN_CART,
          "empty registry: every named cart is unknown");
}

int main(void)
{
    test_constants_and_save_decision();
    test_timer();
    test_resume_decide();
    printf("prod_ckpt_test: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}

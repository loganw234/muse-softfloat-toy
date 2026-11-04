/* toy_cart.h - the cartridge interface.
 *
 * A cartridge owns a fixed-size state made only of f64 bit
 * patterns and integers (no pointers, no heap), zero-initialised
 * before reset() so state bytes are hashable and checkpointable
 * verbatim. step() advances exactly one fixed internal dt.
 * render() draws from state; it may convert to native double for
 * display placement only and must never write state.
 */
#ifndef TOY_CART_H
#define TOY_CART_H

#include <stddef.h>
#include <stdint.h>
#include "toy_fb.h"

/* Every host (emulator, board, WASM) allocates this much state.
 * A cartridge whose state exceeds it does not load anywhere. */
#define TOY_MAX_STATE_SIZE 65536u

typedef struct {
    const char *name;   /* short id, e.g. "fig8" */
    const char *title;  /* display title */
    const char *blurb;  /* one line: what to notice */
    size_t state_size;
    /* Live-demo pacing and the standard cross-platform proof run.
     * Keeping these in the descriptor makes a new cartridge enter
     * every runner without editing emulator, board or site code. */
    uint32_t demo_steps_per_frame;
    uint32_t proof_steps;
    uint32_t proof_report_every;
    void (*reset)(void *st);
    void (*step)(void *st);
    void (*render)(const void *st, toy_fb_t *fb);
} toy_cart_t;

extern const toy_cart_t toy_cart_fig8;
extern const toy_cart_t toy_cart_pendulum;
extern const toy_cart_t toy_cart_microscope;

extern const toy_cart_t *const toy_carts[];
extern const int toy_cart_count;

const toy_cart_t *toy_cart_by_name(const char *name);

/* FNV-1a 64 over raw state bytes. The toy's proof hook:
 * identical on host, RP2040 and RP2350 for identical step counts. */
uint64_t toy_hash_state(const void *st, size_t n);

#endif

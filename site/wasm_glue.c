/* wasm_glue.c - browser-facing instance of the sim core.
 *
 * One selected cartridge, one state allocation, one framebuffer.
 * The same state can be rendered at multiple sizes without stepping
 * it, which lets the page show both target glasses at once.
 */
#include <stddef.h>
#include <stdint.h>
#include "toy_util.h"
#include "toy_cart.h"

#define TOY_WASM_MAX_FB 360

static _Alignas(8) unsigned char toy_state[TOY_MAX_STATE_SIZE];
static uint16_t toy_pixels[TOY_WASM_MAX_FB * TOY_WASM_MAX_FB];
static toy_fb_t toy_fb;
static const toy_cart_t *toy_cart;
static uint64_t toy_steps_done;

int toy_wasm_count(void) { return toy_cart_count; }

static const toy_cart_t *cart_at(int index)
{
    if (index < 0 || index >= toy_cart_count) return 0;
    return toy_carts[index];
}

const char *toy_wasm_name(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->name : "";
}

const char *toy_wasm_title(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->title : "";
}

const char *toy_wasm_blurb(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->blurb : "";
}

uint32_t toy_wasm_demo_steps_per_frame(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->demo_steps_per_frame : 0;
}

uint32_t toy_wasm_proof_steps(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->proof_steps : 0;
}

uint32_t toy_wasm_proof_report_every(int index)
{
    const toy_cart_t *c = cart_at(index);
    return c ? c->proof_report_every : 0;
}

int toy_wasm_select(int index)
{
    const toy_cart_t *c = cart_at(index);
    if (!c || c->state_size > sizeof toy_state) return 0;
    toy_cart = c;
    toy_memset(toy_state, 0, sizeof toy_state);
    toy_cart->reset(toy_state);
    toy_steps_done = 0;
    return 1;
}

void toy_wasm_step(uint32_t count)
{
    if (!toy_cart) return;
    if (count > 100000u) count = 100000u;
    for (uint32_t i = 0; i < count; i++) toy_cart->step(toy_state);
    toy_steps_done += count;
}

int toy_wasm_render(int size)
{
    if (!toy_cart || size < 16 || size > TOY_WASM_MAX_FB) return 0;
    toy_fb.w = size;
    toy_fb.h = size;
    toy_fb.px = toy_pixels;
    toy_cart->render(toy_state, &toy_fb);
    return 1;
}

const uint16_t *toy_wasm_framebuffer(void) { return toy_pixels; }
int toy_wasm_framebuffer_width(void) { return toy_fb.w; }
int toy_wasm_framebuffer_height(void) { return toy_fb.h; }

uint32_t toy_wasm_hash_lo(void)
{
    if (!toy_cart) return 0;
    return (uint32_t)toy_hash_state(toy_state, toy_cart->state_size);
}

uint32_t toy_wasm_hash_hi(void)
{
    if (!toy_cart) return 0;
    return (uint32_t)(toy_hash_state(toy_state, toy_cart->state_size) >> 32);
}

uint32_t toy_wasm_steps_lo(void) { return (uint32_t)toy_steps_done; }
uint32_t toy_wasm_steps_hi(void) { return (uint32_t)(toy_steps_done >> 32); }
uint32_t toy_wasm_state_size(void) { return toy_cart ? (uint32_t)toy_cart->state_size : 0; }

/* toy_carts.c - cartridge registry and the state hash. */
#include "toy_cart.h"
#include "toy_util.h"

const toy_cart_t *const toy_carts[] = {
    &toy_cart_fig8,
    &toy_cart_pendulum,
    &toy_cart_carousel,
    &toy_cart_stagnation,
    &toy_cart_microscope,
    &toy_cart_harmonograph,
    &toy_cart_lorenz,
    &toy_cart_intervallens,
    &toy_cart_pendwave,
    &toy_cart_orrery,
    &toy_cart_cloud,
    &toy_cart_mandelbrot,
    &toy_cart_orbitspec,
    &toy_cart_julia,
    &toy_cart_gravity,
    &toy_cart_gpend,
    &toy_cart_seismo,
    &toy_cart_attitude,
    &toy_cart_synspec,
    &toy_cart_heartbeat,
    &toy_cart_solclock,
    &toy_cart_pond,
};
const int toy_cart_count = (int)(sizeof toy_carts / sizeof toy_carts[0]);

const toy_cart_t *toy_cart_by_name(const char *name)
{
    for (int i = 0; i < toy_cart_count; i++)
        if (toy_strcmp(toy_carts[i]->name, name) == 0) return toy_carts[i];
    return 0;
}

uint64_t toy_hash_state(const void *st, size_t n)
{
    const unsigned char *p = (const unsigned char *)st;
    uint64_t h = 1469598103934665603ull; /* FNV-1a 64 offset basis */
    for (size_t i = 0; i < n; i++) {
        h ^= (uint64_t)p[i];
        h *= 1099511628211ull; /* FNV prime */
    }
    return h;
}

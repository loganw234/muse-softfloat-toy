/* toy_carts.c - cartridge registry and the state hash. */
#include "toy_cart.h"
#include "toy_util.h"

const toy_cart_t *const toy_carts[] = {
    &toy_cart_fig8,
    &toy_cart_pendulum,
    &toy_cart_carousel,
    &toy_cart_stagnation,
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

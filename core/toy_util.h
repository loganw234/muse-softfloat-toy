/* toy_util.h - libc-free basics so the same core builds freestanding
 * for wasm32 with -nostdlib (no string.h available there). */
#ifndef TOY_UTIL_H
#define TOY_UTIL_H

#include <stddef.h>

static inline void *toy_memset(void *dst, int v, size_t n)
{
    unsigned char *d = (unsigned char *)dst;
    for (size_t i = 0; i < n; i++) d[i] = (unsigned char)v;
    return dst;
}

static inline void *toy_memcpy(void *dst, const void *src, size_t n)
{
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    for (size_t i = 0; i < n; i++) d[i] = s[i];
    return dst;
}

static inline int toy_strcmp(const char *a, const char *b)
{
    while (*a && *a == *b) { a++; b++; }
    return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

#endif

/* toy_ckpt.c - TOYCKPT1 serialise / validate / load.
 *
 * Every header integer goes through explicit little-endian byte
 * helpers below; no header struct is ever written or read as
 * memory. The payload is copied verbatim. Integer-only, no libc
 * beyond the toy_util.h basics, so host, board and wasm32 build
 * the identical code (see toy_ckpt.h for the format).
 */
#include "toy_ckpt.h"
#include "toy_util.h"

static const char toy_ckpt_magic[8] = { 'T', 'O', 'Y', 'C', 'K', 'P', 'T', '1' };

static void put_u16le(unsigned char *p, uint16_t v)
{
    p[0] = (unsigned char)(v & 0xffu);
    p[1] = (unsigned char)((v >> 8) & 0xffu);
}
static void put_u32le(unsigned char *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) p[i] = (unsigned char)(v >> (8 * i));
}
static void put_u64le(unsigned char *p, uint64_t v)
{
    for (int i = 0; i < 8; i++) p[i] = (unsigned char)(v >> (8 * i));
}
static uint16_t get_u16le(const unsigned char *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t get_u32le(const unsigned char *p)
{
    uint32_t v = 0;
    for (int i = 3; i >= 0; i--) v = (v << 8) | (uint32_t)p[i];
    return v;
}
static uint64_t get_u64le(const unsigned char *p)
{
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) v = (v << 8) | (uint64_t)p[i];
    return v;
}

/* Length of a cartridge name, or (size_t)-1 if it cannot be
 * carried: empty, longer than the 15-char cap, or non-ASCII /
 * non-printable (the field is meant to be read in a hex dump). */
static size_t name_len_ok(const char *name)
{
    if (!name) return (size_t)-1;
    size_t n = 0;
    while (name[n]) {
        unsigned char c = (unsigned char)name[n];
        if (c < 0x21u || c > 0x7eu) return (size_t)-1;
        n++;
        if (n > TOY_CKPT_NAME_CAP) return (size_t)-1;
    }
    return n ? n : (size_t)-1;
}

const char *toy_ckpt_strerror(toy_ckpt_err_t err)
{
    switch (err) {
    case TOY_CKPT_OK:                return "ok";
    case TOY_CKPT_ERR_NULL:          return "null argument";
    case TOY_CKPT_ERR_TOO_SMALL:     return "buffer shorter than header";
    case TOY_CKPT_ERR_BAD_MAGIC:     return "bad magic";
    case TOY_CKPT_ERR_FORMAT_VERSION:return "unknown format_version";
    case TOY_CKPT_ERR_HEADER_SIZE:   return "header_size != 64";
    case TOY_CKPT_ERR_HEADER_CRC:    return "header_crc32 mismatch";
    case TOY_CKPT_ERR_BAD_NAME:      return "malformed cart_name";
    case TOY_CKPT_ERR_UNKNOWN_CART:  return "unknown cart_name";
    case TOY_CKPT_ERR_CART_MISMATCH: return "cart_name != expected cart";
    case TOY_CKPT_ERR_FLAGS:         return "flags != 0";
    case TOY_CKPT_ERR_STATE_SIZE:    return "state_size mismatch";
    case TOY_CKPT_ERR_STATE_VERSION: return "cart_state_version mismatch";
    case TOY_CKPT_ERR_LENGTH:        return "total length != 64 + state_size";
    case TOY_CKPT_ERR_PAYLOAD_CRC:   return "payload_crc32 mismatch";
    case TOY_CKPT_ERR_STATE_HASH:    return "state_hash mismatch";
    case TOY_CKPT_ERR_OUT_TOO_SMALL: return "destination too small";
    }
    return "?";
}

uint32_t toy_ckpt_crc32(const void *data, size_t len)
{
    /* Bitwise reflected CRC-32/ISO-HDLC. Deliberately table-free:
     * at most 65,600 bytes per image, and no 1 KiB table to keep
     * identical (or audit) across runners. */
    const unsigned char *p = (const unsigned char *)data;
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint32_t)p[i];
        for (int bit = 0; bit < 8; bit++) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xedb88320u & mask);
        }
    }
    return ~crc;
}

size_t toy_ckpt_image_size(const toy_cart_t *cart)
{
    if (!cart) return 0;
    if (cart->state_size == 0 || cart->state_size > TOY_MAX_STATE_SIZE) return 0;
    if (name_len_ok(cart->name) == (size_t)-1) return 0;
    return TOY_CKPT_HEADER_SIZE + cart->state_size;
}

toy_ckpt_err_t toy_ckpt_serialize(const toy_cart_t *cart,
                                  const void *state, uint64_t step_count,
                                  unsigned char *out, size_t out_cap,
                                  size_t *out_len)
{
    if (!cart || !state || !out) return TOY_CKPT_ERR_NULL;
    size_t name_len = name_len_ok(cart->name);
    if (name_len == (size_t)-1) return TOY_CKPT_ERR_BAD_NAME;
    if (cart->state_size == 0 || cart->state_size > TOY_MAX_STATE_SIZE)
        return TOY_CKPT_ERR_STATE_SIZE;
    size_t total = TOY_CKPT_HEADER_SIZE + cart->state_size;
    if (out_cap < total) return TOY_CKPT_ERR_OUT_TOO_SMALL;

    unsigned char *payload = out + TOY_CKPT_HEADER_SIZE;
    toy_memcpy(payload, state, cart->state_size);

    toy_memset(out, 0, TOY_CKPT_HEADER_SIZE);
    toy_memcpy(out, toy_ckpt_magic, 8);
    put_u16le(out + 8, (uint16_t)TOY_CKPT_FORMAT_VERSION);
    put_u16le(out + 10, (uint16_t)TOY_CKPT_HEADER_SIZE);
    put_u32le(out + 12, 0u); /* flags */
    toy_memcpy(out + 16, cart->name, name_len); /* rest stays NUL */
    put_u32le(out + 32, (uint32_t)cart->state_size);
    put_u32le(out + 36, cart->state_version);
    put_u64le(out + 40, step_count);
    put_u64le(out + 48, toy_hash_state(state, cart->state_size));
    put_u32le(out + 56, toy_ckpt_crc32(payload, cart->state_size));
    put_u32le(out + 60, toy_ckpt_crc32(out, 60));

    if (out_len) *out_len = total;
    return TOY_CKPT_OK;
}

toy_ckpt_err_t toy_ckpt_parse_header(const unsigned char *buf, size_t len,
                                     toy_ckpt_info_t *info)
{
    if (!buf || !info) return TOY_CKPT_ERR_NULL;
    if (len < TOY_CKPT_HEADER_SIZE) return TOY_CKPT_ERR_TOO_SMALL;
    for (int i = 0; i < 8; i++)
        if (buf[i] != (unsigned char)toy_ckpt_magic[i])
            return TOY_CKPT_ERR_BAD_MAGIC;
    if (get_u16le(buf + 8) != TOY_CKPT_FORMAT_VERSION)
        return TOY_CKPT_ERR_FORMAT_VERSION;
    if (get_u16le(buf + 10) != TOY_CKPT_HEADER_SIZE)
        return TOY_CKPT_ERR_HEADER_SIZE;
    if (get_u32le(buf + 60) != toy_ckpt_crc32(buf, 60))
        return TOY_CKPT_ERR_HEADER_CRC;

    /* Name must carry its NUL inside the field so the registry
     * lookup below reads a bounded, terminated string. */
    size_t n = 0;
    while (n < 16 && buf[16 + n]) {
        if (buf[16 + n] < 0x21u || buf[16 + n] > 0x7eu)
            return TOY_CKPT_ERR_BAD_NAME;
        n++;
    }
    if (n == 0 || n == 16) return TOY_CKPT_ERR_BAD_NAME;

    toy_memset(info, 0, sizeof *info);
    toy_memcpy(info->cart_name, buf + 16, 16);
    info->format_version = get_u16le(buf + 8);
    info->header_size    = get_u16le(buf + 10);
    info->flags          = get_u32le(buf + 12);
    info->state_size     = get_u32le(buf + 32);
    info->state_version  = get_u32le(buf + 36);
    info->step_count     = get_u64le(buf + 40);
    info->state_hash     = get_u64le(buf + 48);
    info->payload_crc32  = get_u32le(buf + 56);
    info->header_crc32   = get_u32le(buf + 60);
    return TOY_CKPT_OK;
}

toy_ckpt_err_t toy_ckpt_validate(const unsigned char *buf, size_t len,
                                 const toy_cart_t *expected,
                                 toy_ckpt_info_t *info,
                                 const toy_cart_t **cart_out)
{
    if (!buf) return TOY_CKPT_ERR_NULL;
    toy_ckpt_info_t parsed;
    toy_ckpt_err_t err = toy_ckpt_parse_header(buf, len, &parsed);
    if (err != TOY_CKPT_OK) return err;

    const toy_cart_t *cart = toy_cart_by_name(parsed.cart_name);
    if (!cart) return TOY_CKPT_ERR_UNKNOWN_CART;
    if (expected && cart != expected) return TOY_CKPT_ERR_CART_MISMATCH;
    if (parsed.flags != 0u) return TOY_CKPT_ERR_FLAGS;
    if (parsed.state_size == 0 || parsed.state_size > TOY_MAX_STATE_SIZE ||
        (size_t)parsed.state_size != cart->state_size)
        return TOY_CKPT_ERR_STATE_SIZE;
    if (parsed.state_version != cart->state_version)
        return TOY_CKPT_ERR_STATE_VERSION;
    if (len != (size_t)TOY_CKPT_HEADER_SIZE + (size_t)parsed.state_size)
        return TOY_CKPT_ERR_LENGTH;

    const unsigned char *payload = buf + TOY_CKPT_HEADER_SIZE;
    if (toy_ckpt_crc32(payload, parsed.state_size) != parsed.payload_crc32)
        return TOY_CKPT_ERR_PAYLOAD_CRC;
    if (toy_hash_state(payload, parsed.state_size) != parsed.state_hash)
        return TOY_CKPT_ERR_STATE_HASH;

    if (info) *info = parsed;
    if (cart_out) *cart_out = cart;
    return TOY_CKPT_OK;
}

toy_ckpt_err_t toy_ckpt_load(const unsigned char *buf, size_t len,
                             const toy_cart_t *expected,
                             void *state, size_t state_cap,
                             const toy_cart_t **cart_out,
                             uint64_t *step_count_out)
{
    if (!state) return TOY_CKPT_ERR_NULL;
    toy_ckpt_info_t info;
    const toy_cart_t *cart = 0;
    toy_ckpt_err_t err = toy_ckpt_validate(buf, len, expected, &info, &cart);
    if (err != TOY_CKPT_OK) return err;
    if (state_cap < (size_t)info.state_size) return TOY_CKPT_ERR_OUT_TOO_SMALL;

    toy_memset(state, 0, info.state_size);
    toy_memcpy(state, buf + TOY_CKPT_HEADER_SIZE, info.state_size);
    /* Guard the copy itself (study section 7 step 4). Cannot
     * fail after validate unless the destination aliased buf;
     * callers must pass a state buffer distinct from the image. */
    if (toy_hash_state(state, info.state_size) != info.state_hash)
        return TOY_CKPT_ERR_STATE_HASH;

    if (cart_out) *cart_out = cart;
    if (step_count_out) *step_count_out = info.step_count;
    return TOY_CKPT_OK;
}

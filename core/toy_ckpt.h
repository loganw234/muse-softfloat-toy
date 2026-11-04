/* toy_ckpt.h - checkpoint image format TOYCKPT1.
 *
 * One byte format on every runner (docs/study-checkpoint.md
 * section 2): a 64-byte header serialised field-by-field in
 * little-endian byte order, then the cartridge state bytes
 * verbatim. The header is never a C struct on disk; the bytes
 * below are the format.
 *
 *   offset  size  field
 *   ------  ----  ------------------------------------------------
 *     0       8   magic: ASCII "TOYCKPT1"
 *     8       2   format_version  u16 LE = 1   (envelope version)
 *    10       2   header_size     u16 LE = 64  (payload starts here)
 *    12       4   flags           u32 LE = 0   (reserved; 0 in v1)
 *    16      16   cart_name: ASCII, NUL within the field
 *    32       4   state_size      u32 LE  (1 .. TOY_MAX_STATE_SIZE)
 *    36       4   cart_state_version u32 LE (toy_cart_t.state_version)
 *    40       8   step_count      u64 LE  (runner steps since reset)
 *    48       8   state_hash      u64 LE  (toy_hash_state of payload)
 *    56       4   payload_crc32   u32 LE  (CRC-32/ISO-HDLC of payload)
 *    60       4   header_crc32    u32 LE  (CRC-32 of header [0,60))
 *    64   state_size   payload: state bytes, verbatim
 *
 * Total = 64 + state_size, at most 65,600 bytes. Two integrity
 * fields on purpose: a CRC failure means storage damage; CRCs
 * passing with an FNV mismatch means intact bytes with the wrong
 * meaning (mislabelled cartridge, layout drift). Neither is
 * authentication.
 *
 * Verbatim payloads rely on the layout discipline of study
 * section 3: state fields ordered by descending alignment,
 * padding zeroed by reset(), and any layout change bumping
 * state_version. Loaders copy the payload into the runner's own
 * 8-aligned state allocation; the payload pointer inside an
 * image is never used as a state pointer in place.
 *
 * Integer-only and libc-free (toy_util.h), so the same code
 * builds freestanding for wasm32 and the boards. Storage
 * backends (host file, flash slots, IndexedDB) own the bytes
 * around this image; this file owns the image itself.
 */
#ifndef TOY_CKPT_H
#define TOY_CKPT_H

#include <stddef.h>
#include <stdint.h>
#include "toy_cart.h"

#define TOY_CKPT_HEADER_SIZE   64u
#define TOY_CKPT_FORMAT_VERSION 1u
#define TOY_CKPT_MAX_IMAGE_SIZE (TOY_CKPT_HEADER_SIZE + TOY_MAX_STATE_SIZE)
/* cart_name field is 16 bytes with a mandatory NUL inside it. */
#define TOY_CKPT_NAME_CAP      15u

typedef enum {
    TOY_CKPT_OK = 0,
    TOY_CKPT_ERR_NULL,            /* a required pointer was NULL */
    TOY_CKPT_ERR_TOO_SMALL,       /* buffer shorter than the header */
    TOY_CKPT_ERR_BAD_MAGIC,
    TOY_CKPT_ERR_FORMAT_VERSION,  /* not 1 */
    TOY_CKPT_ERR_HEADER_SIZE,     /* not 64 */
    TOY_CKPT_ERR_HEADER_CRC,
    TOY_CKPT_ERR_BAD_NAME,        /* no NUL in field, empty, >15 chars */
    TOY_CKPT_ERR_UNKNOWN_CART,    /* name not in toy_carts[] */
    TOY_CKPT_ERR_CART_MISMATCH,   /* known cart, but not the expected one */
    TOY_CKPT_ERR_FLAGS,           /* nonzero in v1 */
    TOY_CKPT_ERR_STATE_SIZE,      /* 0, > cap, or != descriptor */
    TOY_CKPT_ERR_STATE_VERSION,   /* != descriptor state_version */
    TOY_CKPT_ERR_LENGTH,          /* total != 64 + state_size (torn/extra) */
    TOY_CKPT_ERR_PAYLOAD_CRC,
    TOY_CKPT_ERR_STATE_HASH,      /* recomputed FNV != stored state_hash */
    TOY_CKPT_ERR_OUT_TOO_SMALL    /* serialise/load destination too small */
} toy_ckpt_err_t;

/* Parsed header fields. cart_name is always NUL-terminated here
 * (validation requires the NUL inside the 16-byte disk field). */
typedef struct {
    char     cart_name[16];
    uint16_t format_version;
    uint16_t header_size;
    uint32_t flags;
    uint32_t state_size;
    uint32_t state_version;
    uint64_t step_count;
    uint64_t state_hash;
    uint32_t payload_crc32;
    uint32_t header_crc32;
} toy_ckpt_info_t;

/* Human-readable name for an error code (status surfaces,
 * rejection diagnostics). Returns "?" for out-of-range codes. */
const char *toy_ckpt_strerror(toy_ckpt_err_t err);

/* CRC-32/ISO-HDLC (reflected poly 0xEDB88320, init/xorout all
 * ones), bitwise, integer only. Exposed so backends can verify
 * read-backs with the one shared implementation. */
uint32_t toy_ckpt_crc32(const void *data, size_t len);

/* Image size for a cartridge: 64 + state_size, or 0 if the
 * descriptor cannot be checkpointed (bad size or name). */
size_t toy_ckpt_image_size(const toy_cart_t *cart);

/* Serialise header + state into out[0..out_cap). On success
 * *out_len (if given) is the image size. The state is copied
 * verbatim; step_count is the runner's count since reset(). */
toy_ckpt_err_t toy_ckpt_serialize(const toy_cart_t *cart,
                                  const void *state, uint64_t step_count,
                                  unsigned char *out, size_t out_cap,
                                  size_t *out_len);

/* Parse only the 64-byte header: length, magic, format_version,
 * header_size, header CRC, and a well-formed cart_name. Does not
 * consult the registry or touch the payload; use it to learn
 * which cartridge an image names before validating fully. */
toy_ckpt_err_t toy_ckpt_parse_header(const unsigned char *buf, size_t len,
                                     toy_ckpt_info_t *info);

/* Full validation in the study section 4 order: magic,
 * format_version, header_size, header CRC, known cart_name,
 * flags, state_size vs cap and descriptor, state_version vs
 * descriptor, exact total length, payload CRC, payload FNV vs
 * stored state_hash. If expected is non-NULL the image must name
 * that cartridge. On success *cart_out (if given) is the named
 * cartridge. Never writes to any live state. */
toy_ckpt_err_t toy_ckpt_validate(const unsigned char *buf, size_t len,
                                 const toy_cart_t *expected,
                                 toy_ckpt_info_t *info,
                                 const toy_cart_t **cart_out);

/* Validate, then install the payload: state[0..state_cap) is
 * zeroed over state_size and the payload copied in. reset() is
 * NOT called - the payload replaces what reset would build.
 * On success *cart_out / *step_count_out (if given) carry the
 * header's cartridge and runner step count for stream
 * continuation. On any error the destination is untouched. */
toy_ckpt_err_t toy_ckpt_load(const unsigned char *buf, size_t len,
                             const toy_cart_t *expected,
                             void *state, size_t state_cap,
                             const toy_cart_t **cart_out,
                             uint64_t *step_count_out);

#endif

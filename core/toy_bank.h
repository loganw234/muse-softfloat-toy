/* toy_bank.h - sparse-keyframe banking, format TOYBANK1.
 *
 * The banking half of the time layer (docs/study-timebank.md
 * section 3.3): while the display sleeps the runner keeps
 * stepping and stores sparse full-state KEYFRAMES; playback
 * reproduces every intermediate state by re-stepping between
 * keyframes and VERIFYING each keyframe hash. Re-stepped states
 * are exact stepped states, bit-identical by the determinism
 * gate - never interpolation, which remains presentation-layer
 * drawing if it is ever added (docs/CARTRIDGES.md).
 *
 * PLACEMENT. This format lives in core/ next to toy_ckpt.h, not
 * in the emulator, for the same reason the checkpoint format
 * does: a bank written on any runner must replay on every
 * other, so the bytes need one shared, freestanding
 * implementation. The clock, by contrast, stays runner-owned
 * (core/toy_clock.h): this file never reads a clock - wall
 * labels arrive from the runner as plain integers and are
 * stored ALONGSIDE state bytes, never inside them (study
 * section 2.3, study-checkpoint section 6).
 *
 * RELATION TO TOYCKPT1. A keyframe IS a checkpoint: each stored
 * state below is a complete TOYCKPT1 image produced and
 * validated by core/toy_ckpt.{h,c}. The bank adds only the
 * layer-2 record around them (cart identity, step range,
 * keyframe index, wall labels, end reason). Separation is
 * deliberate at the outer level: the checkpoint image must stay
 * wall-clock-free so identical (cart, step) images cmp-equal
 * everywhere (study-checkpoint section 6), while the bank
 * header/index is exactly where labels belong.
 *
 * BYTE FORMAT (all integers little-endian, serialised
 * field-by-field; no struct is ever written as memory):
 *
 *   header, 96 bytes:
 *     0   8  magic "TOYBANK1"
 *     8   2  format_version u16 = 1
 *    10   2  header_size u16 = 96 (index starts here)
 *    12   4  flags u32 = 0
 *    16  16  cart_name, NUL within the field
 *    32   4  state_size u32
 *    36   4  state_version u32 (descriptor)
 *    40   8  total_steps u64 (final step index; the last
 *            keyframe sits exactly here)
 *    48   4  keyframe_every u32 (requested spacing in steps;
 *            0 = start+final only)
 *    52   4  keyframe_count u32
 *    56   8  wall_start_s i64 (label; 0 when invalid)
 *    64   8  wall_end_s i64
 *    72   4  tz_offset_min i32
 *    76   1  wall_valid u8 (0/1: was a clock set?)
 *    77   1  end_reason u8 (toy_bank_end_t)
 *    78   2  reserved u16 = 0
 *    80   4  index_crc32 u32 (CRC-32 over the index bytes)
 *    84   8  reserved u64 = 0
 *    92   4  header_crc32 u32 (CRC-32 over header [0,92))
 *   index, keyframe_count x 32 bytes, per keyframe:
 *     +0   8  step_index u64 (steps since reset)
 *     +8   8  state_hash u64 (toy_hash_state at this keyframe)
 *    +16   8  wall_s i64 (label only; never an ordering key)
 *    +24   4  image_offset u32 (from bank start)
 *    +28   4  image_len u32 (= 64 + state_size)
 *   then the TOYCKPT1 images, concatenated in index order.
 *
 * Total = 96 + 32*N + N*(64 + state_size), exact: torn or
 * trailing bytes reject. Ordering is by step_index, always;
 * wall labels only annotate and may be non-monotone after a
 * mid-bank clock-set (study section 7.1).
 *
 * KEYFRAME SCHEDULE. For total_steps T and spacing E > 0 the
 * keyframes are steps 0, E, 2E, ... plus a final keyframe at T
 * (once, even when T is not a multiple of E). E == 0 stores
 * steps 0 and T only ("start-only" density, study section
 * 3.3). toy_bank_keyframe_step() is the one definition; emu
 * banking and replay verification both follow it.
 *
 * BUDGET. Fixed at TOY_BANK_BUDGET_BYTES (8 MiB): at the 60 s
 * default interval an 8 h night is 480 keyframes = 6.42 MiB
 * (fig8) / 3.29 MiB (pendulum) of images, inside budget with
 * the index; a largest-possible 64 KiB-state cartridge gets
 * 127 keyframes and must space them wider (or use microSD
 * density later - same format, smaller E, bigger budget is a
 * future format_version conversation, not a silent overflow).
 * Writers refuse counts past toy_bank_max_keyframes() loudly;
 * banking never blocks or drops steps over storage (study
 * section 7.3 is runner policy built on this refusal).
 */
#ifndef TOY_BANK_H
#define TOY_BANK_H

#include <stddef.h>
#include <stdint.h>
#include "toy_cart.h"
#include "toy_ckpt.h"

#define TOY_BANK_HEADER_SIZE    96u
#define TOY_BANK_INDEX_ENTRY_SIZE 32u
#define TOY_BANK_FORMAT_VERSION 1u
#define TOY_BANK_BUDGET_BYTES   (8u * 1024u * 1024u)
#define TOY_BANK_MAX_KEYFRAMES  4096u

typedef enum {
    TOY_BANK_OK = 0,
    TOY_BANK_ERR_NULL,
    TOY_BANK_ERR_TOO_SMALL,       /* shorter than the header */
    TOY_BANK_ERR_BAD_MAGIC,
    TOY_BANK_ERR_FORMAT_VERSION,
    TOY_BANK_ERR_HEADER_SIZE,
    TOY_BANK_ERR_HEADER_CRC,
    TOY_BANK_ERR_BAD_NAME,
    TOY_BANK_ERR_UNKNOWN_CART,
    TOY_BANK_ERR_CART_MISMATCH,
    TOY_BANK_ERR_FLAGS,
    TOY_BANK_ERR_STATE_SIZE,
    TOY_BANK_ERR_STATE_VERSION,
    TOY_BANK_ERR_COUNT,           /* 0, > cap, or past budget */
    TOY_BANK_ERR_LENGTH,          /* total != layout size */
    TOY_BANK_ERR_INDEX_CRC,
    TOY_BANK_ERR_INDEX,           /* steps/offsets incoherent */
    TOY_BANK_ERR_KEYFRAME_CKPT,   /* embedded TOYCKPT1 invalid */
    TOY_BANK_ERR_INDEX_MISMATCH,  /* index != embedded header */
    TOY_BANK_ERR_REPLAY_HASH,     /* restep hash != keyframe hash */
    TOY_BANK_ERR_OUT_TOO_SMALL,
    TOY_BANK_ERR_SEQUENCE,        /* writer calls out of order */
    TOY_BANK_ERR_FEED_CART        /* signal-feed cart: replay cannot re-feed yet */
} toy_bank_err_t;

/* Why a bank ended (study section 5.2 + the USER value section
 * 7.4 adds; COMPLETE is the emu finite-run ending). */
typedef enum {
    TOY_BANK_END_COMPLETE = 0,  /* requested steps all banked */
    TOY_BANK_END_SCHEDULE = 1,  /* active window returned */
    TOY_BANK_END_PICKUP = 2,    /* wake gesture / tap */
    TOY_BANK_END_POWER = 3,     /* power loss closed the bank */
    TOY_BANK_END_FULL = 4,      /* storage budget reached */
    TOY_BANK_END_USER = 5       /* explicit user reset/switch */
} toy_bank_end_t;

typedef struct {
    char     cart_name[16];
    uint16_t format_version;
    uint16_t header_size;
    uint32_t flags;
    uint32_t state_size;
    uint32_t state_version;
    uint64_t total_steps;
    uint32_t keyframe_every;
    uint32_t keyframe_count;
    int64_t  wall_start_s;
    int64_t  wall_end_s;
    int32_t  tz_offset_min;
    uint8_t  wall_valid;
    uint8_t  end_reason;
    uint32_t index_crc32;
    uint32_t header_crc32;
} toy_bank_info_t;

typedef struct {
    uint64_t step_index;
    uint64_t state_hash;
    int64_t  wall_s;
    uint32_t image_offset;
    uint32_t image_len;
} toy_bank_keyframe_t;

const char *toy_bank_strerror(toy_bank_err_t err);
const char *toy_bank_end_name(uint8_t end_reason);

/* --- keyframe schedule & budget arithmetic --- */

/* Number of keyframes for (total_steps, every) per the
 * schedule above. */
uint32_t toy_bank_keyframe_count(uint64_t total_steps, uint32_t every);

/* Step index of keyframe `idx` (0-based). Callers pass
 * idx < toy_bank_keyframe_count(total, every). */
uint64_t toy_bank_keyframe_step(uint64_t total_steps, uint32_t every,
                                uint32_t idx);

/* Most keyframes of this cartridge that fit the fixed budget,
 * or 0 if the cartridge cannot be checkpointed at all. */
uint32_t toy_bank_max_keyframes(const toy_cart_t *cart);

/* Exact bank size for (cart, keyframe_count), or 0 if it
 * cannot be written (bad cart, count 0/cap/budget). */
size_t toy_bank_estimate_size(const toy_cart_t *cart, uint32_t keyframe_count);

/* --- writer (buffer-based v1; the runner owns stepping) ---
 * Declare the full run up front (emu knows steps + spacing),
 * add each keyframe in schedule order with its wall label,
 * then finish with the end wall label + reason. The writer
 * only copies state bytes OUT through toy_ckpt_serialize; it
 * never steps and never reads a clock. */
typedef struct {
    const toy_cart_t *cart;
    unsigned char *buf;
    size_t cap;
    size_t total_len;      /* layout size, fixed at begin */
    uint64_t total_steps;
    uint32_t every;
    uint32_t count;        /* declared keyframes */
    uint32_t added;        /* keyframes written so far */
    int64_t wall_start_s;
    uint8_t wall_valid;
    int32_t tz_offset_min;
} toy_bank_writer_t;

toy_bank_err_t toy_bank_begin(toy_bank_writer_t *w, const toy_cart_t *cart,
                              uint64_t total_steps, uint32_t every,
                              int64_t wall_start_s, uint8_t wall_valid,
                              int32_t tz_offset_min,
                              unsigned char *buf, size_t cap);

/* Append the next keyframe. step_index must be the scheduled
 * step for this slot; wall_s is the runner's current label. */
toy_bank_err_t toy_bank_add_keyframe(toy_bank_writer_t *w, const void *state,
                                     uint64_t step_index, int64_t wall_s);

/* Seal the bank (header + index CRCs). Requires every declared
 * keyframe added. *out_len (if given) is the bank size. */
toy_bank_err_t toy_bank_finish(toy_bank_writer_t *w, int64_t wall_end_s,
                               uint8_t end_reason, size_t *out_len);

/* --- reader ---
 * Full validation: header (magic/version/size/CRC), known
 * cart, descriptor sizes/versions, budget-shaped count, exact
 * length, index CRC, monotone steps ending at total_steps,
 * each embedded TOYCKPT1 via toy_ckpt_validate, and index
 * entries agreeing with the embedded headers. Never writes to
 * any live state. */
toy_bank_err_t toy_bank_parse(const unsigned char *buf, size_t len,
                              const toy_cart_t *expected,
                              toy_bank_info_t *info,
                              const toy_cart_t **cart_out);

/* Index entry idx from an already-parsed bank. */
toy_bank_err_t toy_bank_keyframe(const unsigned char *buf, size_t len,
                                 const toy_bank_info_t *info, uint32_t idx,
                                 toy_bank_keyframe_t *out);

/* The playback proof (study section 4.3): seed scratch state
 * from keyframe 0, re-step to each later keyframe (replaying
 * the descriptor's proof_inputs at their absolute steps, as
 * every runner's step loop does), and require toy_hash_state
 * to equal the stored keyframe hash at every boundary. On
 * TOY_BANK_ERR_REPLAY_HASH, *fail_idx / *fail_step /
 * *want_hash / *got_hash (when given) name the first bad
 * keyframe: storage corruption or a nondeterminism bug -
 * playback must stop there and say so. On success
 * *verified_out (if given) is the keyframe count checked. */
toy_bank_err_t toy_bank_verify_replay(const unsigned char *buf, size_t len,
                                      void *scratch, size_t scratch_cap,
                                      uint32_t *verified_out,
                                      uint32_t *fail_idx,
                                      uint64_t *fail_step,
                                      uint64_t *want_hash,
                                      uint64_t *got_hash);

/* --- morning report v1 (study section 5.2, no cart changes) ---
 * A layer-2 record from the bank alone. steps_expected_1x is
 * demo_rate x wall span and is 0 when the clock was unset
 * (wall_valid == 0 prints as "n/a", never invented). The
 * caller states whether toy_bank_verify_replay has checked
 * the hash chain. Text rendering is the runner's (emu prints
 * it; the site/device draw it) - this struct is the record. */
typedef struct {
    char     cart_name[16];
    uint64_t steps_at_start;
    uint64_t steps_at_end;
    uint64_t steps_banked;       /* end - start */
    uint64_t steps_expected_1x;  /* 0 when wall_valid == 0 */
    uint64_t hash_start;
    uint64_t hash_end;
    uint32_t keyframe_count;
    uint32_t keyframe_every;
    int64_t  wall_start_s;
    int64_t  wall_end_s;
    int32_t  tz_offset_min;
    uint8_t  wall_valid;
    uint8_t  end_reason;
    uint8_t  playback_verified;
    size_t   bank_bytes;
} toy_morning_report_t;

toy_bank_err_t toy_bank_morning_report(const unsigned char *buf, size_t len,
                                       uint8_t playback_verified,
                                       toy_morning_report_t *out,
                                       const toy_cart_t **cart_out);

#endif

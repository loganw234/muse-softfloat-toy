/* ckpt_flash.h - internal-flash checkpoint slot backend (board).
 *
 * Stores TOYCKPT1 images (core/toy_ckpt.h) in the A/B slot pair
 * at the end of QSPI flash whose geometry core/toy_ckpt_slot.h
 * defines. This is the resume-last store: one running cartridge,
 * newest valid image wins. Checkpoint banks/history belong on
 * the RP2350 kit's microSD, not here (study-checkpoint.md 5.2).
 *
 * NOT wired into main_board.c behaviour: nothing here runs at
 * boot, no slot is read or written by the proof firmware, and
 * the API is linked only so Friday's wiring is a call, not a
 * build change. Compile-proven on RP2040 + RP2350; the first
 * on-device write/read proof is Friday's checklist item.
 *
 * Flash-size source: PICO_FLASH_SIZE_BYTES from the board header
 * (pico: 2 MiB; pico2: 4 MiB assumed - the 16 MiB module slot
 * placement in board/README.md lands at the true flash end only
 * once a 16 MiB board header defines it). Writes go through the
 * SDK's flash_safe_execute (pico/flash.h): interrupts and the
 * other core are the SDK helper's problem, never hand-rolled
 * here. This firmware is single-core, so the build defines
 * PICO_FLASH_ASSUME_CORE1_SAFE=1 (board/CMakeLists.txt); a
 * future core-1 firmware must drop that and call
 * flash_safe_execute_core_init() on core 1 instead.
 *
 * Write path contract: the image buffer must be in SRAM, not an
 * XIP flash pointer (flash is unreadable while programming). A
 * write validates the image in RAM, erases+programs the target
 * slot (never the newest valid one, via write_next), then reads
 * back through XIP and re-validates + byte-compares, so a dying
 * sector fails at write time rather than at resume time.
 * Cost: up to 17 sector erases + 257 page programs at cap size
 * with XIP stalled (fig8 today: 17 erases, 55 programs - the
 * whole slot is erased so slot state stays uniform). Call
 * between frames, never inside step()/render().
 *
 * Selftest: TOY_CKPT_FLASH_SELFTEST (CMake option, OFF by
 * default) compiles toy_ckpt_flash_selftest_roundtrip() as a
 * callable one-shot. Nothing calls it even then - Friday's
 * wiring adds a deliberate boot-time call in a throwaway build.
 * Default OFF because a boot-time write on every power-up would
 * burn erase cycles for no product reason and would destroy a
 * real resume slot if ever shipped by accident.
 */
#ifndef TOY_CKPT_FLASH_H
#define TOY_CKPT_FLASH_H

#include <stddef.h>
#include <stdint.h>
#include "toy_ckpt.h"
#include "toy_ckpt_slot.h"

#ifndef TOY_CKPT_FLASH_SELFTEST
#define TOY_CKPT_FLASH_SELFTEST 0
#endif

typedef enum {
    TOY_CKPT_FLASH_OK = 0,
    TOY_CKPT_FLASH_ERR_ARG,     /* NULL/bad arg, or data in XIP flash */
    TOY_CKPT_FLASH_ERR_LAYOUT,  /* no PICO_FLASH_SIZE_BYTES / no fit */
    TOY_CKPT_FLASH_ERR_RANGE,   /* slot index or length out of range */
    TOY_CKPT_FLASH_ERR_IMAGE,   /* image failed toy_ckpt_validate() */
    TOY_CKPT_FLASH_ERR_NO_VALID,/* scan found no valid slot */
    TOY_CKPT_FLASH_ERR_SAFE,    /* flash_safe_execute refused (see
                                   toy_ckpt_flash_last_safe_rc()) */
    TOY_CKPT_FLASH_ERR_VERIFY   /* read-back mismatch after program */
} toy_ckpt_flash_err_t;

/* Resolve (once, lazily is fine) and return the slot layout for
 * this build's PICO_FLASH_SIZE_BYTES. Returns NULL when the
 * layout cannot be resolved; every other call then fails with
 * TOY_CKPT_FLASH_ERR_LAYOUT. Firmware-reserve argument is 0:
 * placement from the flash end is the protection, and Friday
 * checks the shipped BIN size against layout->base_offset. */
const toy_ckpt_slot_layout_t *toy_ckpt_flash_layout(void);

/* Pico SDK return code from the last flash_safe_execute call
 * (PICO_OK == 0 on success), for boot-log diagnosis. */
int toy_ckpt_flash_last_safe_rc(void);

/* XIP pointer to a slot's first byte, or NULL for a bad index.
 * Read-only convenience for debugging; reads for use should go
 * through toy_ckpt_flash_read_slot()/load_newest(). */
const unsigned char *toy_ckpt_flash_slot_xip(uint32_t index);

/* Copy slot `index`'s image (64 + state_size bytes) to out after
 * full validation for `expected` (NULL = any cart). On success
 * *out_len / *info_out (if given) carry the image length/header.
 * Blank/torn slots fail ERR_NO_VALID; a too-small out fails
 * ERR_RANGE and copies nothing. */
toy_ckpt_flash_err_t toy_ckpt_flash_read_slot(uint32_t index,
                                              const toy_cart_t *expected,
                                              unsigned char *out,
                                              size_t out_cap,
                                              size_t *out_len,
                                              toy_ckpt_info_t *info_out);

/* Scan all slots and report the newest valid one (highest
 * step_count among fully valid images for `expected`). */
toy_ckpt_flash_err_t toy_ckpt_flash_find_newest(const toy_cart_t *expected,
                                                uint32_t *index_out,
                                                toy_ckpt_info_t *info_out);

/* Find newest + read_slot in one call (the resume-last read). */
toy_ckpt_flash_err_t toy_ckpt_flash_load_newest(const toy_cart_t *expected,
                                                unsigned char *out,
                                                size_t out_cap,
                                                size_t *out_len,
                                                uint32_t *index_out,
                                                toy_ckpt_info_t *info_out);

/* Erase + program + verify one explicit slot. The image must
 * validate in RAM (any cart) and fit the slot; bytes beyond the
 * image's page-rounded length stay erased (whole-slot erase). */
toy_ckpt_flash_err_t toy_ckpt_flash_write_slot(uint32_t index,
                                               const unsigned char *image,
                                               size_t image_len);

/* Write to the rotation slot (after the newest valid image for
 * the image's own cartridge; slot 0 when none valid), so the
 * previous checkpoint survives a power cut mid-write. Reports
 * the slot written via *index_out (if given). */
toy_ckpt_flash_err_t toy_ckpt_flash_write_next(const unsigned char *image,
                                               size_t image_len,
                                               uint32_t *index_out);

/* Erase one slot, or the whole slot region. Erased slots probe
 * blank and are skipped by scans. */
toy_ckpt_flash_err_t toy_ckpt_flash_erase_slot(uint32_t index);
toy_ckpt_flash_err_t toy_ckpt_flash_erase_all(void);

#if TOY_CKPT_FLASH_SELFTEST
/* One-shot Friday proof: write `image` with write_next, read it
 * back with load_newest, byte-compare. Compiled only with
 * -DTOY_CKPT_FLASH_SELFTEST=ON; never called by this firmware. */
toy_ckpt_flash_err_t toy_ckpt_flash_selftest_roundtrip(const unsigned char *image,
                                                       size_t image_len,
                                                       uint32_t *index_out);
#endif

#endif

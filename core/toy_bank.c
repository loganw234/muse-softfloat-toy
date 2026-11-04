/* toy_bank.c - TOYBANK1 write / parse / verified replay.
 *
 * Same serialisation discipline as toy_ckpt.c: every integer
 * through explicit little-endian helpers, keyframe payloads
 * are TOYCKPT1 images from toy_ckpt_serialize, integer-only
 * and libc-free beyond toy_util.h so host, board and wasm32
 * build identical code. No clock is read here; wall labels are
 * integers the runner hands in (see toy_bank.h placement note).
 */
#include "toy_bank.h"
#include "toy_util.h"

static const char toy_bank_magic[8] = { 'T', 'O', 'Y', 'B', 'A', 'N', 'K', '1' };

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

const char *toy_bank_strerror(toy_bank_err_t err)
{
    switch (err) {
    case TOY_BANK_OK:                return "ok";
    case TOY_BANK_ERR_NULL:          return "null argument";
    case TOY_BANK_ERR_TOO_SMALL:     return "buffer shorter than header";
    case TOY_BANK_ERR_BAD_MAGIC:     return "bad magic";
    case TOY_BANK_ERR_FORMAT_VERSION:return "unknown format_version";
    case TOY_BANK_ERR_HEADER_SIZE:   return "header_size != 96";
    case TOY_BANK_ERR_HEADER_CRC:    return "header_crc32 mismatch";
    case TOY_BANK_ERR_BAD_NAME:      return "malformed cart_name";
    case TOY_BANK_ERR_UNKNOWN_CART:  return "unknown cart_name";
    case TOY_BANK_ERR_CART_MISMATCH: return "cart_name != expected cart";
    case TOY_BANK_ERR_FLAGS:         return "flags != 0";
    case TOY_BANK_ERR_STATE_SIZE:    return "state_size mismatch";
    case TOY_BANK_ERR_STATE_VERSION: return "cart_state_version mismatch";
    case TOY_BANK_ERR_COUNT:         return "keyframe_count 0/cap/budget";
    case TOY_BANK_ERR_LENGTH:        return "total length != bank layout";
    case TOY_BANK_ERR_INDEX_CRC:     return "index_crc32 mismatch";
    case TOY_BANK_ERR_INDEX:         return "index steps/offsets incoherent";
    case TOY_BANK_ERR_KEYFRAME_CKPT: return "embedded TOYCKPT1 invalid";
    case TOY_BANK_ERR_INDEX_MISMATCH:return "index != embedded ckpt header";
    case TOY_BANK_ERR_REPLAY_HASH:   return "restep hash != keyframe hash";
    case TOY_BANK_ERR_OUT_TOO_SMALL: return "destination too small";
    case TOY_BANK_ERR_SEQUENCE:      return "writer calls out of order";
    case TOY_BANK_ERR_FEED_CART:     return "signal-feed carts cannot bank yet";
    }
    return "?";
}

const char *toy_bank_end_name(uint8_t end_reason)
{
    switch (end_reason) {
    case TOY_BANK_END_COMPLETE: return "COMPLETE";
    case TOY_BANK_END_SCHEDULE: return "SCHEDULE";
    case TOY_BANK_END_PICKUP:   return "PICKUP";
    case TOY_BANK_END_POWER:    return "POWER";
    case TOY_BANK_END_FULL:     return "FULL";
    case TOY_BANK_END_USER:     return "USER";
    }
    return "?";
}

/* --- schedule & budget --- */

uint32_t toy_bank_keyframe_count(uint64_t total_steps, uint32_t every)
{
    if (total_steps == 0) return 1;          /* just step 0 */
    if (every == 0) return 2;                /* 0 and total */
    uint64_t n = 1 + total_steps / every + (total_steps % every ? 1 : 0);
    return n > 0xffffffffull ? 0xffffffffu : (uint32_t)n;
}

uint64_t toy_bank_keyframe_step(uint64_t total_steps, uint32_t every,
                                uint32_t idx)
{
    uint32_t count = toy_bank_keyframe_count(total_steps, every);
    if (idx + 1 >= count) return total_steps; /* also clamps bad idx */
    if (every == 0) return 0;                 /* idx 0 of {0, total} */
    return (uint64_t)idx * (uint64_t)every;
}

static size_t bank_layout_size(size_t state_size, uint32_t count)
{
    return (size_t)TOY_BANK_HEADER_SIZE +
           (size_t)count * ((size_t)TOY_BANK_INDEX_ENTRY_SIZE +
                            (size_t)TOY_CKPT_HEADER_SIZE + state_size);
}

uint32_t toy_bank_max_keyframes(const toy_cart_t *cart)
{
    if (!cart || toy_ckpt_image_size(cart) == 0) return 0;
    size_t per = (size_t)TOY_BANK_INDEX_ENTRY_SIZE +
                 (size_t)TOY_CKPT_HEADER_SIZE + cart->state_size;
    uint32_t n = (uint32_t)(((size_t)TOY_BANK_BUDGET_BYTES -
                             (size_t)TOY_BANK_HEADER_SIZE) / per);
    if (n > TOY_BANK_MAX_KEYFRAMES) n = TOY_BANK_MAX_KEYFRAMES;
    return n;
}

size_t toy_bank_estimate_size(const toy_cart_t *cart, uint32_t keyframe_count)
{
    if (!cart || keyframe_count == 0) return 0;
    if (keyframe_count > toy_bank_max_keyframes(cart)) return 0;
    return bank_layout_size(cart->state_size, keyframe_count);
}

/* --- writer --- */

static unsigned char *index_entry(unsigned char *buf, uint32_t idx)
{
    return buf + TOY_BANK_HEADER_SIZE + (size_t)idx * TOY_BANK_INDEX_ENTRY_SIZE;
}
static const unsigned char *index_entry_c(const unsigned char *buf, uint32_t idx)
{
    return buf + TOY_BANK_HEADER_SIZE + (size_t)idx * TOY_BANK_INDEX_ENTRY_SIZE;
}

toy_bank_err_t toy_bank_begin(toy_bank_writer_t *w, const toy_cart_t *cart,
                              uint64_t total_steps, uint32_t every,
                              int64_t wall_start_s, uint8_t wall_valid,
                              int32_t tz_offset_min,
                              unsigned char *buf, size_t cap)
{
    if (!w || !cart || !buf) return TOY_BANK_ERR_NULL;
    /* Replay re-steps without a signal source (source position
     * is runner-owned, like checkpoints): refuse feed carts
     * loudly until re-feeding lands with the first such cart. */
    if (cart->feed) return TOY_BANK_ERR_FEED_CART;
    uint32_t count = toy_bank_keyframe_count(total_steps, every);
    size_t total = toy_bank_estimate_size(cart, count);
    if (total == 0) return TOY_BANK_ERR_COUNT; /* past budget/cap */
    if (cap < total) return TOY_BANK_ERR_OUT_TOO_SMALL;

    toy_memset(w, 0, sizeof *w);
    w->cart = cart;
    w->buf = buf;
    w->cap = cap;
    w->total_len = total;
    w->total_steps = total_steps;
    w->every = every;
    w->count = count;
    w->wall_start_s = wall_start_s;
    w->wall_valid = wall_valid ? 1u : 0u;
    w->tz_offset_min = tz_offset_min;
    toy_memset(buf, 0, total); /* header sealed at finish */
    return TOY_BANK_OK;
}

toy_bank_err_t toy_bank_add_keyframe(toy_bank_writer_t *w, const void *state,
                                     uint64_t step_index, int64_t wall_s)
{
    if (!w || !state) return TOY_BANK_ERR_NULL;
    if (!w->cart || w->added >= w->count) return TOY_BANK_ERR_SEQUENCE;
    if (step_index != toy_bank_keyframe_step(w->total_steps, w->every, w->added))
        return TOY_BANK_ERR_SEQUENCE;

    size_t image_len = toy_ckpt_image_size(w->cart);
    size_t offset = (size_t)TOY_BANK_HEADER_SIZE +
                    (size_t)w->count * TOY_BANK_INDEX_ENTRY_SIZE +
                    (size_t)w->added * image_len;
    size_t written = 0;
    toy_ckpt_err_t e = toy_ckpt_serialize(w->cart, state, step_index,
                                          w->buf + offset, image_len, &written);
    if (e != TOY_CKPT_OK) return TOY_BANK_ERR_KEYFRAME_CKPT;

    unsigned char *en = index_entry(w->buf, w->added);
    put_u64le(en + 0, step_index);
    put_u64le(en + 8, toy_hash_state(state, w->cart->state_size));
    put_u64le(en + 16, (uint64_t)wall_s);
    put_u32le(en + 24, (uint32_t)offset);
    put_u32le(en + 28, (uint32_t)written);
    w->added++;
    return TOY_BANK_OK;
}

toy_bank_err_t toy_bank_finish(toy_bank_writer_t *w, int64_t wall_end_s,
                               uint8_t end_reason, size_t *out_len)
{
    if (!w) return TOY_BANK_ERR_NULL;
    if (!w->cart || w->added != w->count) return TOY_BANK_ERR_SEQUENCE;
    unsigned char *h = w->buf;

    toy_memset(h, 0, TOY_BANK_HEADER_SIZE);
    toy_memcpy(h, toy_bank_magic, 8);
    put_u16le(h + 8, (uint16_t)TOY_BANK_FORMAT_VERSION);
    put_u16le(h + 10, (uint16_t)TOY_BANK_HEADER_SIZE);
    put_u32le(h + 12, 0u); /* flags */
    size_t name_len = 0;
    while (w->cart->name[name_len]) name_len++; /* ckpt-checked <=15 */
    toy_memcpy(h + 16, w->cart->name, name_len);
    put_u32le(h + 32, (uint32_t)w->cart->state_size);
    put_u32le(h + 36, w->cart->state_version);
    put_u64le(h + 40, w->total_steps);
    put_u32le(h + 48, w->every);
    put_u32le(h + 52, w->count);
    put_u64le(h + 56, (uint64_t)w->wall_start_s);
    put_u64le(h + 64, (uint64_t)wall_end_s);
    put_u32le(h + 72, (uint32_t)w->tz_offset_min);
    h[76] = w->wall_valid;
    h[77] = end_reason;
    put_u32le(h + 80, toy_ckpt_crc32(h + TOY_BANK_HEADER_SIZE,
                                     (size_t)w->count * TOY_BANK_INDEX_ENTRY_SIZE));
    put_u32le(h + 92, toy_ckpt_crc32(h, 92));

    if (out_len) *out_len = w->total_len;
    return TOY_BANK_OK;
}

/* --- reader --- */

static toy_bank_err_t read_keyframe(const unsigned char *buf,
                                    const toy_bank_info_t *info, uint32_t idx,
                                    toy_bank_keyframe_t *out)
{
    const unsigned char *en = index_entry_c(buf, idx);
    out->step_index = get_u64le(en + 0);
    out->state_hash = get_u64le(en + 8);
    out->wall_s = (int64_t)get_u64le(en + 16);
    out->image_offset = get_u32le(en + 24);
    out->image_len = get_u32le(en + 28);
    (void)info;
    return TOY_BANK_OK;
}

toy_bank_err_t toy_bank_parse(const unsigned char *buf, size_t len,
                              const toy_cart_t *expected,
                              toy_bank_info_t *info,
                              const toy_cart_t **cart_out)
{
    if (!buf) return TOY_BANK_ERR_NULL;
    if (len < TOY_BANK_HEADER_SIZE) return TOY_BANK_ERR_TOO_SMALL;
    for (int i = 0; i < 8; i++)
        if (buf[i] != (unsigned char)toy_bank_magic[i])
            return TOY_BANK_ERR_BAD_MAGIC;
    if (get_u16le(buf + 8) != TOY_BANK_FORMAT_VERSION)
        return TOY_BANK_ERR_FORMAT_VERSION;
    if (get_u16le(buf + 10) != TOY_BANK_HEADER_SIZE)
        return TOY_BANK_ERR_HEADER_SIZE;
    if (get_u32le(buf + 92) != toy_ckpt_crc32(buf, 92))
        return TOY_BANK_ERR_HEADER_CRC;

    size_t n = 0;
    while (n < 16 && buf[16 + n]) {
        if (buf[16 + n] < 0x21u || buf[16 + n] > 0x7eu)
            return TOY_BANK_ERR_BAD_NAME;
        n++;
    }
    if (n == 0 || n == 16) return TOY_BANK_ERR_BAD_NAME;

    toy_bank_info_t parsed;
    toy_memset(&parsed, 0, sizeof parsed);
    toy_memcpy(parsed.cart_name, buf + 16, 16);
    parsed.format_version = get_u16le(buf + 8);
    parsed.header_size = get_u16le(buf + 10);
    parsed.flags = get_u32le(buf + 12);
    parsed.state_size = get_u32le(buf + 32);
    parsed.state_version = get_u32le(buf + 36);
    parsed.total_steps = get_u64le(buf + 40);
    parsed.keyframe_every = get_u32le(buf + 48);
    parsed.keyframe_count = get_u32le(buf + 52);
    parsed.wall_start_s = (int64_t)get_u64le(buf + 56);
    parsed.wall_end_s = (int64_t)get_u64le(buf + 64);
    parsed.tz_offset_min = (int32_t)get_u32le(buf + 72);
    parsed.wall_valid = buf[76];
    parsed.end_reason = buf[77];
    parsed.index_crc32 = get_u32le(buf + 80);
    parsed.header_crc32 = get_u32le(buf + 92);
    if (parsed.wall_valid > 1u || parsed.end_reason > TOY_BANK_END_USER ||
        get_u16le(buf + 78) != 0u || get_u64le(buf + 84) != 0u)
        return TOY_BANK_ERR_FLAGS;

    const toy_cart_t *cart = toy_cart_by_name(parsed.cart_name);
    if (!cart) return TOY_BANK_ERR_UNKNOWN_CART;
    if (expected && cart != expected) return TOY_BANK_ERR_CART_MISMATCH;
    if (parsed.flags != 0u) return TOY_BANK_ERR_FLAGS;
    if (parsed.state_size == 0 || parsed.state_size > TOY_MAX_STATE_SIZE ||
        (size_t)parsed.state_size != cart->state_size)
        return TOY_BANK_ERR_STATE_SIZE;
    if (parsed.state_version != cart->state_version)
        return TOY_BANK_ERR_STATE_VERSION;
    if (parsed.keyframe_count == 0 ||
        parsed.keyframe_count > toy_bank_max_keyframes(cart))
        return TOY_BANK_ERR_COUNT;
    /* Index must agree with the header schedule, not merely be
     * monotone: count and every step are derived values. */
    if (parsed.keyframe_count !=
        toy_bank_keyframe_count(parsed.total_steps, parsed.keyframe_every))
        return TOY_BANK_ERR_COUNT;
    if (len != bank_layout_size(cart->state_size, parsed.keyframe_count))
        return TOY_BANK_ERR_LENGTH;
    if (toy_ckpt_crc32(buf + TOY_BANK_HEADER_SIZE,
                       (size_t)parsed.keyframe_count *
                           TOY_BANK_INDEX_ENTRY_SIZE) != parsed.index_crc32)
        return TOY_BANK_ERR_INDEX_CRC;

    /* Index coherence + each embedded checkpoint, in order. */
    size_t image_len = toy_ckpt_image_size(cart);
    uint64_t prev_step = 0;
    for (uint32_t i = 0; i < parsed.keyframe_count; i++) {
        toy_bank_keyframe_t kf;
        read_keyframe(buf, &parsed, i, &kf);
        size_t want_off = (size_t)TOY_BANK_HEADER_SIZE +
                          (size_t)parsed.keyframe_count *
                              TOY_BANK_INDEX_ENTRY_SIZE +
                          (size_t)i * image_len;
        if ((size_t)kf.image_offset != want_off ||
            (size_t)kf.image_len != image_len)
            return TOY_BANK_ERR_INDEX;
        if (i > 0 && kf.step_index <= prev_step) return TOY_BANK_ERR_INDEX;
        if (kf.step_index != toy_bank_keyframe_step(parsed.total_steps,
                                                    parsed.keyframe_every,
                                                    i))
            return TOY_BANK_ERR_INDEX;
        prev_step = kf.step_index;

        toy_ckpt_info_t ci;
        if (toy_ckpt_validate(buf + kf.image_offset, kf.image_len, cart,
                              &ci, NULL) != TOY_CKPT_OK)
            return TOY_BANK_ERR_KEYFRAME_CKPT;
        if (ci.step_count != kf.step_index || ci.state_hash != kf.state_hash)
            return TOY_BANK_ERR_INDEX_MISMATCH;
    }
    if (prev_step != parsed.total_steps) return TOY_BANK_ERR_INDEX;

    if (info) *info = parsed;
    if (cart_out) *cart_out = cart;
    return TOY_BANK_OK;
}

toy_bank_err_t toy_bank_keyframe(const unsigned char *buf, size_t len,
                                 const toy_bank_info_t *info, uint32_t idx,
                                 toy_bank_keyframe_t *out)
{
    if (!buf || !info || !out) return TOY_BANK_ERR_NULL;
    if (idx >= info->keyframe_count) return TOY_BANK_ERR_INDEX;
    if (len < bank_layout_size(info->state_size, info->keyframe_count))
        return TOY_BANK_ERR_LENGTH;
    return read_keyframe(buf, info, idx, out);
}

toy_bank_err_t toy_bank_verify_replay(const unsigned char *buf, size_t len,
                                      void *scratch, size_t scratch_cap,
                                      uint32_t *verified_out,
                                      uint32_t *fail_idx,
                                      uint64_t *fail_step,
                                      uint64_t *want_hash,
                                      uint64_t *got_hash)
{
    if (!scratch) return TOY_BANK_ERR_NULL;
    toy_bank_info_t info;
    const toy_cart_t *cart = 0;
    toy_bank_err_t err = toy_bank_parse(buf, len, NULL, &info, &cart);
    if (err != TOY_BANK_OK) return err;
    if (cart->feed) return TOY_BANK_ERR_FEED_CART; /* cannot re-feed yet */
    if (scratch_cap < cart->state_size) return TOY_BANK_ERR_OUT_TOO_SMALL;

    /* Seed from keyframe 0 through the checkpoint loader (which
     * re-validates it and guards the copy). */
    toy_bank_keyframe_t kf0;
    read_keyframe(buf, &info, 0, &kf0);
    uint64_t at_step = 0;
    if (toy_ckpt_load(buf + kf0.image_offset, kf0.image_len, cart,
                      scratch, scratch_cap, NULL, &at_step) != TOY_CKPT_OK)
        return TOY_BANK_ERR_KEYFRAME_CKPT;

    /* Proof entries due at or before the seed step are already
     * reflected in the seed state (the writer applied them);
     * the check in toy_cart_check_input bounds each step to the
     * per-step cap, so none can still be pending. Skip them. */
    uint32_t cursor = 0;
    if (cart->proof_inputs) {
        while (cursor < cart->proof_input_count &&
               (uint64_t)cart->proof_inputs[cursor].step <= at_step)
            cursor++;
    }
    if (toy_hash_state(scratch, cart->state_size) != kf0.state_hash)
        return TOY_BANK_ERR_KEYFRAME_CKPT; /* load already guarantees */

    if (verified_out) *verified_out = 1;
    uint64_t prev_step = kf0.step_index;
    for (uint32_t i = 1; i < info.keyframe_count; i++) {
        toy_bank_keyframe_t kf;
        read_keyframe(buf, &info, i, &kf);
        for (uint64_t s = prev_step + 1; s <= kf.step_index; s++) {
            toy_apply_proof_at(cart, scratch, (uint32_t)s, &cursor);
            cart->step(scratch);
        }
        uint64_t got = toy_hash_state(scratch, cart->state_size);
        if (got != kf.state_hash) {
            if (fail_idx) *fail_idx = i;
            if (fail_step) *fail_step = kf.step_index;
            if (want_hash) *want_hash = kf.state_hash;
            if (got_hash) *got_hash = got;
            return TOY_BANK_ERR_REPLAY_HASH;
        }
        if (verified_out) *verified_out = i + 1;
        prev_step = kf.step_index;
    }
    return TOY_BANK_OK;
}

toy_bank_err_t toy_bank_morning_report(const unsigned char *buf, size_t len,
                                       uint8_t playback_verified,
                                       toy_morning_report_t *out,
                                       const toy_cart_t **cart_out)
{
    if (!out) return TOY_BANK_ERR_NULL;
    toy_bank_info_t info;
    const toy_cart_t *cart = 0;
    toy_bank_err_t err = toy_bank_parse(buf, len, NULL, &info, &cart);
    if (err != TOY_BANK_OK) return err;

    toy_bank_keyframe_t first, last;
    read_keyframe(buf, &info, 0, &first);
    read_keyframe(buf, &info, info.keyframe_count - 1, &last);

    toy_memset(out, 0, sizeof *out);
    toy_memcpy(out->cart_name, info.cart_name, 16);
    out->steps_at_start = first.step_index;
    out->steps_at_end = last.step_index;
    out->steps_banked = last.step_index - first.step_index;
    out->steps_expected_1x = 0;
    if (info.wall_valid && info.wall_end_s > info.wall_start_s) {
        uint64_t span_s = (uint64_t)(info.wall_end_s - info.wall_start_s);
        out->steps_expected_1x = span_s * (uint64_t)cart->demo_steps_per_frame *
                                 30ull; /* nominal 30 fps demo rate */
    }
    out->hash_start = first.state_hash;
    out->hash_end = last.state_hash;
    out->keyframe_count = info.keyframe_count;
    out->keyframe_every = info.keyframe_every;
    out->wall_start_s = info.wall_start_s;
    out->wall_end_s = info.wall_end_s;
    out->tz_offset_min = info.tz_offset_min;
    out->wall_valid = info.wall_valid;
    out->end_reason = info.end_reason;
    out->playback_verified = playback_verified ? 1u : 0u;
    out->bank_bytes = len;
    if (cart_out) *cart_out = cart;
    return TOY_BANK_OK;
}

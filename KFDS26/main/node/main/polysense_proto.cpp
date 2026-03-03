#include "polysense_proto.h"

// CRC16-CCITT (poly 0x1021, init 0xFFFF, no xorout)
uint16_t ps_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc = (crc << 1);
        }
    }
    return crc;
}

size_t ps_build_frame(uint8_t *dst,
                      size_t dst_size,
                      uint8_t  node_id,
                      uint8_t  msg_type,
                      const uint8_t *payload,
                      uint16_t payload_len)
{
    const size_t total_len =
        2 + sizeof(ps_header_t) + payload_len + 2;

    if (!dst || dst_size < total_len) return 0;

    size_t pos = 0;

    // SYNC
    dst[pos++] = PS_SYNC_LO;
    dst[pos++] = PS_SYNC_HI;

    // Header
    ps_header_t hdr;
    hdr.ver      = PS_PROTO_VER;
    hdr.msg_type = msg_type;
    hdr.node_id  = node_id;
    hdr.len      = payload_len;

    // Copy header into dst
    const uint8_t *h = (const uint8_t *)&hdr;
    for (size_t i = 0; i < sizeof(ps_header_t); ++i) {
        dst[pos++] = h[i];
    }

    // Payload
    for (uint16_t i = 0; i < payload_len; ++i) {
        dst[pos++] = payload ? payload[i] : 0;
    }

    // CRC over header+payload (starting at ver)
    uint16_t crc = ps_crc16_ccitt(dst + 2, sizeof(ps_header_t) + payload_len);
    dst[pos++] = (uint8_t)(crc & 0xFF);
    dst[pos++] = (uint8_t)((crc >> 8) & 0xFF);

    return total_len;
}

// Simple streaming parser for on-node or GS if desired
enum {
    PS_PSTATE_SYNC0 = 0,
    PS_PSTATE_SYNC1,
    PS_PSTATE_HEADER,
    PS_PSTATE_PAYLOAD,
    PS_PSTATE_CRC0,
    PS_PSTATE_CRC1
};

void ps_parser_init(ps_parser_t *p) {
    if (!p) return;
    p->state      = PS_PSTATE_SYNC0;
    p->sync_seen  = 0;
    p->header_pos = 0;
    p->payload_len= 0;
    p->payload_pos= 0;
    p->crc_recv   = 0;
    p->crc_calc   = 0xFFFF;
}

static void ps_crc16_feed(uint16_t *crc, uint8_t b) {
    *crc ^= (uint16_t)b << 8;
    for (int i = 0; i < 8; i++) {
        if (*crc & 0x8000) *crc = (*crc << 1) ^ 0x1021;
        else               *crc = (*crc << 1);
    }
}

int ps_parser_feed(ps_parser_t *p,
                   uint8_t byte,
                   ps_header_t *out_hdr,
                   uint8_t *out_payload,
                   size_t out_payload_size)
{
    if (!p) return 0;

    switch (p->state) {
    case PS_PSTATE_SYNC0:
        if (byte == PS_SYNC_LO) {
            p->state = PS_PSTATE_SYNC1;
        }
        break;

    case PS_PSTATE_SYNC1:
        if (byte == PS_SYNC_HI) {
            p->state      = PS_PSTATE_HEADER;
            p->header_pos = 0;
            p->crc_calc   = 0xFFFF;
        } else {
            // stay in SYNC1 only if this byte is also PS_SYNC_LO,
            // else go back to SYNC0
            p->state = (byte == PS_SYNC_LO) ? PS_PSTATE_SYNC1 : PS_PSTATE_SYNC0;
        }
        break;

    case PS_PSTATE_HEADER:
        if (p->header_pos < sizeof(ps_header_t)) {
            p->header_buf[p->header_pos++] = byte;
            ps_crc16_feed(&p->crc_calc, byte);
            if (p->header_pos == sizeof(ps_header_t)) {
                ps_header_t *hdr = (ps_header_t *)p->header_buf;
                p->payload_len = hdr->len;
                if (!out_payload || out_payload_size < p->payload_len) {
                    // Payload buffer too small or null; reset.
                    p->state = PS_PSTATE_SYNC0;
                    break;
                }
                p->payload_pos = 0;
                p->state = (p->payload_len > 0) ? PS_PSTATE_PAYLOAD : PS_PSTATE_CRC0;
            }
        }
        break;

    case PS_PSTATE_PAYLOAD:
        if (p->payload_pos < p->payload_len) {
            if (out_payload) out_payload[p->payload_pos] = byte;
            p->payload_pos++;
            ps_crc16_feed(&p->crc_calc, byte);
            if (p->payload_pos == p->payload_len) {
                p->state = PS_PSTATE_CRC0;
            }
        }
        break;

    case PS_PSTATE_CRC0:
        p->crc_recv = byte;
        p->state    = PS_PSTATE_CRC1;
        break;

    case PS_PSTATE_CRC1: {
        p->crc_recv |= ((uint16_t)byte << 8);
        uint16_t final_crc = p->crc_calc;
        // full frame received; check CRC
        int ok = (final_crc == p->crc_recv);

        if (ok && out_hdr && out_payload) {
            *out_hdr = *(ps_header_t *)p->header_buf;
        }

        // reset for next frame
        ps_parser_init(p);
        return ok ? 1 : 0;
    }
    }

    return 0;
}

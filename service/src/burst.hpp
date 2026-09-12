// On-air burst protocol ([FB]) — byte-compatible with
// lr2021-flrc-firmware/src/flrc_burst_protocol.{h,c} and the host's
// FlrcBurstAssembly. See docs/protocol.md §3.
#pragma once

#include <cstdint>
#include <cstring>
#include <cstddef>
#include <vector>

namespace burst {

constexpr uint16_t kMagic = 0x4642;              // 'FB' — on-air LE bytes 42 46
constexpr uint32_t kMaxTotalPayload = 12288;
constexpr uint16_t kPacketPayload = 252;         // fixed in v1 (nRF default)
constexpr uint16_t kChunk = kPacketPayload - 16; // 236
constexpr uint16_t kMaxPackets = 53;
constexpr size_t kHeaderLen = 16;

// IEEE 802.3 reflected CRC32 — identical to the firmware and the host
// (init 0xFFFFFFFF, poly 0xEDB88320 shifted right per LSB, final ~).
inline uint32_t crc32(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) {
    return 0;
  }
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}

struct Header {
  uint16_t magic;
  uint16_t burst_id;
  uint16_t packet_idx;
  uint16_t total_packets;
  uint32_t total_payload_len;
  uint32_t payload_crc32;
} __attribute__((packed));

static_assert(sizeof(Header) == kHeaderLen, "header must be 16 bytes");

// Encode one packet: header + payload[chunk] (zero-padded to the full
// packet size), per the nRF firmware's TX layout.
inline void encode_packet(uint8_t *out, size_t out_cap, uint16_t burst_id,
                          uint16_t idx, uint16_t total,
                          const uint8_t *payload, uint32_t total_len,
                          size_t *out_len) {
  Header h{kMagic,   burst_id, idx, total,
           total_len, crc32(payload, total_len)};
  std::memcpy(out, &h, sizeof(h));
  uint32_t off = static_cast<uint32_t>(idx) * kChunk;
  uint32_t take = total_len - off;
  if (take > kChunk) {
    take = kChunk;
  }
  std::memcpy(out + kHeaderLen, payload + off, take);
  // zero-pad to the full on-air packet size
  if (kHeaderLen + take < kPacketPayload) {
    std::memset(out + kHeaderLen + take, 0, kPacketPayload - kHeaderLen - take);
  }
  *out_len = kPacketPayload;
}

// Parse a received raw air packet. Returns false on malformed headers.
inline bool decode_header(const uint8_t *pkt, size_t len, Header *h) {
  if (len < kHeaderLen) {
    return false;
  }
  std::memcpy(h, pkt, sizeof(*h));
  if (h->magic != kMagic) {
    return false;
  }
  if (h->total_packets == 0 || h->total_packets > kMaxPackets) {
    return false;
  }
  if (h->total_payload_len == 0 || h->total_payload_len > kMaxTotalPayload) {
    return false;
  }
  if (h->packet_idx >= h->total_packets) {
    return false;
  }
  return true;
}

// True when the payload's first bytes mark a consensus frame (SLRF) or an
// [FB] burst chunk — the scan "consensus" counter's rule.
inline bool is_consensus_prefix(const uint8_t *p, size_t len) {
  if (len >= 4 && p[0] == 0x53 && p[1] == 0x4C && p[2] == 0x52 && p[3] == 0x46) {
    return true; // "SLRF"
  }
  if (len >= 2 && p[0] == 0x42 && p[1] == 0x46) {
    return true; // 'B','F' — LE bytes of the 0x4642 burst magic
  }
  return false;
}

} // namespace burst

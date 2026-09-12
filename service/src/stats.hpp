// The 68-byte 'S' stats frame body — offsets MUST match the nRF firmware
// (docs/protocol.md §2); the host parses them at fixed offsets.
#pragma once

#include <cstdint>
#include <cstring>

namespace stats {

struct Counters {
  uint32_t bursts_tx;            // 0
  uint32_t bursts_rx;            // 4
  uint32_t packets_tx;           // 8
  uint32_t packets_rx;           // 12
  uint32_t crc_errors;           // 16
  uint32_t host_tx_dropped;      // 20
  uint8_t burst_mode;            // 24 (0=RX, 1=TX_ABORTING, 2=TX)
  uint8_t zero_25;               // 25 (was LBT retries)
  uint8_t pending_tx_valid;      // 26
  uint8_t restart_rx_pending;    // 27
  uint8_t zero_28;               // 28 (was backoff)
  uint8_t pad[3];
  uint32_t uptime_ms;            // 32
  uint32_t uart_ring_drops;      // 36
  uint32_t staging_overwrites;   // 40
  uint32_t burst_id_conflicts;   // 44
  uint32_t hw_received_packets;  // 48
  uint32_t hw_crc_errors;        // 52
  uint32_t hw_length_errors;     // 56
  uint32_t hw_crc_ok;            // 60
  uint32_t parser_frames_killed; // 64
};

static_assert(sizeof(Counters) == 68, "stats body must be exactly 68 bytes");

inline void serialize(const Counters &c, uint8_t out[68]) {
  std::memcpy(out, &c, 68);
}

} // namespace stats

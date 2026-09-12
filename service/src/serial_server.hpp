// Serial server — the host↔device byte-tag protocol state machine
// (docs/protocol.md). Reads the cross-core UART, decodes commands
// (T/C/W/S/Q/X), drives the radio worker, and writes G/R/E/S/D frames
// back. Faithful to the nRF firmware's parser quirks (100 ms idle resync
// with skip_remain, 'T' 2-byte magic, CONFIG len ∈ {15,17}).
#pragma once

#include <cstdint>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>

#include "radio_worker.hpp"

namespace serial {

enum class State : uint8_t {
  kTag,
  kTxMagic,
  kSetWindow,
  kLenLo,
  kLenHi,
  kConfig,
  kPayload,
  kScanCount,
  kScanBody,
};

struct ParsedConfig {
  uint32_t frequency_hz;
  uint32_t bandwidth_hz;
  uint8_t coding_rate;
  int8_t tx_power_dbm;
  uint32_t sync_word;
  uint8_t role;
  uint8_t body_len; // 15 or 17
  uint8_t rate_sel;
  uint8_t payload_sel;
};

class SerialServer {
public:
  SerialServer(radio::RadioWorker *worker, const std::string &uart_path,
               uint32_t baud);

  // Opens the UART, sends READY once the worker reports radio-ok, then
  // runs the forever loop (call from its own thread).
  void run();

  // Worker-callback entries (thread-safe; each emitter takes write_mu_).
  void onWorkerRx(const uint8_t *pkt, size_t len, int16_t rssi);
  void onWorkerTerminal(radio::Terminal t);
  void onWorkerScan(uint32_t freq_hz, uint32_t frames, uint32_t consensus,
                    int8_t best_rssi);
  void onWorkerLog(const std::string &line);

private:
  // ---- UART byte io ----
  int readByte(uint8_t *b, uint32_t timeout_ms); // 0 = byte, -1 = timeout/err
  bool writeAll(const uint8_t *buf, size_t len);
  void writeFrame(char tag, const uint8_t *payload, size_t len);
  void writeReady(uint16_t max_payload);
  void writeError(uint8_t code, const char *msg);
  void writeStats();
  void writeRx(const uint8_t *pkt, size_t len, int16_t rssi);
  void writeDiag(uint8_t event_id, uint32_t val);

  // ---- command handlers ----
  void handleConfig(const uint8_t *body, uint8_t len);
  void handleScanQ(uint8_t count, uint32_t dwell_ms, const uint32_t *freqs);
  void handleReset();
  void handleTx(const uint8_t *payload, size_t len);
  void maybeEmitStats(bool force);
  static uint32_t nowMs();

  radio::RadioWorker *worker_;
  std::string uart_path_;
  uint32_t baud_;
  int fd_ = -1;

  // parser state
  State state_ = State::kTag;
  uint8_t tag_ = 0;
  uint16_t frame_len_ = 0;
  size_t have_ = 0;
  uint16_t skip_remain_ = 0;
  uint32_t last_arrival_ms_ = 0;
  uint32_t parser_frames_killed_ = 0;
  uint8_t config_body_[17];
  uint8_t window_body_[8];
  uint8_t scan_count_ = 0;
  uint8_t scan_have_ = 0;
  uint32_t scan_freqs_[16];
  uint32_t scan_dwell_ = 300;

  uint32_t last_stats_ms_ = 0;
  bool ready_sent_ = false;
  // SET_WINDOW storage (accepted, unused — the host schedules)
  uint32_t slot_offset_ms_ = 100;
  uint32_t slot_period_ms_ = 1000;
  // Guards frame emission across the parser thread and the worker callbacks.
  std::recursive_mutex write_mu_;
};

} // namespace serial

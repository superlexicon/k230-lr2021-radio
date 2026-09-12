// Radio worker — owns the LR2021 and the worker rt_thread. The serial
// server feeds it config/TX requests; it emits RX packets, TX terminals
// and scan results upward. Runs on the dedicated RT-SMART core: the
// worker's only job is keeping the radio in RX and servicing bursts.
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <LR2021.h>
#include "stats.hpp"

namespace radio {

enum class Terminal : uint8_t {
  kOk = 0,
  kBusy = 4,     // E 0x04 — frame not taken
  kFailed = 5,   // E 0x05 — aborted mid-flight, aired-unknown
  kDropped = 6,  // E 0x06 — definitively not sent
};

struct Config {
  uint32_t frequency_hz = 916500000;
  int8_t tx_power_dbm = 20;
  uint8_t role = 0; // 0=BOTH 1=TX_ONLY 2=RX_ONLY
};

struct ScanParams {
  uint32_t start_hz = 916500000;
  uint32_t step_hz = 500000;
  uint8_t count = 6;
  uint32_t dwell_ms = 300;
};

// Upward callbacks (invoked on the worker thread; implementations must be
// quick and hand off to their own queues).
struct Callbacks {
  // One raw air packet received (includes any [FB] header) + RSSI dBm.
  std::function<void(const uint8_t *pkt, size_t len, int16_t rssi)> on_rx_packet;
  // One terminal per accepted TX frame: 'G' (kOk) or 'E' code.
  std::function<void(Terminal terminal)> on_tx_terminal;
  // Scan results per channel boundary (events 50-53).
  std::function<void(uint32_t freq_hz, uint32_t frames, uint32_t consensus,
                     int8_t best_rssi)>
      on_scan_result;
  // Diagnostics line (worker log).
  std::function<void(const std::string &line)> on_log;
};

class RadioWorker {
public:
  // `power_gpio` = module/RF power enable (GPIO44 on the T-Display K230),
  // `cs`=14, `rst`=5 per the LILYGO board.
  bool init(const Callbacks &cb, int power_gpio = 44);

  // Apply a normal CONFIG (returns false + reason if the radio rejects it).
  bool applyConfig(const Config &cfg, std::string *err);

  // Enter scan mode (owns the radio until the sweep completes; TX suppressed).
  void startScan(const ScanParams &p);

  // Enqueue a payload for [FB]-chunked transmission. Returns kBusy when both
  // queue slots are occupied. Exactly one on_tx_terminal follows per accept.
  Terminal enqueueTx(const uint8_t *payload, size_t len);

  // Counters for the 'S' frame.
  void snapshot(stats::Counters *out, uint32_t uptime_ms);

  bool radioOk() const { return radio_ok_; }

  // Thread entry — runs the forever loop (spawn from main via pthread).
  void workerLoop();

private:
  void serviceTxQueue();
  bool transmitBurst(const uint8_t *payload, size_t len, std::string *err);
  void armReceive();
  void scanTick(uint32_t now_ms);
  static uint32_t nowMs();
  static void sleepUs(uint32_t us);

  Callbacks cb_;
  LR2021 *radio_ = nullptr; // RadioLib wrapper (Module-allocated)
  bool radio_ok_ = false;
  Config cfg_{};
  ScanParams scan_{};
  bool scan_active_ = false;
  uint8_t scan_idx_ = 0;
  uint32_t scan_dwell_until_ = 0;
  uint32_t scan_frames_ = 0;
  uint32_t scan_consensus_ = 0;
  int8_t scan_best_rssi_ = -128;

  std::mutex tx_mu_;
  std::queue<std::vector<uint8_t>> tx_q_;
  stats::Counters st_{};
  uint32_t burst_id_counter_ = 1;
};

} // namespace radio

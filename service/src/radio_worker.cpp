// Radio worker implementation — see radio_worker.hpp.
//
// Threading: one worker rt_thread owns the radio exclusively. The serial
// server only enqueues TX frames (mutex-guarded) and receives callbacks.
// IRQ handling is POLLED (this HAL has no GPIO interrupts): the worker
// spins get_clear_IrqStatus() with a ~100 µs yield, which on a dedicated
// 800 MHz core is fast enough to catch consecutive burst packets
// (252 B @ 2.6 Mbps ≈ 0.8 ms airtime; Phase 1 gates this at ≥99%).
#include "radio_worker.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <cstdio>

#include <Module.h>
#include "burst.hpp"
#include "rt_fpioa.h" // pin_gpio_t

// GPIO ioctl contract (mirrors k230Hal.cpp / the canaan gpio driver).
#define GPIO_DM_OUTPUT  _IOW('G', 0, int)
#define GPIO_DM_INPUT   _IOW('G', 1, int)
#define GPIO_WRITE_LOW  _IOW('G', 4, int)
#define GPIO_WRITE_HIGH _IOW('G', 5, int)

// Widen a few protected LR20xx config methods — the LILYGO wrapper doesn't
// expose them, but air-format interop with the nRF nodes needs them.
class K230Radio : public LR2021 {
public:
  K230Radio(Module *m) : LR2021(m) {}
  using LR2021::setFLRCSyncWord;
  using LR2021::setPacketParamsFLRC;
  using LR2021::setModulationParamsFLRC;
  using LR2021::setCRC;
};

using namespace burst;

namespace radio {

static const uint32_t kPowerGpioDefault = 44;

uint32_t RadioWorker::nowMs() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

void RadioWorker::sleepUs(uint32_t us) {
  struct timespec ts;
  ts.tv_sec = us / 1000000;
  ts.tv_nsec = (us % 1000000) * 1000;
  nanosleep(&ts, nullptr);
}

// Module/RF power enable (GPIO44 high on the T-Display K230).
static void power_enable(int gpio) {
  int fd = open("/dev/gpio", O_RDWR);
  if (fd < 0) {
    return;
  }
  pin_gpio_t p;
  p.pin = gpio;
  ioctl(fd, GPIO_DM_OUTPUT, &p);
  ioctl(fd, GPIO_WRITE_HIGH, &p);
  close(fd);
}

bool RadioWorker::init(const Callbacks &cb, int power_gpio) {
  cb_ = cb;
  power_enable(power_gpio);
  sleepUs(50000);

  // Module(cs=14, irq=NC, rst=5, gpio=NC) — the patched Module ctor
  // auto-instantiates k230Hal (fpioa mux + /dev/spi1 + reset pulse).
  K230Radio *k = new K230Radio(new Module(14, RADIOLIB_NC, 5, RADIOLIB_NC));
  radio_ = k;

  // FLRC 2.6 Mbps (BR_2_600_BW_2_6 = 0x00), CR 1/1 — the air profile the
  // nRF nodes use (their CONFIG coding_rate is stored but never applied).
  // tcxo 1.8 V per the EVK (matches the nRF overlay's tcxo-voltage).
  int16_t st = k->beginFLRC(916.5f, RADIOLIB_LR20xx_FLRC_BR_2_600_BW_2_6,
                            1, 20, 16, 1.8f, 0);
  if (st != RADIOLIB_ERR_NONE) {
    if (cb_.on_log) {
      cb_.on_log("radio init failed: " + std::to_string(st));
    }
    return false;
  }

  // Air sync word: the nRF nodes' fixed bytes 90 56 34 12 (host config
  // value 0x12345690, LE on air). Byte order on the wire is a Phase-1
  // bring-up check against a captured nRF frame.
  k->setFLRCSyncWord(0, 0x12345690u);

  radio_ok_ = true;
  st_.burst_mode = 0;
  armReceive();
  return true;
}

void RadioWorker::armReceive() {
  if (!radio_ok_ || cfg_.role == 1) {
    return; // TX-only
  }
  radio_->startReceive();
  st_.burst_mode = 0;
  st_.restart_rx_pending = 0;
}

bool RadioWorker::applyConfig(const Config &cfg, std::string *err) {
  if (!radio_ok_) {
    *err = "radio init failed";
    return false;
  }
  cfg_ = cfg;
  int16_t st = radio_->setFrequency(static_cast<float>(cfg.frequency_hz) / 1e6f);
  if (st != RADIOLIB_ERR_NONE) {
    *err = "set frequency failed";
    return false;
  }
  radio_->setOutputPower(cfg.tx_power_dbm);
  // Bandwidth/coding-rate/sync-word CONFIG fields are stored-but-fixed on
  // the nRF too (air profile is compiled in) — same here.
  if (!scan_active_) {
    armReceive();
  }
  return true;
}

void RadioWorker::startScan(const ScanParams &p) {
  scan_ = p;
  scan_idx_ = 0;
  scan_frames_ = 0;
  scan_consensus_ = 0;
  scan_best_rssi_ = -128;
  scan_dwell_until_ = nowMs() + p.dwell_ms;
  scan_active_ = true;
  st_.pending_tx_valid = 0;
  // RX keeps running on the current frequency for the first dwell (the nRF
  // quirk: channel 0 reports `start` Hz while listening on the home freq).
}

void RadioWorker::scanTick(uint32_t now_ms) {
  if (static_cast<int32_t>(now_ms - scan_dwell_until_) < 0) {
    // dwell: any RX_DONE is consumed by the normal loop; tally there.
    return;
  }
  if (cb_.on_scan_result) {
    cb_.on_scan_result(scan_.start_hz + static_cast<uint32_t>(scan_idx_) * scan_.step_hz,
                       scan_frames_, scan_consensus_, scan_best_rssi_);
  }
  scan_idx_++;
  if (scan_idx_ >= scan_.count) {
    scan_active_ = false;
    armReceive(); // stays on the last swept channel; the host retunes
    if (cb_.on_log) {
      cb_.on_log("scan complete: " + std::to_string(scan_.count) + " channels");
    }
    return;
  }
  uint32_t f = scan_.start_hz + static_cast<uint32_t>(scan_idx_) * scan_.step_hz;
  radio_->setFrequency(static_cast<float>(f) / 1e6f);
  scan_frames_ = 0;
  scan_consensus_ = 0;
  scan_best_rssi_ = -128;
  scan_dwell_until_ = now_ms + scan_.dwell_ms;
  armReceive();
}

Terminal RadioWorker::enqueueTx(const uint8_t *payload, size_t len) {
  if (!radio_ok_) {
    return Terminal::kBusy;
  }
  std::lock_guard<std::mutex> lk(tx_mu_);
  if (tx_q_.size() >= 2) {
    st_.host_tx_dropped++;
    return Terminal::kBusy;
  }
  tx_q_.emplace(payload, payload + len);
  st_.pending_tx_valid = 1;
  return Terminal::kOk;
}

bool RadioWorker::transmitBurst(const uint8_t *payload, size_t len,
                                std::string *err) {
  uint16_t total = static_cast<uint16_t>((len + kChunk - 1) / kChunk);
  if (total == 0 || total > kMaxPackets || len > kMaxTotalPayload) {
    *err = "bad burst size";
    return false;
  }
  // Per-boot offset so on-air burst ids don't collide across nodes.
  uint32_t boot = nowMs() | 1u;
  uint16_t burst_id = static_cast<uint16_t>((burst_id_counter_++) ^
                                            (boot ^ (boot >> 16)));
  if (burst_id == 0) {
    burst_id = 1;
  }
  st_.bursts_tx++;

  std::vector<uint8_t> pkt(kPacketPayload);
  for (uint16_t idx = 0; idx < total; idx++) {
    size_t pkt_len = 0;
    encode_packet(pkt.data(), pkt.size(), burst_id, idx, total, payload, len,
                  &pkt_len);
    radio_->startTransmit(pkt.data(), pkt_len);
    st_.packets_tx++;
    // Poll for TX_DONE (1<<19); 2 s timeout, 100 µs poll cadence.
    uint32_t deadline = nowMs() + 2000;
    bool done = false;
    while (static_cast<int32_t>(nowMs() - deadline) < 0) {
      uint32_t irq = radio_->get_clear_IrqStatus();
      if (irq & RADIOLIB_LR20xx_IRQ_TX_DONE) {
        done = true;
        break;
      }
      if (irq & RADIOLIB_LR20xx_IRQ_TIMEOUT) {
        break;
      }
      sleepUs(100);
    }
    radio_->finishTransmit();
    if (!done) {
      *err = "tx timeout";
      return false;
    }
    // min inter-frame gap on air (nRF: 2000 µs)
    sleepUs(2000);
  }
  return true;
}

void RadioWorker::serviceTxQueue() {
  if (cfg_.role == 2 || scan_active_) {
    return; // RX-only or scan owns the radio
  }
  std::vector<uint8_t> frame;
  {
    std::lock_guard<std::mutex> lk(tx_mu_);
    if (tx_q_.empty()) {
      return;
    }
    frame = std::move(tx_q_.front());
    tx_q_.pop();
    st_.pending_tx_valid = tx_q_.size() > 0 ? 1 : 0;
  }
  // Half-duplex turnaround: stand by, TX the burst, re-arm RX.
  radio_->standby();
  st_.burst_mode = 2;
  std::string err;
  bool ok = transmitBurst(frame.data(), frame.size(), &err);
  st_.burst_mode = 0;
  if (ok) {
    if (cb_.on_tx_terminal) {
      cb_.on_tx_terminal(Terminal::kOk);
    }
  } else {
    if (cb_.on_log) {
      cb_.on_log("tx failed: " + err);
    }
    if (cb_.on_tx_terminal) {
      cb_.on_tx_terminal(Terminal::kDropped);
    }
  }
  armReceive();
}

void RadioWorker::workerLoop() {
  uint8_t pkt[kPacketPayload];
  for (;;) {
    if (scan_active_) {
      scanTick(nowMs());
      sleepUs(200);
      continue;
    }
    serviceTxQueue();
    if (!radio_ok_ || cfg_.role == 1) {
      sleepUs(1000);
      continue;
    }
    uint32_t irq = radio_->get_clear_IrqStatus();
    if (irq & RADIOLIB_LR20xx_IRQ_RX_DONE) {
      int16_t rssi = radio_->getRSSI();
      size_t len = kPacketPayload;
      if (radio_->readData(pkt, len) == RADIOLIB_ERR_NONE) {
        st_.packets_rx++;
        st_.bursts_rx++;
        if (rssi > scan_best_rssi_ && scan_active_) {
          scan_best_rssi_ = rssi;
        }
        if (scan_active_ && is_consensus_prefix(pkt, len)) {
          scan_consensus_++;
        }
        if (scan_active_) {
          scan_frames_++;
        }
        if (cb_.on_rx_packet) {
          cb_.on_rx_packet(pkt, len, rssi);
        }
      }
      armReceive(); // immediate re-arm — the burst-turnaround race is the
                    // Phase-1 gate; turnaround is bounded by this loop.
    } else if (irq & RADIOLIB_LR20xx_IRQ_TIMEOUT) {
      armReceive();
    }
    sleepUs(100);
  }
}

void RadioWorker::snapshot(stats::Counters *out, uint32_t uptime_ms) {
  std::lock_guard<std::mutex> lk(tx_mu_);
  st_.uptime_ms = uptime_ms;
  *out = st_;
}

} // namespace radio

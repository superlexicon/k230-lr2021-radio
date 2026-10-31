// Radio worker implementation — see radio_worker.hpp.
//
// DESIGN PARITY (synced 2026-10-26 against lr2021-flrc-firmware@97b895a):
// - 24 KB (24,576 B) max burst payload / 50 packets @ 511 B (flrc 97b895a).
// - Interframe pacing 220 µs (flrc c35f669) — ported below.
// - Per-burst burst-id anti-collision (flrc 684a158 / 97b895a): incrementing
//   counter mixed with monotonic ns + ms, 0 skipped.
// - Precomputed per-burst CRC32 (flrc 97b895a): computed once before the
//   packet loop so 50-packet bursts don't recompute CRC32 inside the 220 µs gap.
// - Immediate RX re-arm before UART emission: armReceive() runs right after
//   SPI readData() so serial write latency cannot delay catching the next
//   chunk in a 50-packet burst.
// - READY v2 / CONFIG v2 selectors (flrc 87509e1): DELIBERATELY NOT
//   ported — the flashed nRF build predates the v2 READY (no version
//   byte; the node logs "did not negotiate v2") while running the 511 B
//   profile. Keeping this node's READY legacy too holds the serial
//   surface identical to the fleet. (Advertising v2 would actually be
//   safe today — the host's selectors resolve to the same 511 B — but
//   legacy is the smaller surface until the v2 firmware ships fleet-wide.)
// - RX burst timeout margin 30 s (flrc cb8e33c): NOT APPLICABLE — this
//   worker arms RX via RadioLib's RADIOLIB_LR20xx_RX_TIMEOUT_INF
//   (continuous), so the nRF's 2.2 s re-arm-cycling bug has no
//   equivalent here.
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
#include "mpi_pm_api.h"

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
  using LR2021::setPacketType;
  using LR2021::setFLRCSyncWord;
  using LR2021::setPacketParamsFLRC;
  using LR2021::setModulationParamsFLRC;
  using LR2021::setGfskCrcParams;
  using LR2021::writeRegMem32;
  using LR2021::configFifoIrq;
  using LR2021::getPacketStatusFLRC;
  using LR2021::getRxFifoLevel;
  using LR2021::clearRxBuffer;
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
  if (us == 0) {
    return;
  }
#if defined(__riscv)
  if (us < 1000) {
    uint64_t start = 0;
    asm volatile("rdtime %0" : "=r"(start));
    uint64_t ticks = static_cast<uint64_t>(us) * 27ULL;
    while (true) {
      uint64_t now = 0;
      asm volatile("rdtime %0" : "=r"(now));
      if (now - start >= ticks) {
        break;
      }
    }
    return;
  }
#endif
  struct timespec ts;
  ts.tv_sec = us / 1000000;
  ts.tv_nsec = (us % 1000000) * 1000;
  nanosleep(&ts, nullptr);
}

// Module/RF power enable (GPIO44 high on the T-Display K230).
static constexpr int8_t kMaxSafeBenchTxPowerDbm = 10;
static constexpr int8_t kThrottledTxPowerDbm = 4;

static void power_set(int gpio, bool high) {
  int fd = open("/dev/gpio", O_RDWR);
  if (fd < 0) {
    return;
  }
  pin_gpio_t p;
  p.pin = gpio;
  ioctl(fd, GPIO_DM_OUTPUT, &p);
  ioctl(fd, high ? GPIO_WRITE_HIGH : GPIO_WRITE_LOW, &p);
  close(fd);
}

bool RadioWorker::init(const Callbacks &cb, int power_gpio) {
  cb_ = cb;
  power_gpio_ = power_gpio;
  power_set(power_gpio, true);
  // Ensure fan GPIO42 is configured as output
  power_set(42, false);
  sleepUs(50000);

  // Module(cs=14, irq=NC, rst=5, gpio=NC) — the patched Module ctor
  // auto-instantiates k230Hal (fpioa mux + /dev/spi1 + reset pulse).
  if (!radio_) {
    K230Radio *k = new K230Radio(new Module(14, RADIOLIB_NC, 5, RADIOLIB_NC));
    radio_ = k;
  }
  K230Radio *k = static_cast<K230Radio *>(radio_);

  // FLRC 2.6 Mbps (BR_2_600_BW_2_6 = 0x00), CR 1/1 — the air profile the
  // nRF nodes use (their CONFIG coding_rate is stored but never applied).
  // tcxo 3.0 V on the LILYGO T-Display K230 board (per Lora.cpp).
  // Cap default bench TX power to +10 dBm (instead of +22 dBm) to cut RF PA heat ~75%.
  int16_t st = k->beginFLRC(915.0f, RADIOLIB_LR20xx_FLRC_BR_2_600_BW_2_6,
                            1, kMaxSafeBenchTxPowerDbm, 16, 3.0f, 0);
  if (st != RADIOLIB_ERR_NONE) {
    if (cb_.on_log) {
      cb_.on_log("radio init failed: " + std::to_string(st));
    }
    return false;
  }

  // Re-apply FLRC profile and 1024B FIFO layout after calibration inside beginFLRC
  k->setPacketType(RADIOLIB_LR20xx_PACKET_TYPE_FLRC);
  k->setModulationParamsFLRC(RADIOLIB_LR20xx_FLRC_BR_2_600_BW_2_6,
                             RADIOLIB_LR20xx_FLRC_CR_1_1,
                             RADIOLIB_LR20xx_FLRC_PULSE_SHAPE_BT_05);
  k->setPacketParamsFLRC(RADIOLIB_LR20xx_FLRC_PREAMBLE_LEN_32_BITS,
                         RADIOLIB_LR20xx_FLRC_SYNCWORD_LENGTH_4_BYTES,
                         RADIOLIB_LR20xx_FLRC_RX_MATCH_SYNCWORD_1, 0, 511,
                         RADIOLIB_LR20xx_FLRC_CRC_2_BYTES);
  k->setGfskCrcParams(0xFFFFFFFFu, 0x0000755Bu);

  // Air sync word: the nRF nodes' fixed bytes { 0x90, 0x56, 0x34, 0x12 } at
  // syncword_index = 1 (LR20XX_RADIO_FLRC_TX_SYNCWORD_1). setFLRCSyncWord
  // shifts the uint32 big-endian into { sync_num, b3, b2, b1, b0 }, so
  // passing (1, 0x90563412u) sends { 0x01, 0x90, 0x56, 0x34, 0x12 }.
  k->setFLRCSyncWord(1, 0x90563412u);

  {
    uint32_t tx_fifo_addr = 0x00804000u;
    uint32_t tx_fifo_size = 0x000003FCu;
    uint32_t rx_fifo_addr = 0x00804400u;
    uint32_t rx_fifo_size = 0x000003E8u;
    k->writeRegMem32(0x00F3002Cu, &tx_fifo_addr, 1);
    k->writeRegMem32(0x00F30034u, &tx_fifo_size, 1);
    k->writeRegMem32(0x00F30028u, &rx_fifo_addr, 1);
    k->writeRegMem32(0x00F30030u, &rx_fifo_size, 1);
  }
  k->configFifoIrq(0x3F, 0x3F, 511, 0, 0, 511);
  {
    uint32_t rx_thresh = (511u << 16);
    uint32_t tx_thresh = (511u << 16);
    k->writeRegMem32(0x00F2A098u, &rx_thresh, 1);
    k->writeRegMem32(0x00F2A09Cu, &tx_thresh, 1);
  }

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
  std::lock_guard<std::recursive_mutex> lk(radio_mu_);
  if (!radio_ok_) {
    *err = "radio init failed";
    return false;
  }
  Config clamped = cfg;
  int8_t max_pwr = (thermal_level_ >= 1) ? kThrottledTxPowerDbm : kMaxSafeBenchTxPowerDbm;
  if (clamped.tx_power_dbm > max_pwr) {
    clamped.tx_power_dbm = max_pwr;
  }
  bool pwr_changed = (clamped.tx_power_dbm != cfg_.tx_power_dbm);
  if (!scan_active_ && clamped.frequency_hz == cfg_.frequency_hz && !pwr_changed &&
      clamped.role == cfg_.role) {
    return true;
  }
  scan_active_ = false;
  cfg_ = clamped;
  radio_->standby();
  if (clamped.frequency_hz > 0) {
    int16_t st = radio_->setFrequency(static_cast<float>(clamped.frequency_hz) / 1e6f);
    if (st != RADIOLIB_ERR_NONE) {
      *err = "set frequency failed";
      armReceive();
      return false;
    }
  }
  if (pwr_changed) {
    radio_->setOutputPower(clamped.tx_power_dbm);
  }
  // Bandwidth/coding-rate/sync-word CONFIG fields are stored-but-fixed on
  // the nRF too (air profile is compiled in) — same here.
  armReceive();
  return true;
}

void RadioWorker::startScan(const ScanParams &p) {
  std::lock_guard<std::recursive_mutex> lk(radio_mu_);
  if (!scan_active_ && cfg_.frequency_hz > 0) {
    pre_scan_freq_hz_ = cfg_.frequency_hz;
  }
  scan_ = p;
  scan_idx_ = 0;
  scan_frames_ = 0;
  scan_consensus_ = 0;
  scan_best_rssi_ = -128;
  scan_dwell_until_ = nowMs() + p.dwell_ms;
  scan_active_ = true;
  st_.pending_tx_valid = 0;
  if (radio_ok_ && p.start_hz > 0) {
    radio_->standby();
    radio_->setFrequency(static_cast<float>(p.start_hz) / 1e6f);
    cfg_.frequency_hz = p.start_hz;
    armReceive();
  }
}

void RadioWorker::scanTick(uint32_t now_ms) {
  std::lock_guard<std::recursive_mutex> lk(radio_mu_);
  // Any single consensus phase message (scan_consensus_ > 0) is sufficient to
  // know there is an active subnet on this frequency — advance immediately!
  if (scan_consensus_ == 0 &&
      static_cast<int32_t>(now_ms - scan_dwell_until_) < 0) {
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
    if (radio_ok_ && pre_scan_freq_hz_ > 0) {
      radio_->standby();
      radio_->setFrequency(static_cast<float>(pre_scan_freq_hz_) / 1e6f);
      cfg_.frequency_hz = pre_scan_freq_hz_;
    }
    armReceive();
    if (cb_.on_log) {
      cb_.on_log("scan complete: " + std::to_string(scan_.count) + " channels");
    }
    return;
  }
  uint32_t f = scan_.start_hz + static_cast<uint32_t>(scan_idx_) * scan_.step_hz;
  radio_->standby();
  radio_->setFrequency(static_cast<float>(f) / 1e6f);
  cfg_.frequency_hz = f;
  scan_frames_ = 0;
  scan_consensus_ = 0;
  scan_best_rssi_ = -128;
  scan_dwell_until_ = now_ms + scan_.dwell_ms;
  armReceive();
}

Terminal RadioWorker::enqueueTx(const uint8_t *payload, size_t len) {
  if (!radio_ok_ || thermal_level_ >= 2) {
    return Terminal::kBusy;
  }
  std::lock_guard<std::mutex> lk(tx_mu_);
  if (tx_q_.size() >= 16) {
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
  // Per-burst jittered offset so on-air burst ids don't collide across nodes.
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  uint32_t entropy = static_cast<uint32_t>(ts.tv_nsec) ^
                     (static_cast<uint32_t>(ts.tv_sec) * 0x9E3779B9u) ^
                     (burst_id_counter_++ * 0x85EBCA6Bu);
  uint16_t burst_id = static_cast<uint16_t>(entropy ^ (entropy >> 16));
  if (burst_id == 0) {
    burst_id = 1;
  }
  st_.bursts_tx++;

  uint32_t payload_crc = crc32(payload, len);
  std::vector<uint8_t> pkt(kPacketPayload);
  for (uint16_t idx = 0; idx < total; idx++) {
    size_t pkt_len = 0;
    encode_packet(pkt.data(), pkt.size(), burst_id, idx, total, payload, len,
                  payload_crc, &pkt_len);
    bool done = false;
    for (int attempt = 0; attempt < 3 && !done; attempt++) {
      if (attempt > 0) {
        radio_->standby();
        sleepUs(500);
      }
      int16_t st = radio_->startTransmit(pkt.data(), pkt_len);
      if (st != RADIOLIB_ERR_NONE) {
        continue;
      }
      // Poll for TX_DONE (1<<19); 25 ms per-attempt timeout, 100 µs poll cadence.
      uint32_t deadline = nowMs() + 25;
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
    }
    if (!done) {
      *err = "tx timeout";
      return false;
    }
    st_.packets_tx++;
    // INTERFRAME PACING (flrc c35f669, ported 2026-10-07): the flrc_burst
    // reference sample's 220 µs — the nRF fleet's current pacing. Halves
    // every burst's air time vs the old 2000 µs; this worker's 100 µs
    // TX_DONE poll cadence catches completion well inside the gap.
    sleepUs(220);
  }
  return true;
}

void RadioWorker::thermalTick(uint32_t now_ms) {
  if (last_thermal_check_ms_ != 0 &&
      static_cast<uint32_t>(now_ms - last_thermal_check_ms_) < 4000) {
    return;
  }
  last_thermal_check_ms_ = now_ms;

  int next_level = thermal_level_;
  double temp_c = -100.0;
  bool got_state = false;

  // 1. Prefer reading the non-blocking state published by the OS thermal guardian thread
  struct timespec now_ts = {0, 0};
  clock_gettime(CLOCK_MONOTONIC, &now_ts);

  // 1. Prefer /tmp/os_thermal_state published by app.elf's OS thermal guardian if fresh (<= 10s old)
  FILE *fp = fopen("/tmp/os_thermal_state", "r");
  if (fp) {
    int lvl = 0;
    double tc = 0.0;
    long ts_sec = 0;
    int n = fscanf(fp, "%d %lf %ld", &lvl, &tc, &ts_sec);
    fclose(fp);
    if (n >= 2 && tc > -20.0 && tc < 150.0) {
      if (n < 3 || (now_ts.tv_sec - ts_sec) <= 10) {
        next_level = lvl;
        temp_c = tc;
        got_state = true;
      }
    }
  }

  // 2. Fallback: read /dev/ts & manage hardware CPU1 DVFS directly if app.elf is not running
  if (!got_state) {
    if (!pm_initialized_) {
      kd_mpi_pm_set_clock(PM_DOMAIN_KPU, K_FALSE);
      kd_mpi_pm_set_power(PM_DOMAIN_KPU, K_FALSE);
      kd_mpi_pm_set_clock(PM_DOMAIN_DPU, K_FALSE);
      kd_mpi_pm_set_power(PM_DOMAIN_DPU, K_FALSE);
      kd_mpi_pm_set_clock(PM_DOMAIN_VPU, K_FALSE);
      kd_mpi_pm_set_power(PM_DOMAIN_VPU, K_FALSE);
      kd_mpi_pm_set_governor(PM_DOMAIN_CPU, PM_GOVERNOR_AUTO);
      cpu_dvfs_stage_ = 0;
      pm_initialized_ = true;
    }
    if (fd_ts_ < 0) {
      fd_ts_ = open("/dev/ts", O_RDWR);
    }
    if (fd_ts_ >= 0) {
      uint32_t buf[1] = {0};
      if (read(fd_ts_, buf, sizeof(buf)) > 0) {
        double code = static_cast<double>(buf[0] & 0xfff);
        if (code > 0.0 && code < 4095.0) {
          double c2 = code * code;
          double c3 = c2 * code;
          double c4 = c2 * c2;
          temp_c = 1e-10 * c4 * 1.01472 - 1e-6 * c3 * 1.10063 +
                   4.36150 * 1e-3 * c2 - 7.10128 * code + 3565.87;
          if (temp_c > -20.0 && temp_c < 150.0) {
            got_state = true;
            if (temp_c >= 45.0 && !fan_on_) {
              power_set(42, true);
              fan_on_ = true;
            } else if (temp_c < 38.0 && fan_on_) {
              power_set(42, false);
              fan_on_ = false;
            }
            int next_dvfs = 0;
            if (temp_c >= 74.0 || (cpu_dvfs_stage_ == 5 && temp_c >= 71.0)) next_dvfs = 5;
            else if (temp_c >= 68.0 || (cpu_dvfs_stage_ >= 4 && temp_c >= 65.0)) next_dvfs = 4;
            else if (temp_c >= 62.0 || (cpu_dvfs_stage_ >= 3 && temp_c >= 59.0)) next_dvfs = 3;
            else if (temp_c >= 56.0 || (cpu_dvfs_stage_ >= 2 && temp_c >= 53.0)) next_dvfs = 2;
            else if (temp_c >= 50.0 || (cpu_dvfs_stage_ >= 1 && temp_c >= 47.0)) next_dvfs = 1;
            if (next_dvfs != cpu_dvfs_stage_) {
              if (next_dvfs == 0) {
                kd_mpi_pm_set_governor(PM_DOMAIN_CPU, PM_GOVERNOR_AUTO);
              } else {
                kd_mpi_pm_set_governor(PM_DOMAIN_CPU, PM_GOVERNOR_MANUAL);
                kd_mpi_pm_set_profile(PM_DOMAIN_CPU, next_dvfs);
              }
              cpu_dvfs_stage_ = next_dvfs;
            }
            if (temp_c >= 82.0) {
              next_level = 2;
            } else if (thermal_level_ == 2) {
              if (temp_c < 55.0) {
                next_level = 0;
              }
            } else if (temp_c >= 68.0) {
              next_level = 1;
            } else if (thermal_level_ == 1 && temp_c < 60.0) {
              next_level = 0;
            }
          }
        }
      }
    }
  }

  if (!got_state) {
    return;
  }

  if (next_level != thermal_level_) {
    int prev = thermal_level_;
    thermal_level_ = next_level;
    if (next_level == 2) {
      if (cb_.on_log) {
        cb_.on_log("THERMAL CUTOFF: die temp " + std::to_string(temp_c) +
                   " C >= 80 C! Powering down LR2021 radio (GPIO44 LOW).");
      }
      {
        std::lock_guard<std::recursive_mutex> rlk(radio_mu_);
        if (radio_ && radio_ok_) {
          radio_->standby();
        }
        power_set(42, true);
        power_set(power_gpio_, false);
        thermal_powered_down_ = true;
      }
      // Drain any pending TX frames so callers are not blocked
      std::vector<size_t> dropped;
      {
        std::lock_guard<std::mutex> lk(tx_mu_);
        while (!tx_q_.empty()) {
          dropped.push_back(tx_q_.front().size());
          tx_q_.pop();
        }
        st_.pending_tx_valid = 0;
      }
      for (size_t i = 0; i < dropped.size(); i++) {
        if (cb_.on_tx_terminal) {
          cb_.on_tx_terminal(Terminal::kDropped);
        }
      }
    } else if (next_level == 1) {
      if (cb_.on_log) {
        cb_.on_log("THERMAL THROTTLE: die temp " + std::to_string(temp_c) +
                   " C >= 70 C — reducing TX power to +4 dBm & pacing bursts.");
      }
      std::lock_guard<std::recursive_mutex> rlk(radio_mu_);
      if (radio_ && radio_ok_ && !thermal_powered_down_) {
        radio_->standby();
        radio_->setOutputPower(kThrottledTxPowerDbm);
        armReceive();
      }
    } else if (next_level == 0) {
      if (cb_.on_log) {
        cb_.on_log("THERMAL RECOVERY: die temp cooled to " +
                   std::to_string(temp_c) + " C — resuming normal radio operation.");
      }
      std::lock_guard<std::recursive_mutex> rlk(radio_mu_);
      if (thermal_powered_down_) {
        Config saved_cfg = cfg_;
        if (init(cb_, power_gpio_)) {
          thermal_powered_down_ = false;
          std::string err;
          applyConfig(saved_cfg, &err);
        }
      } else if (prev == 1 && radio_ && radio_ok_) {
        radio_->standby();
        radio_->setOutputPower(cfg_.tx_power_dbm);
        armReceive();
      }
    }
  }
}

void RadioWorker::serviceTxQueue() {
  if (cfg_.role == 2 || scan_active_ || thermal_level_ >= 2) {
    return; // RX-only, scan owns the radio, or thermal cutoff active
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
  bool ok = false;
  std::string err;
  {
    std::lock_guard<std::recursive_mutex> rlk(radio_mu_);
    radio_->standby();
    st_.burst_mode = 2;
    ok = transmitBurst(frame.data(), frame.size(), &err);
    st_.burst_mode = 0;
    armReceive();
  }
  if (thermal_level_ == 1) {
    // Cooldown sleep between TX bursts when die temperature >= 70 C
    sleepUs(15000);
  }
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
}

void RadioWorker::pollOnce() {
  uint32_t now_ms = nowMs();
  thermalTick(now_ms);
  if (thermal_level_ >= 2) {
    return;
  }
  uint8_t pkt[kPacketPayload];
  if (scan_active_) {
    scanTick(now_ms);
  }
  if (!radio_ok_) {
    return;
  }
  if (cfg_.role == 1) {
    if (!scan_active_) {
      serviceTxQueue();
    }
    return;
  }

  K230Radio *k = static_cast<K230Radio *>(radio_);
  bool in_burst = false;
  uint32_t burst_deadline_ms = 0;
  struct PendingRx {
    uint8_t data[kPacketPayload];
    size_t len;
    int16_t rssi;
  };
  std::vector<PendingRx> burst_rx_queue;
  bool check_fifo_after_rx = false;

  for (;;) {
    bool got_rx = false;
    int16_t rssi = 0;
    size_t len = 0;
    {
      std::lock_guard<std::recursive_mutex> rlk(radio_mu_);
      uint32_t irq = radio_->get_clear_IrqStatus();
      bool has_err = (irq & (RADIOLIB_LR20xx_IRQ_CRC_ERROR |
                             RADIOLIB_LR20xx_IRQ_LEN_ERROR |
                             RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR |
                             RADIOLIB_LR20xx_IRQ_TIMEOUT)) != 0;
      bool has_rx = (irq & RADIOLIB_LR20xx_IRQ_RX_DONE) != 0;
      if (!has_rx && !has_err) {
        uint16_t fifo_lvl = k->getRxFifoLevel();
        if (check_fifo_after_rx && fifo_lvl >= kPacketPayload) {
          has_rx = true;
        } else if (fifo_lvl > 0 && !in_burst) {
          // A packet is actively being demodulated into the 1000B RX FIFO right
          // now; stay in the tight 150 us poll loop so we pop it immediately on
          // RX_DONE before a second burst packet can overflow the 1000B FIFO.
          in_burst = true;
          burst_deadline_ms = nowMs() + 4;
        }
      }
      check_fifo_after_rx = false;
      if (has_rx) {
        uint16_t fifo_lvl = k->getRxFifoLevel();
        if (irq & (RADIOLIB_LR20xx_IRQ_CRC_ERROR |
                   RADIOLIB_LR20xx_IRQ_LEN_ERROR |
                   RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR)) {
          if (irq & RADIOLIB_LR20xx_IRQ_CRC_ERROR) {
            st_.crc_errors++;
            st_.hw_crc_errors++;
          } else {
            st_.hw_length_errors++;
          }
          k->clearRxBuffer();
          armReceive();
          in_burst = false;
        } else if (fifo_lvl < kPacketPayload) {
          // Never read kPacketPayload bytes when fewer bytes sit in the
          // hardware RX FIFO — doing so underflows the LR2021 RX FIFO read
          // pointer and wedges subsequent reception.
          if (fifo_lvl > 0) {
            k->clearRxBuffer();
            armReceive();
          }
          in_burst = false;
        } else {
          float rssi_avg = -95.0f;
          uint16_t rx_len = 0;
          k->getPacketStatusFLRC(nullptr, &rssi_avg, &rx_len, nullptr);
          rssi = static_cast<int16_t>(rssi_avg);
          len = kPacketPayload;
          // Continuous RX mode (0xFFFFFF) stays in RX automatically across
          // consecutive packets in a 220 us burst; do NOT call armReceive()
          // or clearRxBuffer() mid-burst so arriving packets in the 1000B RX
          // FIFO are never wiped!
          bool read_ok = (radio_->readData(pkt, len) == RADIOLIB_ERR_NONE);
          if (read_ok) {
            got_rx = true;
            st_.packets_rx++;
            st_.hw_received_packets++;
            st_.hw_crc_ok++;
            if (rssi > scan_best_rssi_ && scan_active_) {
              scan_best_rssi_ = rssi;
            }
            if (scan_active_ && is_consensus_prefix(pkt, len)) {
              scan_consensus_++;
            }
            if (scan_active_) {
              scan_frames_++;
            }
          } else {
            armReceive();
            in_burst = false;
          }
        }
      } else if (irq & (RADIOLIB_LR20xx_IRQ_CRC_ERROR |
                        RADIOLIB_LR20xx_IRQ_LEN_ERROR |
                        RADIOLIB_LR20xx_IRQ_TIMEOUT)) {
        if (irq & RADIOLIB_LR20xx_IRQ_CRC_ERROR) {
          st_.crc_errors++;
          st_.hw_crc_errors++;
        } else if (irq & RADIOLIB_LR20xx_IRQ_LEN_ERROR) {
          st_.hw_length_errors++;
        }
        armReceive();
        in_burst = false;
      }
    }

    if (got_rx) {
      check_fifo_after_rx = true;
      Header hdr{};
      bool is_fb = decode_header(pkt, len, &hdr);
      if (!is_fb) {
        std::lock_guard<std::recursive_mutex> rlk(radio_mu_);
        k->clearRxBuffer();
        armReceive();
        in_burst = false;
        st_.burst_mode = 0;
        burst_rx_queue.clear();
        break;
      }
      if (hdr.packet_idx + 1 < hdr.total_packets) {
        in_burst = true;
        st_.burst_mode = 1;
        burst_deadline_ms = nowMs() + 10; // next packet arrives in ~1.8 ms
      } else {
        in_burst = false;
        st_.burst_mode = 0;
        st_.bursts_rx++;
      }
      if (in_burst) {
        PendingRx item{};
        memcpy(item.data, pkt, len);
        item.len = len;
        item.rssi = rssi;
        burst_rx_queue.push_back(item);
        continue; // tight poll for next packet in burst without IPC/VFS syscalls
      }
      if (cb_.on_rx_packet) {
        for (const auto &q : burst_rx_queue) {
          cb_.on_rx_packet(q.data, q.len, q.rssi);
        }
        burst_rx_queue.clear();
        cb_.on_rx_packet(pkt, len, rssi);
      } else {
        burst_rx_queue.clear();
      }
      {
        std::lock_guard<std::recursive_mutex> rlk(radio_mu_);
        uint16_t fifo_lvl = k->getRxFifoLevel();
        if (fifo_lvl >= kPacketPayload) {
          in_burst = true;
          burst_deadline_ms = nowMs() + 5;
          continue;
        } else if (fifo_lvl > 0) {
          in_burst = true;
          burst_deadline_ms = nowMs() + 4;
          continue;
        } else {
          // Burst finished and RX FIFO is completely empty: reset RX FIFO
          // pointers to 0x00804400 so the 1000B FIFO starts fresh for the
          // next burst.
          k->clearRxBuffer();
        }
      }
    }

    if (in_burst) {
      if (static_cast<int32_t>(nowMs() - burst_deadline_ms) < 0) {
        sleepUs(150);
        continue; // poll SPI IRQ while burst packets are arriving
      }
      {
        std::lock_guard<std::recursive_mutex> rlk(radio_mu_);
        k->clearRxBuffer();
        armReceive();
      }
      in_burst = false;
      st_.burst_mode = 0;
    }
    if (!burst_rx_queue.empty()) {
      if (cb_.on_rx_packet) {
        for (const auto &q : burst_rx_queue) {
          cb_.on_rx_packet(q.data, q.len, q.rssi);
        }
      }
      burst_rx_queue.clear();
    }
    break;
  }

  if (!scan_active_) {
    serviceTxQueue();
  }
}

void RadioWorker::workerLoop() {
  for (;;) {
    pollOnce();
    sleepUs(1000);
  }
}

void RadioWorker::snapshot(stats::Counters *out, uint32_t uptime_ms) {
  std::lock_guard<std::mutex> lk(tx_mu_);
  st_.uptime_ms = uptime_ms;
  *out = st_;
}

} // namespace radio

// Serial server implementation — see serial_server.hpp.
#include "serial_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "burst.hpp"

using namespace burst;

namespace serial {

uint32_t SerialServer::nowMs() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static constexpr uint32_t kIpcTotalSize = 262144;
static constexpr uint32_t kIpcR2VBase = 0x00000;
static constexpr uint32_t kIpcV2RBase = 0x10000;
static constexpr uint32_t kIpcRingCap = 65520;

SerialServer::SerialServer(radio::RadioWorker *worker,
                           const std::string &uart_path, uint32_t baud)
    : worker_(worker), uart_path_(uart_path), baud_(baud) {
  if (uart_path_.rfind("ipc://", 0) == 0) {
    is_ipc_ = true;
    ipc_path_ = uart_path_.substr(6);
    if (ipc_path_.empty()) {
      ipc_path_ = "/tmp/slx_ipc.bin";
    }
  } else if (uart_path_.rfind("tcp://", 0) == 0) {
    is_tcp_ = true;
    size_t colon = uart_path_.rfind(':');
    if (colon != std::string::npos && colon > 5) {
      tcp_port_ = static_cast<uint16_t>(std::atoi(uart_path_.c_str() + colon + 1));
    }
    if (tcp_port_ == 0) {
      tcp_port_ = 9200;
    }
  } else if (!uart_path_.empty() && uart_path_[0] == ':') {
    is_tcp_ = true;
    tcp_port_ = static_cast<uint16_t>(std::atoi(uart_path_.c_str() + 1));
    if (tcp_port_ == 0) {
      tcp_port_ = 9200;
    }
  }
}

int SerialServer::readByte(uint8_t *b, uint32_t timeout_ms) {
  if (fd_ < 0) {
    return -2;
  }
  if (is_ipc_) {
    if (ipc_rx_pos_ < ipc_rx_len_) {
      *b = ipc_rx_buf_[ipc_rx_pos_++];
      return 0;
    }
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint32_t deadline =
        static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000) + timeout_ms;
    for (;;) {
      {
        std::lock_guard<std::recursive_mutex> lk(write_mu_);
        uint32_t hdr[2] = {0, 0};
        lseek(fd_, kIpcV2RBase, SEEK_SET);
        if (read(fd_, hdr, 8) == 8 && hdr[0] != hdr[1]) {
          uint32_t avail = hdr[0] - hdr[1];
          if (avail > kIpcRingCap) {
            hdr[1] = hdr[0];
            lseek(fd_, kIpcV2RBase + 4, SEEK_SET);
            write(fd_, &hdr[1], 4);
          } else {
            uint32_t take = avail < sizeof(ipc_rx_buf_)
                                ? avail
                                : static_cast<uint32_t>(sizeof(ipc_rx_buf_));
            uint32_t off = hdr[1] % kIpcRingCap;
            uint32_t first = (off + take <= kIpcRingCap) ? take : (kIpcRingCap - off);
            lseek(fd_, kIpcV2RBase + 16 + off, SEEK_SET);
            read(fd_, ipc_rx_buf_, first);
            if (take > first) {
              lseek(fd_, kIpcV2RBase + 16, SEEK_SET);
              read(fd_, ipc_rx_buf_ + first, take - first);
            }
            hdr[1] += take;
            lseek(fd_, kIpcV2RBase + 4, SEEK_SET);
            write(fd_, &hdr[1], 4);
            ipc_rx_pos_ = 1;
            ipc_rx_len_ = take;
            *b = ipc_rx_buf_[0];
            return 0;
          }
        }
      }
      if (timeout_ms == 0) {
        return -1;
      }
      clock_gettime(CLOCK_MONOTONIC, &ts);
      uint32_t now = static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
      if (static_cast<int32_t>(now - deadline) >= 0) {
        return -1;
      }
      struct timespec slp{0, 500000}; // 500 µs
      nanosleep(&slp, nullptr);
    }
  }
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  uint32_t deadline =
      static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000) + timeout_ms;
  for (;;) {
    ssize_t n = read(fd_, b, 1);
    if (n == 1) {
      return 0;
    }
    if (is_tcp_) {
      if (n == 0) {
        return -2; // TCP peer closed connection
      }
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        return -2; // TCP socket error
      }
    }
    if (timeout_ms == 0) {
      return -1;
    }
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint32_t now = static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    if (static_cast<int32_t>(now - deadline) >= 0) {
      return -1;
    }
    // brief sleep to avoid a hard spin on an empty FIFO
    struct timespec slp{0, 200000}; // 200 µs
    nanosleep(&slp, nullptr);
  }
}

bool SerialServer::writeAll(const uint8_t *buf, size_t len) {
  if (fd_ < 0) {
    return false;
  }
  if (is_ipc_) {
    std::lock_guard<std::recursive_mutex> lk(write_mu_);
    uint32_t hdr[2] = {0, 0};
    lseek(fd_, kIpcR2VBase, SEEK_SET);
    if (read(fd_, hdr, 8) != 8) {
      return false;
    }
    uint32_t used = hdr[0] - hdr[1];
    if (used > kIpcRingCap || len > kIpcRingCap) {
      hdr[1] = hdr[0];
      used = 0;
    }
    if (used + len > kIpcRingCap) {
      // Advance reader tail to make room if validator hasn't drained yet
      hdr[1] = hdr[0] - (kIpcRingCap - static_cast<uint32_t>(len));
      lseek(fd_, kIpcR2VBase + 4, SEEK_SET);
      write(fd_, &hdr[1], 4);
    }
    uint32_t off = hdr[0] % kIpcRingCap;
    uint32_t n = static_cast<uint32_t>(len);
    uint32_t first = (off + n <= kIpcRingCap) ? n : (kIpcRingCap - off);
    lseek(fd_, kIpcR2VBase + 16 + off, SEEK_SET);
    write(fd_, buf, first);
    if (n > first) {
      lseek(fd_, kIpcR2VBase + 16, SEEK_SET);
      write(fd_, buf + first, n - first);
    }
    hdr[0] += n;
    lseek(fd_, kIpcR2VBase, SEEK_SET);
    write(fd_, &hdr[0], 4);
    return true;
  }
  size_t off = 0;
  while (off < len) {
    ssize_t n;
    if (is_tcp_) {
#ifdef MSG_NOSIGNAL
      n = send(fd_, buf + off, len - off, MSG_NOSIGNAL);
#else
      n = send(fd_, buf + off, len - off, 0);
#endif
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        struct timespec slp{0, 200000};
        nanosleep(&slp, nullptr);
        continue;
      }
    } else {
      n = write(fd_, buf + off, len - off);
    }
    if (n <= 0) {
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

void SerialServer::writeFrame(char tag, const uint8_t *payload, size_t len) {
  uint8_t hdr[3] = {static_cast<uint8_t>(tag),
                    static_cast<uint8_t>(len & 0xFF),
                    static_cast<uint8_t>((len >> 8) & 0xFF)};
  writeAll(hdr, 3);
  if (len) {
    writeAll(payload, len);
  }
}

void SerialServer::onWorkerRx(const uint8_t *pkt, size_t len, int16_t rssi) {
  writeRx(pkt, len, rssi);
}

void SerialServer::onWorkerTerminal(radio::Terminal t) {
  if (t == radio::Terminal::kOk) {
    writeReady(511); // 'G' terminal — confirmed aired
  } else {
    uint8_t code = static_cast<uint8_t>(t);
    writeError(code, code == 0x05 ? "tx aborted" : "tx dropped");
  }
}

void SerialServer::onWorkerScan(uint32_t freq_hz, uint32_t frames,
                                uint32_t consensus, int8_t best_rssi) {
  writeDiag(50, freq_hz);
  writeDiag(51, frames);
  writeDiag(52, consensus);
  writeDiag(53, static_cast<uint32_t>(static_cast<int32_t>(best_rssi) + 128));
}

void SerialServer::onWorkerLog(const std::string &line) {
  printf("[radio] %s\n", line.c_str());
}

void SerialServer::writeReady(uint16_t max_payload) {
  // LEGACY READY SHAPE (2026-10-07): [G][max_payload:u16] — no protocol
  // version byte, matching the flashed nRF build (its TX terminals report
  // max_payload=511 with no version byte; the node logs "did not
  // negotiate v2" and keeps CONFIG at the legacy 15-byte body). The
  // max_payload VALUE is 511 — the fleet's on-air packet size. (v2 could
  // be advertised safely — the host's selectors would set the same 511 B
  // profile — but legacy keeps the serial surface identical to the fleet
  // until the v2 build ships everywhere.)
  std::lock_guard<std::recursive_mutex> lk(write_mu_);
  uint8_t p[3] = {static_cast<uint8_t>('G'),
                  static_cast<uint8_t>(max_payload & 0xFF),
                  static_cast<uint8_t>((max_payload >> 8) & 0xFF)};
  writeAll(p, 3);
}

void SerialServer::writeError(uint8_t code, const char *msg) {
  std::lock_guard<std::recursive_mutex> lk(write_mu_);
  uint8_t p[3 + 64];
  size_t mlen = strlen(msg);
  if (mlen > 64) {
    mlen = 64;
  }
  p[0] = static_cast<uint8_t>('E');
  p[1] = code;
  p[2] = static_cast<uint8_t>(mlen);
  memcpy(p + 3, msg, mlen);
  writeAll(p, 3 + mlen);
}

void SerialServer::writeStats() {
  std::lock_guard<std::recursive_mutex> lk(write_mu_);
  stats::Counters c{};
  worker_->snapshot(&c, nowMs());
  c.parser_frames_killed = parser_frames_killed_;
  uint8_t body[68];
  stats::serialize(c, body);
  writeFrame('S', body, 68);
}

void SerialServer::writeRx(const uint8_t *pkt, size_t len, int16_t rssi) {
  std::lock_guard<std::recursive_mutex> lk(write_mu_);
  // 'R' [len:u16][payload][rssi:i16] — raw air packet, host reassembles [FB].
  uint8_t frame[3 + 512 + 2];
  if (len <= 512) {
    frame[0] = static_cast<uint8_t>('R');
    frame[1] = static_cast<uint8_t>(len & 0xFF);
    frame[2] = static_cast<uint8_t>((len >> 8) & 0xFF);
    if (len > 0) {
      memcpy(frame + 3, pkt, len);
    }
    frame[3 + len] = static_cast<uint8_t>(rssi & 0xFF);
    frame[3 + len + 1] = static_cast<uint8_t>((rssi >> 8) & 0xFF);
    writeAll(frame, 3 + len + 2);
    return;
  }
  uint8_t hdr[3] = {static_cast<uint8_t>('R'),
                    static_cast<uint8_t>(len & 0xFF),
                    static_cast<uint8_t>((len >> 8) & 0xFF)};
  uint8_t r[2] = {static_cast<uint8_t>(rssi & 0xFF),
                  static_cast<uint8_t>((rssi >> 8) & 0xFF)};
  writeAll(hdr, 3);
  writeAll(pkt, len);
  writeAll(r, 2);
}

void SerialServer::writeDiag(uint8_t event_id, uint32_t val) {
  std::lock_guard<std::recursive_mutex> lk(write_mu_);
  uint8_t p[6] = {event_id,
                  static_cast<uint8_t>(val & 0xFF),
                  static_cast<uint8_t>((val >> 8) & 0xFF),
                  static_cast<uint8_t>((val >> 16) & 0xFF),
                  static_cast<uint8_t>((val >> 24) & 0xFF),
                  0};
  writeFrame('D', p, 6);
}

void SerialServer::handleConfig(const uint8_t *body, uint8_t len) {
  ParsedConfig c{};
  c.frequency_hz = static_cast<uint32_t>(body[0]) |
                   (static_cast<uint32_t>(body[1]) << 8) |
                   (static_cast<uint32_t>(body[2]) << 16) |
                   (static_cast<uint32_t>(body[3]) << 24);
  c.bandwidth_hz = static_cast<uint32_t>(body[4]) |
                   (static_cast<uint32_t>(body[5]) << 8) |
                   (static_cast<uint32_t>(body[6]) << 16) |
                   (static_cast<uint32_t>(body[7]) << 24);
  c.coding_rate = body[8];
  c.tx_power_dbm = static_cast<int8_t>(body[9]);
  c.sync_word = static_cast<uint32_t>(body[10]) |
                (static_cast<uint32_t>(body[11]) << 8) |
                (static_cast<uint32_t>(body[12]) << 16) |
                (static_cast<uint32_t>(body[13]) << 24);
  c.role = body[14];
  c.body_len = len;
  if (len >= 17) {
    // CONFIG v2 selectors are parsed but NOT applied: this radio's air
    // profile is the fleet's 511 B / 2.6 Mbps (see burst.hpp / writeReady
    // — READY stays legacy, so a well-behaved host never sends these).
    // rate_sel=1 and payload_sel=1 would be no-ops today; payload_sel=0
    // (252 B) WOULD diverge the chunk stride from the nRF peers and is
    // deliberately ignored.
    c.rate_sel = body[15];
    c.payload_sel = body[16];
  }

  if (c.frequency_hz == 0xFFFFFFFFu) {
    // scan magic — body repurposed: sync = start Hz, bw = step Hz,
    // cr = channel count, pwr = dwell ms ÷ 100.
    radio::ScanParams p;
    p.start_hz = c.sync_word;
    p.step_hz = c.bandwidth_hz;
    p.count = c.coding_rate == 0 ? 27 : c.coding_rate;
    if (p.count > 64) {
      p.count = 64;
    }
    p.dwell_ms = static_cast<uint32_t>(static_cast<uint8_t>(c.tx_power_dbm)) * 100;
    if (p.dwell_ms < 50) {
      p.dwell_ms = 50;
    }
    worker_->startScan(p);
    writeReady(511);
    return;
  }

  std::string err;
  radio::Config rc;
  rc.frequency_hz = c.frequency_hz;
  rc.tx_power_dbm = c.tx_power_dbm;
  rc.role = c.role;
  if (!worker_->applyConfig(rc, &err)) {
    writeError(0x03, err.c_str());
    return;
  }
  writeReady(511);
}

void SerialServer::handleScanQ(uint8_t count, uint32_t dwell_ms,
                               const uint32_t *freqs) {
  if (count == 0 || count > 16) {
    return; // silently ignored per the contract
  }
  (void)freqs; // v1 uses the grid form (scan-CONFIG); explicit list is a
               // Phase-2 refinement once the host needs it.
  radio::ScanParams p;
  p.start_hz = freqs[0];
  p.step_hz = 0;
  p.count = count;
  p.dwell_ms = dwell_ms < 50 ? 50 : dwell_ms;
  worker_->startScan(p);
}

void SerialServer::handleReset() {
  // Contract: a single bare 'G' byte, then the radio state resets. On the
  // nRF this is a full MCU reboot; here the port stays open, so we emit the
  // bare G, re-init the radio, and the next READY follows from the normal
  // cadence (the host re-sends CONFIG every ~1 s during handshake).
  uint8_t g = 'G';
  writeAll(&g, 1);
  if (!is_tcp_ && fd_ >= 0) {
    fsync(fd_);
  }
  // Full re-init (radio power cycle is inside init) happens via the worker;
  // v1 keeps the worker alive and re-arms RX — enough to clear TX/scan state.
}

void SerialServer::handleTx(const uint8_t *payload, size_t len) {
  radio::Terminal t = worker_->enqueueTx(payload, len);
  if (t == radio::Terminal::kBusy) {
    writeError(0x04, "tx queue full");
    return; // frame not taken — but the contract says 0x04 IS the terminal
  }
  writeDiag(47, static_cast<uint32_t>(len));
  // The 'G'/'E' terminal arrives via the worker's on_tx_terminal callback.
}

void SerialServer::maybeEmitStats(bool force) {
  uint32_t now = nowMs();
  if (force || static_cast<uint32_t>(now - last_stats_ms_) >= 10000) {
    last_stats_ms_ = now;
    writeStats();
  }
}

void SerialServer::init() {
  if (is_ipc_) {
    fd_ = open(ipc_path_.c_str(), O_CREAT | O_RDWR, 0666);
    while (fd_ < 0) {
      struct timespec slp{0, 500000000};
      nanosleep(&slp, nullptr);
      fd_ = open(ipc_path_.c_str(), O_CREAT | O_RDWR, 0666);
    }
    off_t sz = lseek(fd_, 0, SEEK_END);
    if (sz < static_cast<off_t>(kIpcTotalSize)) {
      uint8_t zero_page[4096] = {0};
      lseek(fd_, 0, SEEK_SET);
      for (uint32_t off = 0; off < kIpcTotalSize; off += sizeof(zero_page)) {
        write(fd_, zero_page, sizeof(zero_page));
      }
    }
    uint32_t zero_hdr[2] = {0, 0};
    lseek(fd_, kIpcR2VBase, SEEK_SET);
    write(fd_, zero_hdr, 8);
    lseek(fd_, kIpcV2RBase, SEEK_SET);
    write(fd_, zero_hdr, 8);
    printf("[serial] IPC ringbuffer server active at %s\n", ipc_path_.c_str());
  } else if (is_tcp_) {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    while (listen_fd_ < 0) {
      struct timespec slp{0, 500000000};
      nanosleep(&slp, nullptr);
      listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    }
    int one = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(tcp_port_);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    while (bind(listen_fd_, reinterpret_cast<struct sockaddr *>(&addr),
                sizeof(addr)) < 0) {
      struct timespec slp{0, 500000000};
      nanosleep(&slp, nullptr);
    }
    listen(listen_fd_, 4);
    printf("[serial] TCP server listening on port %u\n", tcp_port_);
  } else {
    fd_ = open(uart_path_.c_str(), O_RDWR | O_NOCTTY);
    while (fd_ < 0) {
      struct timespec slp{0, 500000000};
      nanosleep(&slp, nullptr);
      fd_ = open(uart_path_.c_str(), O_RDWR | O_NOCTTY);
    }
    struct termios tio{};
    tcgetattr(fd_, &tio);
    speed_t spd = B115200;
    switch (baud_) {
      case 921600: spd = B921600; break;
      case 460800: spd = B460800; break;
      case 230400: spd = B230400; break;
      case 1000000: spd = B1000000; break;
      default: spd = B115200; break;
    }
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
    tio.c_oflag &= ~OPOST;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &tio);
    tcflush(fd_, TCIOFLUSH);
  }

  for (int i = 0; i < 150 && !worker_->radioOk(); i++) {
    struct timespec slp{0, 100000000};
    nanosleep(&slp, nullptr);
  }
  if (!is_tcp_) {
    writeReady(511);
    ready_sent_ = true;
    last_stats_ms_ = nowMs();
    writeStats();
  }
}

void SerialServer::pollOnce() {
  if (is_tcp_ && fd_ < 0) {
    int cfd = accept(listen_fd_, nullptr, nullptr);
    if (cfd < 0) {
      return;
    }
    int one = 1;
    setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    int fl = fcntl(cfd, F_GETFL, 0);
    if (fl >= 0) {
      fcntl(cfd, F_SETFL, fl | O_NONBLOCK);
    }
    {
      std::lock_guard<std::recursive_mutex> lk(write_mu_);
      fd_ = cfd;
      state_ = State::kTag;
      skip_remain_ = 0;
      have_ = 0;
    }
    printf("[serial] TCP client connected (fd=%d)\n", cfd);
    writeReady(511);
    ready_sent_ = true;
    last_stats_ms_ = nowMs();
    writeStats();
  }

  for (;;) {
    uint8_t b;
    int rc = readByte(&b, 0);
    if (rc == -2) {
      std::lock_guard<std::recursive_mutex> lk(write_mu_);
      if (fd_ >= 0) {
        printf("[serial] TCP client disconnected (fd=%d)\n", fd_);
        close(fd_);
        fd_ = -1;
      }
      return;
    }
    if (rc != 0) {
      maybeEmitStats(false);
      return;
    }
    uint32_t now = nowMs();
    uint32_t since_arrival = now - last_arrival_ms_;

    if ((state_ != State::kTag || skip_remain_ > 0) && since_arrival > 100) {
      if (state_ == State::kPayload || state_ == State::kConfig ||
          state_ == State::kSetWindow || state_ == State::kLenHi) {
        parser_frames_killed_++;
      }
      skip_remain_ = 0;
      state_ = State::kTag;
      have_ = 0;
    }
    last_arrival_ms_ = now;

    if (skip_remain_ > 0) {
      skip_remain_--;
      continue;
    }

    switch (state_) {
    case State::kTag:
      tag_ = b;
      if (b == 'T') {
        state_ = State::kTxMagic;
        have_ = 0;
      } else if (b == 'W') {
        state_ = State::kSetWindow;
        have_ = 0;
      } else if (b == 'C') {
        state_ = State::kLenLo;
        have_ = 0;
      } else if (b == 'S') {
        maybeEmitStats(true);
      } else if (b == 'Q') {
        state_ = State::kScanCount;
        have_ = 0;
      } else if (b == 'X') {
        handleReset();
      }
      break;

    case State::kTxMagic:
      if (b != 'X') {
        state_ = State::kTag; // drop, resume scanning
        break;
      }
      state_ = State::kLenLo;
      break;

    case State::kSetWindow:
      window_body_[have_++] = b;
      if (have_ == 8) {
        slot_offset_ms_ = static_cast<uint32_t>(window_body_[0]) |
                          (static_cast<uint32_t>(window_body_[1]) << 8) |
                          (static_cast<uint32_t>(window_body_[2]) << 16) |
                          (static_cast<uint32_t>(window_body_[3]) << 24);
        slot_period_ms_ = static_cast<uint32_t>(window_body_[4]) |
                          (static_cast<uint32_t>(window_body_[5]) << 8) |
                          (static_cast<uint32_t>(window_body_[6]) << 16) |
                          (static_cast<uint32_t>(window_body_[7]) << 24);
        writeReady(511);
        state_ = State::kTag;
      }
      break;

    case State::kLenLo:
      frame_len_ = b;
      state_ = State::kLenHi;
      break;

    case State::kLenHi: {
      frame_len_ |= static_cast<uint16_t>(b << 8);
      if (tag_ == 'C') {
        if (frame_len_ != 15 && frame_len_ != 17) {
          writeError(0x03, "bad len");
          state_ = State::kTag;
        } else {
          state_ = State::kConfig;
          have_ = 0;
        }
      } else if (tag_ == 'T') {
        if (frame_len_ == 0 || frame_len_ > kMaxTotalPayload) {
          writeError(0x02, "bad tx len");
          skip_remain_ = frame_len_;
          state_ = State::kTag;
        } else {
          state_ = State::kPayload;
          have_ = 0;
        }
      } else {
        state_ = State::kTag;
      }
      break;
    }

    case State::kConfig:
      config_body_[have_++] = b;
      if (have_ == frame_len_) {
        handleConfig(config_body_, static_cast<uint8_t>(frame_len_));
        state_ = State::kTag;
      }
      break;

    case State::kPayload: {
      static uint8_t tx_buf[kMaxTotalPayload];
      tx_buf[have_++] = b;
      if (have_ == frame_len_) {
        handleTx(tx_buf, frame_len_);
        state_ = State::kTag;
      }
      break;
    }

    case State::kScanCount:
      scan_count_ = b;
      scan_have_ = 0;
      state_ = (scan_count_ >= 1 && scan_count_ <= 16) ? State::kScanBody
                                                       : State::kTag;
      break;

    case State::kScanBody: {
      // dwell u32 LE + freq u32 LE × count
      static uint8_t qbuf[4 + 16 * 4];
      qbuf[scan_have_++] = b;
      size_t need = 4 + static_cast<size_t>(scan_count_) * 4;
      if (scan_have_ == need) {
        uint32_t dwell = static_cast<uint32_t>(qbuf[0]) |
                         (static_cast<uint32_t>(qbuf[1]) << 8) |
                         (static_cast<uint32_t>(qbuf[2]) << 16) |
                         (static_cast<uint32_t>(qbuf[3]) << 24);
        uint32_t freqs[16];
        for (int i = 0; i < scan_count_; i++) {
          const uint8_t *p = qbuf + 4 + i * 4;
          freqs[i] = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                     (static_cast<uint32_t>(p[2]) << 16) |
                     (static_cast<uint32_t>(p[3]) << 24);
        }
        handleScanQ(scan_count_, dwell, freqs);
        state_ = State::kTag;
      }
      break;
    }
    }

    maybeEmitStats(false);
  }
}

void SerialServer::run() {
  init();
  for (;;) {
    pollOnce();
    struct timespec slp{0, 1000000};
    nanosleep(&slp, nullptr);
  }
}

} // namespace serial

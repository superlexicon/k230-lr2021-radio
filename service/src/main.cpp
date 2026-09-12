// k230-lr2021-radio service entry point.
//
// Boot order: radio power → RadioLib init (worker) → spawn the worker
// thread → run the serial server (this thread). The serial server sends
// READY as soon as the worker reports radio-ok, satisfying the host's
// handshake (CONFIG re-sent every ~1 s until 'G').
#include <pthread.h>

#include <cstdio>

#include "radio_worker.hpp"
#include "serial_server.hpp"

static radio::RadioWorker g_worker;
static serial::SerialServer *g_server = nullptr;

static void *worker_entry(void *) {
  g_worker.workerLoop();
  return nullptr;
}

int main(int argc, char **argv) {
  const char *uart = (argc > 1) ? argv[1] : "/dev/ttyS1";
  uint32_t baud = (argc > 2) ? static_cast<uint32_t>(atoi(argv[2])) : 115200;

  radio::Callbacks cb;
  cb.on_rx_packet = [](const uint8_t *pkt, size_t len, int16_t rssi) {
    if (g_server) {
      g_server->onWorkerRx(pkt, len, rssi);
    }
  };
  cb.on_tx_terminal = [](radio::Terminal t) {
    if (g_server) {
      g_server->onWorkerTerminal(t);
    }
  };
  cb.on_scan_result = [](uint32_t freq, uint32_t frames, uint32_t consensus,
                         int8_t rssi) {
    if (g_server) {
      g_server->onWorkerScan(freq, frames, consensus, rssi);
    }
  };
  cb.on_log = [](const std::string &line) {
    printf("[radio] %s\n", line.c_str());
  };

  if (!g_worker.init(cb)) {
    printf("[main] radio init failed — retrying is the host's job; exiting\n");
    // The host treats 'E' 0x01 at handshake as fatal-for-session and keeps
    // reopening; a reboot loop would also satisfy it, but exiting lets the
    // RT-SMART supervisor (or SD-card relaunch) retry cleanly.
    return 1;
  }

  static serial::SerialServer server(&g_worker, uart, baud);
  g_server = &server;

  pthread_t th;
  pthread_create(&th, nullptr, worker_entry, nullptr);

  printf("[main] radio service up: uart=%s baud=%u\n", uart, baud);
  server.run(); // never returns
  return 0;
}

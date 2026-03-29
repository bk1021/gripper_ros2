#pragma once
#include <cstdint>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <vector>
#include <string>

// Callback type: invoked from RX thread when a valid frame arrives
using RxCallback = std::function<void(uint32_t can_id, const uint8_t* data, uint8_t len)>;

class SerialTransport {
public:
  SerialTransport() = default;
  ~SerialTransport();

  bool open(const std::string& port, int baudrate);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Thread-safe CAN frame send
  // Builds 17-byte frame: 0xAA 0x00 0x00 DLC ID3 ID2 ID1 ID0 D[0..7] 0x7A
  bool send_frame(uint32_t can_id, const std::vector<uint8_t>& payload);

  void set_rx_callback(RxCallback cb) { rx_cb_ = cb; }

  // Milliseconds since last valid frame was received
  uint64_t rx_age_ms() const;

private:
  void rx_loop();

  int fd_{-1};
  std::mutex tx_mutex_;
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  RxCallback rx_cb_;
  std::atomic<uint64_t> last_rx_ms_{0};
};
#include "gripper_ros2/serial_transport.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <iostream>

SerialTransport::~SerialTransport() { close(); }

bool SerialTransport::open(const std::string& port, int baudrate) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  termios tty{};
  tcgetattr(fd_, &tty);
  // Map baudrate — extend as needed
  speed_t speed = (baudrate == 921600) ? B921600 : B115200;
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  cfmakeraw(&tty);
  // VMIN=0 + VTIME=1: read() times out after 100ms if no data arrives.
  // This is what allows close() -> running_=false -> join() to unblock
  // the RX thread within one tick rather than hanging indefinitely.
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;
  tcsetattr(fd_, TCSANOW, &tty);

  // Switch back to blocking mode for cleaner RX
  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  // FIX: seed last_rx_ms_ to now so rx_age_ms() doesn't return a
  // billions-of-milliseconds false alarm before the first packet arrives.
  last_rx_ms_ = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());

  running_ = true;
  rx_thread_ = std::thread(&SerialTransport::rx_loop, this);
  return true;
}

void SerialTransport::close() {
  running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SerialTransport::send_frame(uint32_t can_id, const std::vector<uint8_t>& payload) {
  // FIX: guard against calling write() on an invalid fd (e.g. if open()
  // failed or send_frame is called before open()), which would silently
  // return EBADF and spam WARN logs from the node.
  if (fd_ < 0) return false;
  if (payload.size() > 8) return false;

  uint8_t frame[17]{};
  frame[0] = 0xAA;
  frame[1] = 0x00; frame[2] = 0x00;
  const uint8_t dlc = static_cast<uint8_t>(payload.size());
  frame[3] = dlc;
  frame[4] = (can_id >> 24) & 0xFF;
  frame[5] = (can_id >> 16) & 0xFF;
  frame[6] = (can_id >>  8) & 0xFF;
  frame[7] =  can_id        & 0xFF;
  for (size_t i = 0; i < dlc; ++i)
    frame[8 + i] = payload[i];
  frame[16] = 0x7A;

  std::lock_guard<std::mutex> lock(tx_mutex_);
  return ::write(fd_, frame, 17) == 17;
}

uint64_t SerialTransport::rx_age_ms() const {
  auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  return static_cast<uint64_t>(now) - last_rx_ms_.load();
}

void SerialTransport::rx_loop() {
  while (running_) {
    // 1. Scan for sync byte
    uint8_t byte = 0;
    if (::read(fd_, &byte, 1) != 1) continue;
    if (byte != 0xAA) continue;

    // 2. Read remaining 16 bytes
    uint8_t rest[16]{};
    size_t got = 0;
    while (got < 16 && running_) {
      ssize_t n = ::read(fd_, rest + got, 16 - got);
      if (n > 0) got += n;
    }
    if (got < 16) continue;

    // 3. Validate end byte — drop and rescan if corrupt
    if (rest[15] != 0x7A) continue;

    // 4. Extract CAN ID and data
    uint32_t can_id = ((uint32_t)rest[3] << 24) | ((uint32_t)rest[4] << 16) |
                      ((uint32_t)rest[5] <<  8) |  (uint32_t)rest[6];

    // Update RX timestamp
    last_rx_ms_ = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    if (rx_cb_) rx_cb_(can_id, rest + 7, 8);
  }
}
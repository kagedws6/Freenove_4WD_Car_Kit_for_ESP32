// Original project extension. See repository attribution and license.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>

namespace rover {
constexpr int kSpeedLimit = 2200; // PWM units, not a measured vehicle speed.
constexpr uint32_t kLeaseMs = 500;
constexpr uint32_t kMaxDriveMs = 1500;
constexpr uint32_t kFrameTimeoutMs = 250;

struct Command {
  char name[24] = {};
  int values[7] = {};
  size_t count = 0;
};

inline bool parseCommand(const char* line, Command& out) {
  out = Command{};
  const char* sep = strchr(line, '#');
  size_t n = sep ? static_cast<size_t>(sep - line) : strlen(line);
  if (!n || n >= sizeof(out.name)) return false;
  for (size_t i = 0; i < n; ++i)
    if ((line[i] < 'A' || line[i] > 'Z') && line[i] != '_') return false;
  memcpy(out.name, line, n);
  while (sep) {
    if (out.count == 7) return false;
    const char* start = sep + 1;
    sep = strchr(start, '#');
    size_t len = sep ? static_cast<size_t>(sep - start) : strlen(start);
    if (!len || len > 11) return false;
    size_t first = start[0] == '-' ? 1 : 0;
    if (first == len) return false;
    for (size_t i = first; i < len; ++i)
      if (start[i] < '0' || start[i] > '9') return false;
    errno = 0;
    char* end = nullptr;
    long value = strtol(start, &end, 10);
    if (errno == ERANGE || end != start + len || value < INT_MIN || value > INT_MAX)
      return false;
    out.values[out.count++] = static_cast<int>(value);
  }
  return true;
}

class LineReceiver {
 public:
  enum Result { Pending, Complete, Invalid };
  char line[128] = {};
  void reset() { used_ = 0; dropping_ = false; carriageReturn_ = false; }
  bool expire(uint32_t now) {
    if ((used_ || dropping_) && static_cast<uint32_t>(now - lastByte_) >= kFrameTimeoutMs) {
      used_ = 0;
      dropping_ = true; // Discard the expired frame's tail through its newline.
      lastByte_ = now;
      return true;
    }
    return false;
  }
  Result push(char ch, uint32_t now) {
    lastByte_ = now;
    if (ch == '\n') {
      if (dropping_ || !used_) { reset(); return Invalid; }
      line[used_] = 0;
      used_ = 0;
      carriageReturn_ = false;
      return Complete;
    }
    if (dropping_) return Pending;
    if (carriageReturn_) { used_ = 0; dropping_ = true; return Invalid; }
    if (ch == '\r') { carriageReturn_ = true; return Pending; }
    if (ch < 32 || ch > 126 || used_ >= sizeof(line) - 1) {
      used_ = 0; dropping_ = true;
      return Invalid;
    }
    line[used_++] = ch;
    return Pending;
  }
 private:
  size_t used_ = 0;
  bool dropping_ = false;
  bool carriageReturn_ = false;
  uint32_t lastByte_ = 0;
};

class MotionGuard {
 public:
  int target[4] = {};
  int output[4] = {};
  bool armed = false;
  void arm(uint32_t now, uint32_t duration = kLeaseMs, bool bounded = false) {
    lastCommand_ = now; lease_ = duration; bounded_ = bounded; armed = true;
  }
  bool heartbeat(uint32_t now) {
    if (!armed || static_cast<uint32_t>(now - lastCommand_) >= lease_) {
      stop(); return false;
    }
    if (!bounded_) lastCommand_ = now;
    return true;
  }
  void stop() {
    armed = false;
    for (int i = 0; i < 4; ++i) target[i] = output[i] = 0;
  }
  void request(int a, int b, int c, int d) {
    int v[4] = {a,b,c,d};
    bool zero = true;
    for (int i = 0; i < 4; ++i) {
      target[i] = armed ? clamp(v[i]) : 0;
      zero = zero && target[i] == 0;
    }
    if (zero) for (int i = 0; i < 4; ++i) output[i] = 0;
  }
  bool tick(uint32_t now) {
    if (armed && static_cast<uint32_t>(now - lastCommand_) >= lease_) stop();
    uint32_t dt = static_cast<uint32_t>(now - lastTick_);
    if (dt < 10) return armed;
    lastTick_ = now;
    if (dt > 50) dt = 50; // A stalled loop must not jump straight to full output.
    const int step = static_cast<int>(dt * 4000 / 1000);
    for (int i = 0; i < 4; ++i) {
      int goal = target[i];
      if ((goal < 0 && output[i] > 0) || (goal > 0 && output[i] < 0)) goal = 0;
      int delta = goal - output[i];
      output[i] += delta > step ? step : delta < -step ? -step : delta;
    }
    return armed;
  }
 private:
  static int clamp(int v) { return v > kSpeedLimit ? kSpeedLimit : v < -kSpeedLimit ? -kSpeedLimit : v; }
  uint32_t lastCommand_ = 0, lastTick_ = 0, lease_ = kLeaseMs;
  bool bounded_ = false;
};
} // namespace rover

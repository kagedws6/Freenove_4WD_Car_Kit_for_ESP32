#include "../Sketches/06.3_Multi_Functional_Car/RoverControl.h"
#include <assert.h>
#include <stdio.h>
#include <string>

int main() {
  rover::Command c;
  assert(rover::parseCommand("CMD_MOTOR#4095#4095#-4095#-4095", c));
  assert(c.count == 4 && c.values[2] == -4095);
  assert(rover::parseCommand("CMD_STOP", c) && c.count == 0);
  assert(rover::parseCommand("CMD_DRIVE#-2147483648#0#1", c));
  for (const char* bad : {"", "CMD_STOP#", "CMD_DRIVE##1", "CMD_DRIVE#1x",
       "CMD_DRIVE# 1", "CMD_DRIVE#+1", "CMD_DRIVE#2147483648",
       "CMD_DRIVE#-2147483649", "CMD_DRIVE#999999999999999",
       "CMD_DRIVE#1#2#3#4#5#6#7#8", "cmd_stop", "CMD_DRIVE#-"})
    assert(!rover::parseCommand(bad, c));
  rover::LineReceiver rx;
  for (char ch : std::string("CMD_STOP\r\n")) {
    auto r = rx.push(ch, 0);
    if (ch == '\n') assert(r == rover::LineReceiver::Complete);
  }
  assert(std::string(rx.line) == "CMD_STOP");
  rx.push('C', 1); rx.push('\r', 1);
  assert(rx.push('M', 1) == rover::LineReceiver::Invalid);
  assert(rx.push('\n', 1) == rover::LineReceiver::Invalid);
  rx.push('C', 10);
  assert(!rx.expire(259)); assert(rx.expire(260));
  assert(rx.push('\n', 261) == rover::LineReceiver::Invalid);
  for (int i = 0; i < 127; ++i) assert(rx.push('X', 300) == rover::LineReceiver::Pending);
  assert(rx.push('X', 300) == rover::LineReceiver::Invalid);
  assert(rx.push('\n', 301) == rover::LineReceiver::Invalid);
  assert(rx.push('\0', 302) == rover::LineReceiver::Invalid);
  rx.reset();
  for (char ch : std::string("CMD_STOP\n")) rx.push(ch, 400);
  assert(std::string(rx.line) == "CMD_STOP");

  rover::MotionGuard m;
  m.request(4095,4095,4095,4095); m.tick(20); assert(m.output[0] == 0);
  m.arm(20); m.request(4095,-4095,4000,-4000); m.tick(40);
  assert(m.output[0] == 80 && m.output[1] == -80);
  assert(m.target[0] == 2200 && m.target[1] == -2200);
  m.tick(520); assert(!m.armed && m.output[0] == 0);
  assert(!m.heartbeat(521));
  m.arm(1000); m.request(2200,2200,2200,2200);
  assert(m.heartbeat(1490)); m.tick(1989); assert(m.armed);
  m.tick(1990); assert(!m.armed && m.output[0] == 0);
  m.arm(2000,600,true); m.request(2200,2200,2200,2200);
  assert(m.heartbeat(2400)); m.tick(2600); assert(!m.armed);
  m.arm(3000,1500,true); m.request(2200,2200,2200,2200);
  for (uint32_t t=3020;t<=3400;t+=20) m.tick(t);
  int previous=m.output[0]; assert(previous > 0);
  m.request(-2200,-2200,-2200,-2200);
  bool sawZero=false;
  for (uint32_t t=3420;t<=4000;t+=20) {
    m.tick(t);
    if (m.output[0] == 0) sawZero=true;
    if (m.output[0] < 0) assert(sawZero);
    assert(m.output[0] >= -2200 && m.output[0] <= 2200);
  }
  m.request(0,0,0,0); assert(m.output[0] == 0);
  m.stop(); assert(!m.armed && m.target[0] == 0);
  rover::MotionGuard wrap;
  wrap.arm(0xfffffff0U); wrap.request(1000,1000,1000,1000);
  wrap.tick(4); assert(wrap.armed);
  wrap.tick(0x1e4); assert(!wrap.armed && wrap.output[0] == 0);
  puts("PASS: strict parsing, bounded framing, expiry, heartbeat, fixed pulses, slew, reversal, rollover");
}

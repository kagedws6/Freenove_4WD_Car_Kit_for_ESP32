# Modular rover control prototype

The changes are confined to `Sketches/06.3_Multi_Functional_Car`. Other Freenove examples remain unchanged. Based on upstream commit `d7a0c6eb61fcdf0794be5e45c463e7fdcc4163ba`; upstream attribution and its non-commercial/share-alike license remain applicable.

## Movement policy

- Default PWM cap: ±2200 out of the original ±4095 range. PWM is not a measured speed or stopping distance.
- Ramp: 4000 PWM units/second, with elapsed-time capping after a delayed loop. Reverse requests pass through zero.
- Zero-speed requests, stop, expired leases, malformed commands and command-client disconnects cut motor output immediately when serviced, without a deceleration ramp.
- Manual drive and autonomous line/light modes require a fresh motion command or heartbeat within 500 ms. Status, camera and light-display requests do not renew movement.
- Fixed-duration drive requests accept 1–1500 ms. Heartbeats cannot extend a fixed-duration request. A new drive request starts a new duration.
- This is a software control policy, not an independent hardware cutoff. A hung processor, stalled bus, mechanical inertia or electrical fault is not covered by these tests.

The former minimum-1600 PWM boost was removed so the ramp can start at zero. Starting torque and low-speed behavior need physical tuning. Autonomous mode requests are capped too, which changes stock line/light behavior. The original phone app may send only one movement command while a button is held; if it does, movement now expires after 500 ms. Do not bypass the timeout to restore that behavior: update the controller to refresh commands.

## Wire protocol

TCP port 4000 accepts ASCII, `#`-separated fields, terminated by LF or CRLF. One command connection owns control at a time. There is no simultaneous phone/voice arbitration yet. Numeric fields must be complete decimal integers; empty fields, overflow, excessive fields, invalid ranges, unknown commands and overlong frames stop motion. Receive buffers are fixed at 128 bytes. Partial frames time out after 250 ms of inactivity, and their tails are discarded through the newline. Each loop reads at most 96 bytes before returning to motor service. Replies use a non-blocking socket send; backpressure closes the connection and stops outputs.

| Command | Behavior |
| --- | --- |
| `CMD_STOP` | Stop outputs and clear autonomous mode/lease. |
| `CMD_MOTOR#left#left#right#right` | Stock side-pair command; four values must be within ±4095, then PWM is capped. As in upstream, fields 1 and 3 determine the two sides. |
| `CMD_HEARTBEAT` | Refresh a live manual/autonomous lease; cannot revive an expired lease or extend a fixed pulse. |
| `CMD_DRIVE#left#right#milliseconds` | Fixed-duration drive, capped PWM and 1–1500 ms duration. |
| `CMD_STATUS` | Reply: `CMD_STATUS#1#armed#mode#pwm_limit#left_output#right_output#battery_voltage#camera_ready`. Outputs are commanded PWM, not measured wheel speeds. |
| `CMD_POWER` | Original battery-voltage reply. |

Stock servo, head, lights, expression, video, buzzer and car-mode commands remain, with exact field counts and range checks. Servo 1 is 0–180°, servo 2 is 80–180° as constrained by upstream. The camera's stream format on port 7000 is retained. A camera-client disconnect does not restart the controller. Camera initialization failure leaves motor/command service available and reports `camera_ready=0`.

Recommended controller refresh interval: 100 ms while manual movement or an autonomous mode is intentionally active. Always send stop before closing. Stop on controller-side errors too.

## Wi-Fi setup

With no saved settings, connect a phone to the stock `Sunshine` AP (password `Sunshine`) and open `http://192.168.4.1`. Enter home Wi-Fi details locally. The rover stops outputs, saves credentials in its `rover-net` NVS namespace and restarts. It attempts the saved network for up to 15 seconds, then falls back to the setup AP on failure. USB serial prints the resulting IP address, without printing the password. Internet connectivity is not required for the rover's local command interface.

The setup form is only available in AP mode; it is not available for changing credentials over the home LAN. HTTP servicing pauses while a motor lease is active. This prototype retains the stock AP password and has no application authentication on the TCP command port; use a trusted local network and do not forward these ports to the internet. Pairing/authentication and multi-controller ownership are required work before broader deployment.

The voice ESP remains an independent device with its own firmware. It has not yet been given rover tools. First validate this controller with motors unpowered, then validate motion with the wheels clear, before enabling spoken drive requests.

## Build

Validated tool target: Arduino ESP32 core 3.3.11, ESP32 Wrover Module, 4 MB flash, Huge APP partition (3 MB application). Enable PSRAM through this board definition. Use the libraries distributed in this repository's `Libraries` ZIPs; extract them into a local library directory first.

```sh
arduino-cli compile --fqbn esp32:esp32:esp32wrover:PartitionScheme=huge_app --libraries PATH_TO_EXTRACTED_LIBRARIES Sketches/06.3_Multi_Functional_Car
```

See [control-validation.json](control-validation.json) for the build and hardware-check record. Back up the full existing flash privately and verify it against the device before flashing. Keep raw backups, NVS, Wi-Fi details and sensitive serial logs out of GitHub.

## Checkpoint: 2026-10-29

The modified firmware was built and flashed, with both the original 4 MB backup and the written firmware verified against the device. Host control tests passed. The setup page returned HTTP 200, and saved network settings survived a restart. A case-only network-name mismatch was corrected while retaining the entered password; Wi-Fi names are case-sensitive. Startup join failures now report reason codes and whether the exact saved network is visible, without printing passwords or nearby network names.

The TCP bench probe passed on both the setup AP and home LAN: malformed, oversized and expired frames were rejected, output remained zero, and command-client reconnection succeeded. No valid movement request was sent to the real rover. Camera initialization logged an OV3660 sensor-setting warning, but a subsequent 240 × 176 JPEG frame decoded successfully. This checks basic streaming, not image quality or extended reliability.

Physical movement, real-world stopping/ramp behavior and autonomous tracking are untested. The voice bridge is deferred at the owner's request because only one USB cable is available tonight. The working voice firmware remains unchanged. The next integration step is voice-side status and stop tools; spoken driving must follow controlled physical motion testing.

## Host tests

The production parser, frame receiver and motion guard are directly exercised by `tests/control_tests.cpp`. On a host with a C++17 compiler:

```sh
c++ -std=c++17 -Wall -Wextra -Werror tests/control_tests.cpp -o control-tests
./control-tests
```

Tests cover numeric overflow, invalid framing, overlong/expired frames, recovery, disabled movement, limits, timeout boundaries, late heartbeats, fixed pulse deadlines, reversal through zero and `millis()` rollover. They do not measure physical stopping behavior or validate the camera feed.

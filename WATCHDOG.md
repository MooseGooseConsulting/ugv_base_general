# Motion-Command Watchdog (host-silence stop)

## Hazard

The ESP32 lower computer latches the last received motion command forever.
If the host (ROS driver / upper computer) dies or the link drops mid-motion,
the robot keeps driving. This caused a documented wall collision on
2026-09-03 (MooseGooseConsulting UGV Beast).

## Wheel ownership and watchdog

`movtion_module.h` runs encoder sampling, PID and motor writes in one FreeRTOS
wheel task. It starts before the blocking `boot` mission. A nominal 5 ms period
(minimum one RTOS tick) and priority 3 let it preempt Arduino's priority-1 command
loop without depending on mission steps returning. The task has no core affinity.
It dispatches no commands and calls no filesystem, network, serial, servo or
mission functions. A mutex serializes its short wheel service with POD command
updates; command contexts never write wheel pins or mutate PID objects.

- `MOTION_CMD_WATCHDOG_MS` is the initial timeout, default **500 ms**.
- Only accepted `T:1` speed, `T:11` PWM and `T:13` ROS velocity commands replace
  the request and its acceptance timestamp. Accepted zeros also refresh it.
  UART, HTTP, ESP-NOW and mission steps all use the same JSON handler.
- The pure policy in `wheel_control_state.h` uses unsigned elapsed time across
  `millis()` rollover. After elapsed time exceeds the timeout, Beast/type 3
  enters zero-goal PID braking; types 1/2 disable PWM. Expiry stays latched until
  a fresh accepted motion command. Configuration updates first apply the old
  deadline, so increasing the timeout cannot revive an expired request.
- `T:115` emergency stop and `T:900` type reconfiguration request a hard stop,
  independent of PID braking, until a fresh motion command. These requests take
  effect at the next wheel service. A reset epoch preserves PID reset events
  even if stop and new motion both arrive between task samples. PID reentry
  transitions through manual mode to clear the previous integral and output.
- Motion is rejected if mutex/task creation fails; initialized motor outputs
  remain off. There is no fallback to a mission-dependent loop watchdog.
- Timeout `T:136` accepts positive, representable integer milliseconds. Zero,
  negative, fractional and missing settings preserve the previous timeout.
  The browser refreshes moving commands every 100 ms. A shorter custom timeout
  requires a matching producer cadence and transport margin.
- Speed/ROS requests must be finite, with the existing Beast wheel-speed bound
  of ±2 m/s. Direct PWM must have both finite numeric fields within ±255.
  Other base types map speed to PWM in double precision and preserve the legacy
  ±0.5 normalized endpoint (±256 before clamping) while rejecting larger scaled
  values. Accepted endpoints clamp to ±255 before acceptance or integer conversion.
  Rejected motion leaves the request and deadline unchanged.
- Wheel scales affect subsequent accepted speed/ROS commands, as in the prior
  implementation; changing scale does not refresh a motion deadline. PID gains
  are applied by the task. The legacy limits setting is retained, while PID
  output remains bounded to ±255.
- Feedback reads copy both measured wheel speeds and scale values under the
  mutex, then serialize outside it. `L`/`R` speed feedback is measured m/s for
  every type; the old non-Beast path intermittently overwrote those fields with
  PWM, making units depend on when feedback ran.
- Arm and gimbal sequencing remain in their existing command context.
  Unused, unexposed switch/light helpers that directly wrote motor pins were
  removed; the active switch-off command uses the shared stop request.

## Acceptance still required

An independent task closes the source-level blocking-mission gap. A nominal
service period is **not a physical stopping-time guarantee**: RTOS starvation,
interrupt/driver delays, power-stage behavior and braking on the actual robot
still need supervised acceptance. No robot was flashed, connected or operated
for this review. Do not treat a host test or successful compile as readiness to
return a robot to service.

## Reproducible build

This is an Arduino sketch. The reviewed compile uses Arduino CLI 1.5.1, board
`esp32:esp32:esp32`, ESP32 core **2.0.17** (the sketch uses the legacy LEDC API),
and the following Library Manager versions:

| Library | Version |
| --- | --- |
| ArduinoJson | 6.21.5 |
| Adafruit SSD1306 | 2.5.17 |
| Adafruit GFX Library | 1.12.6 |
| Adafruit BusIO | 1.17.4 |
| INA219_WE | 1.3.8 |
| ESP32Encoder | 0.11.7 |
| PID_v2 | 2.0.1 |
| SimpleKalmanFilter | 0.2.0 |
| Adafruit ICM20X | 2.0.7 |
| Adafruit Unified Sensor | 1.1.15 |

LittleFS, WiFi, WebServer, Wire and SPI come from the core. Use this repository's
`SCServo` library. New INA219_WE 1.4.1 uses renamed enum constants and fails in
unchanged battery code; pin 1.3.8 for this build. The deployed firmware/library
versions have not been established.

```sh
arduino-cli core install esp32:esp32@2.0.17 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install 'ArduinoJson@6.21.5' 'Adafruit SSD1306@2.5.17' 'Adafruit GFX Library@1.12.6' 'Adafruit BusIO@1.17.4' 'INA219_WE@1.3.8' 'ESP32Encoder@0.11.7' 'PID_v2@2.0.1' 'SimpleKalmanFilter@0.2.0' 'Adafruit ICM20X@2.0.7' 'Adafruit Unified Sensor@1.1.15'
arduino-cli compile --fqbn esp32:esp32:esp32 --library ./SCServo General_Driver
```

The core's esptool build hook also requires its Python dependencies (including
pyserial). The review used an isolated Python 3.12 environment; no upload command
was run.

## Flashing the UGV Beast

1. Power the robot **off**. Put it on blocks / a stand with wheels free.
2. Connect a USB data cable from your PC to the **USB Type-C port on the
   ESP32 lower-computer driver board** (the General Driver board inside the
   chassis — not the upper-computer Jetson/RPi port).
3. In Arduino IDE select the new `COM` port and upload at 115200 baud.
4. After flashing, confirm the OLED shows the boot sequence and the robot
   type is correct: send `{"T":900,"main":3,"module":2}` over serial
   (115200) if needed (`main:3` = UGV Beast).

## Supervised verification procedure

> Two people recommended: one at the host console, one ready at the robot's
> power switch.

1. **Bench (on blocks)**: wheels off the ground.
2. Open a serial monitor at 115200 to confirm the ESP32 is alive.
3. Start the host ROS driver and command motion (e.g. `T:13` velocity
   commands or the ROS teleop at low speed). Wheels spin.
4. **Kill the host driver hard**: `kill -9 <driver-pid>` (or unplug the USB
   serial cable). Do **not** shut it down cleanly — a clean shutdown may send
   a final zero command.
5. **Pass criterion**: the wheels stop within about 1 s of the last command.
   That 1 s figure is a provisional acceptance threshold. It is not a measured
   braking-time budget, and the 500 ms watchdog period is not a stopping-time
   guarantee. If they do not, power the robot off and do not proceed.
6. Restart the driver and confirm motion commands work again (the watchdog
   resets on the first new command), and that an explicit zero command stream
   (host alive, commanding stop) is *not* watchdog-stopped — the robot simply
   holds still.
7. **Ground test**: robot on the floor in an open area, minimum speed.
   Repeat the `kill -9` test. The robot must coast/brake to a stop within
   ~1 s and well under one wheel-rotation of travel.
8. Repeat on blocks with a stored mission that commands low wheel speed then
   blocks in `T:111` longer than the watchdog, including the `boot` mission path.
   Repeat during arm/gimbal commands that wait on servo feedback; wheels must
   stop while the command remains blocked. Test PWM and speed/ROS modes, each
   supported base type, emergency stop/reentry and timeout changes after expiry.
9. Only after all applicable acceptance checks pass, return the robot to service.

## Host regression checks

No serial device or hardware is used. With Python 3 and g++ installed, fetch
ArduinoJson v6.21.5 and PID_v2 2.0.1 outside the firmware tree, then run:

```sh
git clone --depth 1 --branch v6.21.5 https://github.com/bblanchon/ArduinoJson.git /tmp/ugv-arduinojson-6.21.5
# PID_v2 2.0.1 is installed by the build commands above.
ARDUINOJSON_INCLUDE=/tmp/ugv-arduinojson-6.21.5/src PID_LIBRARY="$HOME/Arduino/libraries/PID_v2" python3 -m unittest discover -s tests -v
```

The harness compiles the complete production wheel adapter, pure state policy,
actual motion/timeout JSON cases, real ArduinoJson and real PID_v2. GPIO, encoder,
clock and RTOS interfaces are inert host adapters. Fifteen native scenarios cover
invalid settings/motion, rollover, PWM bounds, scaled speed/ROS overrange rejection without deadline refresh, direct-PWM expiry on all base types,
zero-PWM followed by type changes, configuration after expiry,
emergency stop, PID integral clearing, stop/reentry and expiry/new-motion between samples, creation
failures, and an actual production task running while the host command context
sleeps past the timeout. A separate check covers browser cadence.

This checks the concurrency structure with a host thread; it does not reproduce
FreeRTOS scheduling, target driver timing, real encoder/physical braking, or the
actual deployed firmware. The full sketch compile is a separate check.

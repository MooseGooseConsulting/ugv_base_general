# Motion-Command Watchdog (host-silence stop)

## Hazard

The ESP32 lower computer latches the last received motion command forever.
If the host (ROS driver / upper computer) dies or the link drops mid-motion,
the robot keeps driving. This caused a documented wall collision on
2026-09-03 (MooseGooseConsulting UGV Beast).

## Design

This branch tightens and hardens the firmware's existing heartbeat into a
proper motion-command watchdog:

- `MOTION_CMD_WATCHDOG_MS` (`General_Driver/ugv_config.h`) — single named
  constant for the timeout, default **500 ms**.
- `lastCmdRecvTime` is refreshed by every **valid motion command** handled in
  `jsonCmdReceiveHandler()` (`General_Driver/uart_ctrl.h`):
  - `T:1` (`CMD_SPEED_CTRL`) — wheel speed
  - `T:11` (`CMD_PWM_INPUT`) — raw PWM
  - `T:13` (`CMD_ROS_CTRL`) — ROS velocity (m/s, rad/s)
  Explicit zero commands count as a refresh: a host actively sending zeros
  is alive and stopped, which is correct.
- `heartBeatCtrl()` (`General_Driver/movtion_module.h`) runs every `loop()`
  iteration in `General_Driver.ino`. If `millis() - lastCmdRecvTime` exceeds
  the timeout it calls `setGoalSpeed(0, 0)`:
  - UGV Beast (`mainType == 3`, PID mode): goal speeds are forced to zero and
    the PID loop actively brakes the wheels.
  - WAVE ROVER / UGV02 (`mainType` 1/2, direct PWM): PWM outputs are zeroed.
  The stop latches (`heartbeatStopFlag`) until the next motion command.
- **Armed from boot**: `lastCmdRecvTime` is initialized at power-on, so if no
  motion command ever arrives the outputs are forced to zero one timeout
  after boot. Default state is stopped.
- The timeout remains tunable at runtime with `{"T":136,"cmd":<ms>}`
  (`CMD_HEART_BEAT_SET`), e.g. raise it temporarily for bench debugging.

Because all command transports — UART/USB serial, the HTTP `/js` endpoint,
ESP-NOW JSON payloads, and stored mission steps — funnel through
`jsonCmdReceiveHandler()`, the watchdog covers every command source, not just
the ROS serial link.

### Behavioral notes / caveats

- The watchdog guards the **wheel/motor outputs only**. Arm (RoArm-M2 bus
  servos) and gimbal commands are unaffected and keep their own
  interpolation/stop logic.
- Stored missions (e.g. the `boot` mission) that issue motion steps refresh
  the watchdog while executing; motion is forced to zero one timeout after
  the mission's last motion step. This is intentional (fail-safe).
- ESP-NOW follower robots receiving wheel commands over ESP-NOW refresh the
  watchdog through the same JSON handler.
- `{"T":136,"cmd":0}` makes the watchdog fire immediately and continuously
  (motors can never run). Do not use `0`; the web page's documented example
  value of `0` should be avoided.

## Build

This is an **Arduino sketch** (not ESP-IDF / PlatformIO).

1. Install [Arduino IDE](https://www.arduino.cc/en/software) and the
   **esp32** board package by Espressif (Boards Manager).
2. Copy the `SCServo` folder from this repo into
   `C:\Users\<you>\AppData\Local\Arduino15\libraries\`
   (or `~/Arduino/libraries/`).
3. Install via Library Manager: **ArduinoJson**, **LittleFS**,
   **Adafruit_SSD1306**, **INA219_WE**, **ESP32Encoder**, **PID_v2**,
   **SimpleKalmanFilter**, **Adafruit_ICM20X**, **Adafruit_ICM20948**,
   **Adafruit_Sensor**.
4. Open `General_Driver/General_Driver.ino`, select board **ESP32 Dev
   Module**, and compile.

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
5. **Pass criterion**: the wheels stop within ~1 s of the last command
   (500 ms watchdog + PID braking). If they do not, power the robot off and
   do not proceed.
6. Restart the driver and confirm motion commands work again (the watchdog
   resets on the first new command), and that an explicit zero command stream
   (host alive, commanding stop) is *not* watchdog-stopped — the robot simply
   holds still.
7. **Ground test**: robot on the floor in an open area, minimum speed.
   Repeat the `kill -9` test. The robot must coast/brake to a stop within
   ~1 s and well under one wheel-rotation of travel.
8. Only then return the robot to normal service.

"""Host-only checks using production functions with ArduinoJson and inert PID/motor stubs.

These do not compile the Arduino sketch or prove real-time/physical stopping.
"""
from pathlib import Path
import re
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(r"(?:void|bool) " + name + r"\([^)]*\)\s*\{", source)
    assert match, name
    start = match.start()
    depth = 1
    end = match.end()
    while depth:
        if source[end] == "{": depth += 1
        if source[end] == "}": depth -= 1
        end += 1
    return source[start:end]


STUBS = r'''
#include <iostream>
#define CHECK(expr) do { if (!(expr)) { std::cerr << "failed: " << #expr << "\n"; return 1; } } while (0)
#include <cmath>
#include <climits>
#include <map>
#include <string>
using std::isfinite;
unsigned long clockMs = 1000;
unsigned long millis() { return clockMs; }
int mainType = 3, HEART_BEAT_DELAY = 500;
unsigned long lastCmdRecvTime = 100;
bool heartbeatStopFlag = true, usePIDCompute = false;
float setpointA = .5, setpointB = .5, setpointA_buffer = .5, setpointB_buffer = .5;
float spd_rate_A = 1, spd_rate_B = 1, TRACK_WIDTH = .5;
float leftOutput = .5, rightOutput = .5;
struct PID { void Setpoint(float) {} } pidA, pidB;
void leftCtrl(float value) { leftOutput = value; }
void rightCtrl(float value) { rightOutput = value; }
#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
#define CMD_SPEED_CTRL 1
#define CMD_PWM_INPUT 11
#define CMD_ROS_CTRL 13
#define CMD_HEART_BEAT_SET 136
'''

MAIN = r'''
int main(int argc, char** argv) {
  std::string test = argv[1];
  if (test == "timeout") {
    for (int invalid : {-1, 0, INT_MIN}) {
      changeHeartBeatDelay(invalid); CHECK(HEART_BEAT_DELAY == 500);
    }
    deserializeJson(jsonCmdReceive, R"({"T":136,"cmd":2147483648})");
    jsonCmdReceiveHandler(); CHECK(HEART_BEAT_DELAY == 500);
    jsonCmdReceive["cmd"] = 2.5;
    jsonCmdReceiveHandler(); CHECK(HEART_BEAT_DELAY == 500);
    jsonCmdReceive["cmd"] = 750;
    jsonCmdReceiveHandler(); CHECK(HEART_BEAT_DELAY == 750);
    clockMs = 2000; heartbeatStopFlag = false; heartBeatCtrl(); CHECK(setpointA == 0 && setpointB == 0);
  } else if (test == "speed") {
    deserializeJson(jsonCmdReceive, R"({"T":1,"L":3,"R":0.5})");
    for (int i=0;i<10;i++) { clockMs += 100; jsonCmdReceiveHandler(); }
    CHECK(lastCmdRecvTime == 100 && heartbeatStopFlag && !usePIDCompute);
    CHECK(setpointA == .5 && setpointB == .5);
    heartbeatStopFlag = false; heartBeatCtrl();
    CHECK(heartbeatStopFlag && setpointA == 0 && setpointB == 0);
    deserializeJson(jsonCmdReceive, R"({"T":1,"L":0,"R":0})");
    jsonCmdReceiveHandler(); CHECK(lastCmdRecvTime == clockMs && !heartbeatStopFlag);
    jsonCmdReceive["L"] = NAN;
    clockMs += 20; jsonCmdReceiveHandler(); CHECK(lastCmdRecvTime != clockMs);
  } else if (test == "ros") {
    deserializeJson(jsonCmdReceive, R"({"T":13,"X":3,"Z":0})");
    jsonCmdReceiveHandler();
    CHECK(lastCmdRecvTime == 100 && heartbeatStopFlag && !usePIDCompute);
    CHECK(setpointA == .5 && setpointB == .5);
    deserializeJson(jsonCmdReceive, R"({"T":13,"X":0,"Z":0})");
    jsonCmdReceiveHandler(); CHECK(lastCmdRecvTime == clockMs && !heartbeatStopFlag);
  } else if (test == "pwm") {
    deserializeJson(jsonCmdReceive, R"({"T":11,"L":0.5})");
    jsonCmdReceiveHandler(); CHECK(lastCmdRecvTime == 100 && heartbeatStopFlag);
    jsonCmdReceive["R"] = INFINITY;
    jsonCmdReceiveHandler(); CHECK(lastCmdRecvTime == 100 && heartbeatStopFlag);
    jsonCmdReceive["L"] = 0; jsonCmdReceive["R"] = 0;
    jsonCmdReceiveHandler();
    CHECK(lastCmdRecvTime == clockMs && !heartbeatStopFlag && !usePIDCompute);
    CHECK(leftOutput == 0 && rightOutput == 0);
  }
}
'''


class WatchdogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        include = Path(os.environ.get("ARDUINOJSON_INCLUDE", ""))
        if not (include / "ArduinoJson.h").is_file():
            raise RuntimeError("Set ARDUINOJSON_INCLUDE to the pinned ArduinoJson v6.21.5 src directory; see WATCHDOG.md")
        cls.tmp = tempfile.TemporaryDirectory()
        motion = (ROOT / "General_Driver/movtion_module.h").read_text()
        handler = (ROOT / "General_Driver/uart_ctrl.h").read_text()
        # Keep the actual motion and timeout cases; all unrelated handlers are excluded.
        motion_cases = handler[handler.index("case CMD_SPEED_CTRL:"):handler.index("case CMD_SET_MOTOR_PID:")]
        timeout_case = handler[handler.index("case CMD_HEART_BEAT_SET:"):handler.index("case CMD_GIMBAL_STEADY:")]
        cpp = STUBS + "\n".join(function(motion, name) for name in ["setGoalSpeed", "rosCtrl", "heartBeatCtrl", "changeHeartBeatDelay"])
        cpp += '\nvoid jsonCmdReceiveHandler(){ int cmdType = jsonCmdReceive["T"].as<int>(); switch(cmdType){' + motion_cases + timeout_case + '}}\n' + MAIN
        source = Path(cls.tmp.name) / "watchdog.cpp"
        source.write_text(cpp)
        cls.binary = Path(cls.tmp.name) / "watchdog"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-I", str(include), str(source), "-o", str(cls.binary)], check=True, capture_output=True, text=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_input_and_timeout_contract(self):
        for case in ["timeout", "speed", "ros", "pwm"]:
            with self.subTest(case=case):
                result = subprocess.run([str(self.binary), case], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_browser_heartbeat_is_inside_default_window(self):
        page = (ROOT / "General_Driver/web_page.h").read_text()
        interval = re.search(r"setInterval\(function\(\)\s*\{\s*heartBeat\(\);\s*\},\s*(\d+)\)", page)
        self.assertIsNotNone(interval)
        self.assertEqual(int(interval.group(1)), 100)
        self.assertNotIn('{"T":136,"cmd":0}', page)


if __name__ == "__main__":
    unittest.main()

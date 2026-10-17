"""Host-only checks using production functions with ArduinoJson and real PID_v2/ArduinoJson and inert hardware adapters.

These do not compile the Arduino sketch or prove real-time/physical stopping.
"""
from pathlib import Path
import re
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def handler_cases(handler, first, following, expected):
    """Fail clearly if handler edits change the isolated production cases."""
    positions = []
    for name in (first, following):
        matches = list(re.finditer(r"^\s*case\s+" + re.escape(name) + r"\s*:", handler, re.MULTILINE))
        if len(matches) != 1:
            raise RuntimeError(f"uart_ctrl.h: expected one case marker for {name}, found {len(matches)}")
        positions.append(matches[0].start())
    if positions[0] >= positions[1]:
        raise RuntimeError(f"uart_ctrl.h: {first} must precede {following}")
    block = handler[positions[0]:positions[1]]
    actual = re.findall(r"^\s*case\s+(CMD_\w+)\s*:", block, re.MULTILINE)
    if actual != expected:
        raise RuntimeError(f"uart_ctrl.h: expected cases {expected} before {following}, found {actual}")
    return block


class WatchdogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        include = Path(os.environ.get("ARDUINOJSON_INCLUDE", ""))
        if not (include / "ArduinoJson.h").is_file():
            raise RuntimeError("Set ARDUINOJSON_INCLUDE to the pinned ArduinoJson v6.21.5 src directory; see WATCHDOG.md")
        cls.tmp = tempfile.TemporaryDirectory()
        pid = Path(os.environ.get("PID_LIBRARY", ""))
        if not (pid / "PID_v2.cpp").is_file():
            raise RuntimeError("Set PID_LIBRARY to the PID_v2 2.0.1 directory; see WATCHDOG.md")
        handler = (ROOT / "General_Driver/uart_ctrl.h").read_text()
        # Keep the actual motion and timeout cases; all unrelated handlers are excluded.
        motion_cases = handler_cases(handler, "CMD_SPEED_CTRL", "CMD_SET_MOTOR_PID",
                                     ["CMD_SPEED_CTRL", "CMD_PWM_INPUT", "CMD_ROS_CTRL"])
        timeout_case = handler_cases(handler, "CMD_HEART_BEAT_SET", "CMD_GIMBAL_STEADY",
                                     ["CMD_HEART_BEAT_SET"])
        cpp = (ROOT / "tests/wheel_harness.cpp").read_text()
        cases = '\nvoid jsonCmdReceiveHandler(){ int cmdType = jsonCmdReceive["T"].as<int>(); switch(cmdType){' + motion_cases + timeout_case + '}}\n'
        marker = '// JSON_CASES inserted by the Python runner.'
        if cpp.count(marker) != 1:
            raise RuntimeError(f"wheel_harness.cpp: expected one JSON_CASES insertion marker, found {cpp.count(marker)}")
        cpp = cpp.replace(marker, cases)
        source = Path(cls.tmp.name) / "watchdog.cpp"
        source.write_text(cpp)
        cls.binary = Path(cls.tmp.name) / "watchdog"
        result = subprocess.run(["g++", "-I", str(pid), str(pid / "PID_v2.cpp"), "-std=c++11", "-pthread", "-Wall", "-Wextra", "-I", str(ROOT / "tests/support"), "-I", str(ROOT / "General_Driver"), "-I", str(include), str(source), "-o", str(cls.binary)], capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stderr)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_input_and_timeout_contract(self):
        for case in ["timeout", "pwm-timeout", "pwm-zero-type", "rejected-motion", "emergency", "rollover", "mapping", "scaled-overrange", "speed-endpoint", "task-failure", "mutex-failure", "brake-integral", "unsampled-stop", "unsampled-expiry", "blocked-mission"]:
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

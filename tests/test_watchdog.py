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
        motion_cases = handler[handler.index("case CMD_SPEED_CTRL:"):handler.index("case CMD_SET_MOTOR_PID:")]
        timeout_case = handler[handler.index("case CMD_HEART_BEAT_SET:"):handler.index("case CMD_GIMBAL_STEADY:")]
        cpp = (ROOT / "tests/wheel_harness.cpp").read_text()
        cases = '\nvoid jsonCmdReceiveHandler(){ int cmdType = jsonCmdReceive["T"].as<int>(); switch(cmdType){' + motion_cases + timeout_case + '}}\n'
        cpp = cpp.replace('// JSON_CASES inserted by the Python runner.', cases)
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
        for case in ["timeout", "pwm-timeout", "pwm-zero-type", "rejected-motion", "emergency", "rollover", "mapping", "scaled-overrange", "task-failure", "mutex-failure", "brake-integral", "unsampled-stop", "unsampled-expiry", "blocked-mission"]:
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

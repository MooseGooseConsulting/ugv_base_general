// Inert host hardware/library adapters; compile the complete production wheel
// adapter and actual JSON cases. No serial, networking, GPIO or robot access.
#include <iostream>
#include <cmath>
#include <climits>
#include <cfloat>
#include <string>
#include "freertos/FreeRTOS.h"
#include <ArduinoJson.h>
#define CHECK(expr) do { if (!(expr)) { std::cerr << "line " << __LINE__ << ": " << #expr << "\n"; return 1; } } while (0)
bool failMutex = false, failTask = false;
std::atomic<bool> stopTask{false};
std::thread wheelThread;
struct Cleanup { ~Cleanup() { stopTask = true; if (wheelThread.joinable()) wheelThread.join(); } } cleanup;
std::atomic<uint32_t> clockMs{0};
bool realClock = false;
const auto started = std::chrono::steady_clock::now();
unsigned long millis() { return realClock ? std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-started).count() : clockMs.load(); }
unsigned long micros() { return millis() * 1000; }
using byte = uint8_t;
using String = std::string;
constexpr int AIN1=0, AIN2=1, PWMA=2, BIN1=3, BIN2=4, PWMB=5;
constexpr int channel_A=0, channel_B=1, freq=20000, ANALOG_WRITE_BITS=8;
constexpr int OUTPUT=1, LOW=0, HIGH=1, AENCA=6, AENCB=7, BENCA=8, BENCB=9;
#define MOTION_CMD_WATCHDOG_MS 500
#define THRESHOLD_PWM 23
int mainType=3, moduleType=0, ONE_CIRCLE_PLUSES=1092;
float __kp=20, __ki=2000, __kd=0, windup_limits=255;
double WHEEL_D=.0523, TRACK_WIDTH=.141;
bool SET_MOTOR_DIR=true;
std::string screenLine_2;
std::atomic<int> duty[2], pins[6];
void pinMode(int,int) {}
void ledcSetup(int,int,int) {}
void ledcAttachPin(int,int) {}
void ledcWrite(int channel,int value) { duty[channel]=value; }
void digitalWrite(int pin,int value) { pins[pin]=value; }
struct ESP32Encoder {
  int64_t count=0;
  void attachHalfQuad(int,int) {}
  void setCount(int64_t value) { count=value; }
  int64_t getCount() { return count; }
};
#include <PID_v2.h>
#include "json_cmd.h"
StaticJsonDocument<512> jsonInfoHttp;
StaticJsonDocument<256> jsonCmdReceive;
struct { void println(const String&) {} } Serial;
#include "movtion_module.h"
// JSON_CASES inserted by the Python runner.

int main(int, char** argv) {
  using WheelControl::Mode;
  std::string test=argv[1];
  if (test=="mutex-failure") failMutex=true;
  movtionPinInit(); initEncoders();
  if (test=="mutex-failure") {
    CHECK(!startWheelControl()); CHECK(!setGoalSpeed(.5,.5));
    CHECK(duty[0]==0 && duty[1]==0); return 0;
  }
  if (test=="task-failure") {
    failTask=true;
    CHECK(!startWheelControl()); CHECK(!setGoalSpeed(.5,.5));
    CHECK(duty[0]==0 && duty[1]==0); return 0;
  }
  if (test=="blocked-mission") {
    realClock=true;
    CHECK(startWheelControl());
    CHECK(setGoalSpeed(.5,.5));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    CHECK(duty[0]>0 && duty[1]>0);
    // Main command context is blocked; only actual production task can expire.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    CHECK(duty[0]==0 && duty[1]==0);
    return 0;
  }
  // Deterministic service invocations, without starting the asynchronous task.
  WheelDriver::pidA.SetOutputLimits(-255,255); WheelDriver::pidB.SetOutputLimits(-255,255);
  WheelDriver::ready=true;
  WheelDriver::state.type(mainType, millis());
  if (test=="timeout") {
    for (const char* input : {R"({"T":136,"cmd":-1})",R"({"T":136,"cmd":0})",R"({"T":136,"cmd":2147483648})",R"({"T":136,"cmd":2.5})",R"({"T":136})"}) {
      deserializeJson(jsonCmdReceive,input); jsonCmdReceiveHandler();
      CHECK(WheelDriver::state.get().timeout==500);
    }
    clockMs=100; CHECK(setGoalSpeed(.5,.5));
    clockMs=601; changeHeartBeatDelay(10000);
    CHECK(WheelDriver::state.get().mode==Mode::Brake);
    CHECK(WheelDriver::state.get().timeout==10000);
    WheelDriver::service(); CHECK(duty[0]==0 && duty[1]==0);
  } else if (test=="pwm-timeout") {
    for (int type : {1,2,3}) {
      mm_settings(type,0);
      clockMs=100; CHECK(setWheelPwm(120,-90));
      WheelDriver::service(); CHECK(duty[0]==120 && duty[1]==90);
      clockMs=600; WheelDriver::service();
      CHECK(duty[0]==120 && duty[1]==90); // Inclusive timeout boundary.
      clockMs=601; WheelDriver::service();
      CHECK(duty[0]==0 && duty[1]==0);
      CHECK(WheelDriver::state.get().left==0 && WheelDriver::state.get().right==0);
      CHECK(WheelDriver::state.get().mode==(type==3 ? Mode::Brake : Mode::Stopped));
      clockMs=900; WheelDriver::service(); CHECK(duty[0]==0 && duty[1]==0);
    }
  } else if (test=="pwm-zero-type") {
    CHECK(setGoalSpeed(.5,.5)); WheelDriver::service();
    clockMs=100; WheelDriver::service(); CHECK(duty[0]>0);
    CHECK(setWheelPwm(0,0)); WheelDriver::service(); CHECK(duty[0]==0 && duty[1]==0);
    for (int type : {1,2,3}) {
      mm_settings(type,0);
      clockMs=clockMs+100; WheelDriver::service();
      CHECK(WheelDriver::state.get().left==0 && WheelDriver::state.get().right==0);
      CHECK(duty[0]==0 && duty[1]==0);
    }
  } else if (test=="rejected-motion") {
    clockMs=100; CHECK(setGoalSpeed(.5,.5));
    const auto before=WheelDriver::state.get();
    for (const char* input : {R"({"T":1,"L":3,"R":0.5})",R"({"T":1,"R":0.5})",R"({"T":13,"X":3,"Z":0})",R"({"T":11,"L":2})",R"({"T":11,"L":256,"R":0})",R"({"T":11,"L":"3","R":0})",R"({"T":11,"L":3.4028235e38,"R":0})",R"({"T":11,"L":-3.4028235e38,"R":0})"}) {
      clockMs=550; deserializeJson(jsonCmdReceive,input); jsonCmdReceiveHandler();
      CHECK(WheelDriver::state.get().mode==before.mode);
      CHECK(WheelDriver::state.get().left==before.left && WheelDriver::state.get().right==before.right);
      CHECK(WheelDriver::state.get().resetEpoch==before.resetEpoch);
      CHECK(WheelDriver::state.get().acceptedAt==100);
    }
    CHECK(!setGoalSpeed(NAN,0)); CHECK(!rosCtrl(0,INFINITY));
    clockMs=601; WheelDriver::service(); CHECK(WheelDriver::state.get().mode==Mode::Brake);
    deserializeJson(jsonCmdReceive,R"({"T":1,"L":0,"R":0})"); jsonCmdReceiveHandler();
    CHECK(WheelDriver::state.get().mode==Mode::Speed && WheelDriver::state.get().acceptedAt==601);
  } else if (test=="emergency") {
    CHECK(setGoalSpeed(.5,.5)); WheelDriver::service();
    clockMs=100; WheelDriver::service(); CHECK(duty[0]>0);
    switchEmergencyStop(); WheelDriver::service(); CHECK(duty[0]==0 && duty[1]==0);
    clockMs=200; setPID(30,2000,0,255); setSpdRate(.5,.5); changeHeartBeatDelay(900);
    WheelDriver::service(); CHECK(duty[0]==0 && WheelDriver::state.get().mode==Mode::Stopped);
    CHECK(setWheelPwm(80,-80)); WheelDriver::service(); CHECK(duty[0]==80 && duty[1]==80);
    mm_settings(1,0); WheelDriver::service(); CHECK(duty[0]==0 && duty[1]==0);
  } else if (test=="unsampled-expiry") {
    CHECK(setGoalSpeed(.5,.5)); WheelDriver::service();
    clockMs=100; WheelDriver::service();
    clockMs=200; WheelDriver::service();
    clockMs=500; WheelDriver::service(); CHECK(duty[0]>0);
    clockMs=501; CHECK(setGoalSpeed(-.5,-.5));
    WheelDriver::service(); CHECK(duty[0]==0 && duty[1]==0);
    clockMs=600; WheelDriver::service(); CHECK(duty[0]>0 && pins[AIN1]==HIGH);
  } else if (test=="unsampled-stop") {
    CHECK(setGoalSpeed(.5,.5)); WheelDriver::service();
    clockMs=100; WheelDriver::service(); CHECK(duty[0]>0);
    clockMs=101; switchEmergencyStop(); CHECK(setGoalSpeed(-.5,-.5));
    WheelDriver::service(); CHECK(duty[0]==0 && duty[1]==0);
    clockMs=200; WheelDriver::service(); CHECK(duty[0]>0 && pins[AIN1]==HIGH);
    clockMs=201; CHECK(setWheelPwm(100,100)); CHECK(setGoalSpeed(.5,.5));
    WheelDriver::service(); CHECK(duty[0]==0 && duty[1]==0);
    clockMs=300; WheelDriver::service(); CHECK(duty[0]>0 && pins[AIN1]==LOW);
    clockMs=301; mm_settings(3,0); CHECK(setGoalSpeed(-.5,-.5));
    WheelDriver::service(); CHECK(duty[0]==0 && duty[1]==0);
  } else if (test=="brake-integral") {
    CHECK(setGoalSpeed(.5,.5)); WheelDriver::service();
    clockMs=100; WheelDriver::service();
    clockMs=200; WheelDriver::service(); CHECK(duty[0]>0 && pins[AIN1]==LOW);
    // Forward movement at timeout while prior positive integral is saturated.
    WheelDriver::encoderA.count=-500; WheelDriver::encoderB.count=-500;
    clockMs=501; WheelDriver::service();
    CHECK(WheelDriver::state.get().mode==Mode::Brake);
    CHECK(duty[0]>0 && pins[AIN1]==HIGH); // Torque opposes measured forward motion.
    CHECK(duty[1]>0 && pins[BIN1]==HIGH);
  } else if (test=="rollover") {
    clockMs=UINT32_MAX-200; CHECK(setWheelPwm(100,100));
    clockMs=299; WheelDriver::service(); CHECK(WheelDriver::state.get().mode==Mode::Pwm);
    clockMs=300; WheelDriver::service(); CHECK(WheelDriver::state.get().mode==Mode::Brake);
  } else if (test=="scaled-overrange") {
    for (int type : {1,2}) {
      mm_settings(type,0); setSpdRate(.5,1);
      clockMs=100; CHECK(setGoalSpeed(.2,.2)); WheelDriver::service();
      const auto before=WheelDriver::state.get();
      for (const char* input : {R"({"T":1,"L":1,"R":0})", R"({"T":1,"L":0,"R":-0.5})", R"({"T":1,"L":3.4028235e38,"R":0})", R"({"T":13,"X":1,"Z":0})"}) {
        clockMs=550; deserializeJson(jsonCmdReceive,input); jsonCmdReceiveHandler();
        const auto after=WheelDriver::state.get();
        CHECK(after.acceptedAt==100 && after.left==before.left && after.right==before.right);
        CHECK(after.mode==before.mode && after.resetEpoch==before.resetEpoch);
      }
      clockMs=601; WheelDriver::service();
      CHECK(WheelDriver::state.get().mode==Mode::Stopped);
      CHECK(duty[0]==0 && duty[1]==0);
    }
  } else if (test=="mapping") {
    mm_settings(1,0); setSpdRate(.5,1);
    CHECK(!setGoalSpeed(FLT_MAX,-FLT_MAX));
    CHECK(setGoalSpeed(255.0f/256,-255.0f/512)); WheelDriver::service();
    CHECK(duty[0]==255 && duty[1]==255 && pins[AIN1]==HIGH && pins[BIN1]==LOW);
    CHECK(setGoalSpeed(.2,.2)); const auto goals=WheelDriver::state.get();
    setSpdRate(1,.1); CHECK(WheelDriver::state.get().left==goals.left && WheelDriver::state.get().right==goals.right);
    CHECK(!setGoalSpeed(INFINITY,0)); CHECK(!setWheelPwm(-256,0));
    mm_settings(3,0); CHECK(rosCtrl(0,2));
    CHECK(std::abs(WheelDriver::state.get().left + .141)<1e-6);
    clockMs=501; setSpdRate(1,1); CHECK(WheelDriver::state.get().mode==Mode::Brake);
    CHECK(!rosCtrl(FLT_MAX,FLT_MAX));
  }
  return 0;
}

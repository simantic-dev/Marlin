#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"

// Pins and registers: see PINS.md.
#define PIN_A 4
#define PIN_B 5
#define PIN_C 6
#define PIN_CS 10
#define PIN_STEP 2
#define PIN_DIR 3
#ifndef FF_ALPHA
#define FF_ALPHA 0.5f  // one-pole filter on the 1 ms step-rate estimate; 1 = unfiltered
#endif
#define STEPS_PER_REV 3200  // Marlin X: 80 steps/mm, GT2 20T pulley = 40 mm/rev

// Nanotec DB42S02 (NEMA17 BLDC), datasheet rev 6: 8 poles, delta 0.19 ohm line-line -> 0.095 ohm star-equivalent.
struct Axis {
  MagneticSensorMT6835 sensor;
  BLDCMotor motor;
  BLDCDriver3PWM driver;
  StepDirListener sd;
  uint32_t loops = 0;
  float maxerr = 0;
  Axis(int a, int b, int c, int cs, int step, int dir)
      : sensor(cs), motor(4, 0.095), driver(a, b, c), sd(step, dir, _2PI / STEPS_PER_REV) {}

  void setup(void (*onStep)()) {
    sensor.init();
    motor.linkSensor(&sensor);
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);
    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = 6;
    motor.current_limit = 10.8;  // DB42S02 peak current
    motor.voltage_sensor_align = 1.0;  // ~10.5 A into 0.095 ohm, within the peak
    motor.velocity_limit = 60;   // rad/s: 380 mm/s at 40 mm/rev
    // Servo tuning for rotor + belt + X carriage (2.4e-6 + 0.37 kg x r^2 = 1.74e-5 kg m^2): velocity loop near 128 Hz,
    // below the ~260 Hz belt resonance; angle loop P = 200 1/s.
    motor.PID_velocity.P = 1.0;
    motor.PID_velocity.I = 20;
    motor.LPF_velocity.Tf = 0.0005;
    motor.P_angle.P = 200;
#ifdef LOOP_FIX
    // With feedforward the loop sees big transients (homing reverses instantly at the endstop). The 60 rad/s clamp and
    // SimpleFOC's default 1000/s output ramp then saturate it into a ~20 Hz limit cycle, so: clamp at 200 rad/s (back-EMF
    // at 6 V is ~430 rad/s for Kt 0.014) and no ramp.
    motor.velocity_limit = 200;
    motor.PID_velocity.output_ramp = 0;
#endif
    motor.init();
    Serial.println(motor.initFOC() ? "FOC init OK" : "FOC init FAILED");
    sd.init();
    sd.enableInterrupt(onStep);
    sd.attach(&motor.target);
    motor.target = motor.shaft_angle;
    tf = micros();
    last = motor.target;
  }

  void loop() {
    motor.loopFOC();
    motor.move();
    loops++;
#ifdef VEL_FF
    // Velocity feedforward: the STEP rate (target change per ms, one-pole filtered) goes straight into the velocity
    // loop, so the angle loop only corrects the residual instead of lagging by v / P_angle.
    uint32_t now = micros();
    if (now - tf >= 1000) {
      float v = (motor.target - last) * 1e6f / (now - tf);
      motor.feed_forward_velocity += FF_ALPHA * (v - motor.feed_forward_velocity);
      last = motor.target;
      tf = now;
    }
#endif
    if (motor.target != 0) maxerr = max(maxerr, fabsf(motor.target - motor.shaft_angle));
  }
  uint32_t tf = 0;
  float last = 0;
};

Axis x(PIN_A, PIN_B, PIN_C, PIN_CS, PIN_STEP, PIN_DIR);
void onStepX() { x.sd.handle(); }
#ifdef DUAL_AXIS
Axis y(22, 23, 8, 9, 20, 21);  // see PINS.md
void onStepY() { y.sd.handle(); }
#endif

void setup() {
  Serial.begin(115200);
  SPI.begin();
  x.setup(onStepX);  // the encoders align one after another
#ifdef DUAL_AXIS
  y.setup(onStepY);
#endif
  Serial.println("Ready");
}

void report(const char *name, Axis &a, uint32_t dt_ms) {
  Serial.printf(" %s target=%.4f angle=%.4f maxerr=%.4f loop_hz=%lu", name, a.motor.target, a.motor.shaft_angle, a.maxerr,
                a.loops * 1000 / dt_ms);
  a.loops = 0;
}

void loop() {
  x.loop();
#ifdef DUAL_AXIS
  y.loop();
#endif
  static uint32_t t;
  uint32_t now = millis();
  if (now - t >= 100) {
    Serial.printf("t=%lu", now);
    report("x", x, now - t);
#ifdef DUAL_AXIS
    report("y", y, now - t);
#endif
    Serial.println();
    t = now;
  }
}

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

MagneticSensorMT6835 sensor(PIN_CS);
// Nanotec DB42S02 (NEMA17 BLDC), datasheet rev 6: 8 poles, delta 0.19 ohm line-line -> 0.095 ohm star-equivalent.
BLDCMotor motor(4, 0.095);
BLDCDriver3PWM driver(PIN_A, PIN_B, PIN_C);
StepDirListener sd(PIN_STEP, PIN_DIR, _2PI / STEPS_PER_REV);
void onStep() { sd.handle(); }

void setup() {
  Serial.begin(115200);
  SPI.begin();
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
  Serial.println("Ready");
}

void loop() {
  motor.loopFOC();
  motor.move();
#ifdef VEL_FF
  // Velocity feedforward: the STEP rate (target change per ms, one-pole filtered) goes straight into the velocity
  // loop, so the angle loop only corrects the residual instead of lagging by v / P_angle.
  static uint32_t tf = micros();
  static float last = motor.target;
  uint32_t now = micros();
  if (now - tf >= 1000) {
    float v = (motor.target - last) * 1e6f / (now - tf);
    motor.feed_forward_velocity += FF_ALPHA * (v - motor.feed_forward_velocity);
    last = motor.target;
    tf = now;
  }
#endif
  static uint32_t t;
  static float maxerr;
  if (motor.target != 0) maxerr = max(maxerr, fabsf(motor.target - motor.shaft_angle));
  if (millis() - t >= 100) {
    t = millis();
    Serial.printf("t=%lu target=%.4f angle=%.4f maxerr=%.4f\n", t, motor.target, motor.shaft_angle, maxerr);
  }
}

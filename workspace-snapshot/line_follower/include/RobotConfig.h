#pragma once

// All existing GPIO assignments stay in main.cpp. No rewiring for this version.
namespace cfg {
constexpr bool A_IS_RIGHT = false; // User verified: A test moved left, then B right.
constexpr bool A_FORWARD_IN1_HIGH = true;
constexpr bool B_FORWARD_IN1_HIGH = true;
constexpr bool MOTOR_DIRECTIONS_VERIFIED = true; // User confirmed both move forward.
constexpr bool CHANNEL_1_IS_LEFT = true; // User confirmed probe 1 is on vehicle's left.
constexpr bool SENSOR_ORDER_VERIFIED = true;

// E1/E2 mapping and count signs must be measured, not inferred from A/B names.
constexpr bool E1_IS_RIGHT = true;
constexpr int E1_FORWARD_SIGN = 1;
constexpr int E2_FORWARD_SIGN = 1;
// Leave IRQs off until encoder outputs are powered and have stable 3.3V logic.
// Otherwise floating input pins can continuously interrupt button handling.
constexpr bool ENCODER_INPUTS_ENABLED = false;
constexpr bool ENCODERS_VERIFIED = false;
constexpr float LEFT_MM_PER_COUNT = 0.0f;  // pi * wheel diameter / x4 counts/rev
constexpr float RIGHT_MM_PER_COUNT = 0.0f;
constexpr float WHEEL_DIAMETER_MM = 65.0f; // User measurement, circumference ~204.20mm
constexpr float TRACK_MM = 120.0f;         // User estimate ONLY; remeasure before unlocking

// Full A->B and four-leg route remains locked until wheel calibration and
// the A starting pose/corner direction have been checked on the real track.
constexpr bool ROUTE_START_POSE_VERIFIED = false;
constexpr int ROUTE_TURN_SIGN = 0; // +1 left, -1 right; 0 deliberately locks
constexpr float ROUTE_SIDE_MM = 1000.0f;

// Optional inner wheel-speed PI/PID. Keep disabled until wheel encoder
// identity, sign, counts/rev, and a safe feedforward PWM are measured.
constexpr bool SPEED_PID_ENABLED = false;
constexpr float SPEED_KP = 0.0f;   // PWM per (mm/s)
constexpr float SPEED_KI = 0.0f;   // PWM per mm
constexpr float SPEED_KD = 0.0f;
constexpr float SPEED_FEEDFORWARD_PWM = 0.0f;
constexpr float SPEED_BASE_MMPS = 0.0f;
constexpr float SPEED_LINE_CORRECTION_MMPS = 0.0f;

// Optional gray line-following steering loop (include/LinePid.h). When
// disabled the Follow/Route states use the original fixed proportional term;
// when enabled they use LinePid with these gains. Follow with speed PID scales
// output / LINE_MAX_CORRECTION by SPEED_LINE_CORRECTION_MMPS into mm/s.
// Keep disabled until the
// sensor order and steering gains are tuned on the real track.
constexpr bool LINE_PID_ENABLED = false;
constexpr float LINE_KP = 0.0f;   // steering PWM per unit line error
constexpr float LINE_KI = 0.0f;   // normally 0: integral can overshoot in turns
constexpr float LINE_KD = 0.0f;   // steering PWM per unit error/second
constexpr float LINE_D_FILTER_SECONDS = 0.04f;
constexpr float LINE_MAX_CORRECTION = 45.0f; // output clamp; matches old K
constexpr float LINE_INTEGRAL_LIMIT = 30.0f;

// Parking starts at a marked, measured axle pose beside the bay, not at A.
// +1: bay on vehicle's right; -1: on its left. Verify against travel direction.
constexpr int BAY_SIDE = -1; // Printed A->B->C->D layout: interior on vehicle LEFT.
constexpr bool PARKING_START_POSE_VERIFIED = false;
constexpr float REVERSE_DEPTH_MM = 0.0f;    // Axle travel AFTER turning away 90 deg
constexpr float PARALLEL_RADIUS_MM = 0.0f;  // Axle midpoint arc radius > TRACK_MM/2
constexpr float PARALLEL_ANGLE_DEG = 0.0f;  // Each reverse arc, normally <= 90 deg
constexpr float PARALLEL_TRIM_MM = 0.0f;    // Optional final straight; +forward
} // namespace cfg

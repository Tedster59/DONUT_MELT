#ifndef DONUT_TRACTION_H
#define DONUT_TRACTION_H

#include "donut_config.h"

// Traction control for melty spin-up (3lb, TRACTION_CONTROL).
//
// Compares how fast the wheels are turning (bidirectional DShot eRPM from the ESCs) with how fast
// the bot is turning, and caps the base melty throttle so the wheels stay at most TC_TARGET_SLIP
// ahead of the bot instead of burning out.
//
//   wheel_rpm (bot-equivalent) = max motor rpm / TC_MOTOR_RPM_PER_BOT_RPM
//   reference rpm = fast accelerometer rpm, or the launch ramp while that is under TC_ACCEL_TRUST_RPM
//   slip = (wheel_rpm - reference) / max(reference, TC_MIN_REF_RPM)
//
// The launch ramp covers the first few hundred RPM, where the accelerometers can't measure the
// bot's speed well enough (one 400g LSB is ~90 RPM at their 2 cm spacing): the reference rises at
// TC_LAUNCH_RPM_PER_S, i.e. how fast the bot can spin up on grip, until the accelerometer reading
// takes over.
//
// While traction control can act (enabled, calibrated, ESC telemetry fresh), the left stick goes
// straight to the throttle and traction control is the only limit. Otherwise the old
// THROTTLE_PC_P ramp is used, like before traction control existed.
//
// CH9 (momentary) toggles traction control on/off for back-to-back comparisons. The state shows on
// the transmitter's SWITCH_F middle page as the decimal of the drive mode (1.3 = melty, TC state 3).

// TC_STATE_* (logged and shown on the transmitter)
#define TC_STATE_OFF 0            // switched off with CH9 (or TRACTION_CONTROL not defined)
#define TC_STATE_UNCALIBRATED 1   // TC_MOTOR_RPM_PER_BOT_RPM is 0: logging only, never limits
#define TC_STATE_NO_TELEMETRY 2   // ESC eRPM missing or stale: not limiting, old throttle ramp
#define TC_STATE_WATCHING 3       // running, slip under target, throttle not capped
#define TC_STATE_LIMITING 4       // throttle is being capped

// Call every main loop while in melty mode with throttle applied. Takes the base melty throttle
// (-1..1, before translation) and returns it capped. Also reads the accelerometers at
// TC_ACCEL_INTERVAL_US and logs at TC_LOG_INTERVAL_US.
double traction_limit_throttle(double throttle);

// Call when the throttle is zero, the bot isn't live or not in melty: the next spin-up starts a
// fresh launch ramp with the cap fully open.
void traction_reset(void);

// true when the drive should skip the THROTTLE_PC_P ramp and let traction control limit instead
bool traction_controls_throttle(void);

// Call every main loop: handles the CH9 on/off toggle.
void traction_update_switch(bot_state_t* bot_state);

uint8_t traction_state(void);

#endif

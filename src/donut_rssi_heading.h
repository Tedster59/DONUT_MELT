#ifndef DONUT_RSSI_HEADING_H
#define DONUT_RSSI_HEADING_H

#include "donut_config.h"

// Phase-locked heading from the RSSI pattern (RSSI_HEADING).
//
// The bot's antenna gives the uplink RSSI a fixed pattern over each rotation: strongest and weakest
// at the same bot angles relative to the transmitter. Timing the heading from rpm alone integrates
// any rpm error into a heading that keeps rotating (1% at 2000 rpm = one turn every 3 s). This
// tracks the rotation ANGLE instead:
//
//   phase(t) = phase_ref + f * (t - t_ref)          [turns]
//
// Every RSSI_HEADING_UPDATE_US it takes the last RSSI_HEADING_WINDOW_US of RSSI samples, computes
// their harmonics against that phase, and matches them to a learned template of the pattern to
// find how far the model is off. The speed f comes from the corrected accel rpm times a scale
// factor, so speed changes (translation, spin-up) are followed at once; the measured phase error
// corrects the phase (RSSI_HEADING_KP) and slowly the scale (RSSI_HEADING_KS). The template's
// fundamental is pinned to phase 0, which ties the heading frame to the antenna pattern, so speed
// errors can't make it creep.
//
// "Forward" = the pattern frame plus an offset per spin direction. The right stick X moves that
// offset (same feel as the rpm-based trim). The first time it locks in a direction the offset is
// set so the heading doesn't jump from where the rpm-based timing had it.
//
// Offline replays: third_spin.csv (steady ~2630 rpm) locked 93%, 10.5 deg spread per update;
// heading_shadow.csv (2000 rpm with translation dips) 89%, 10.8 deg; spinup_two.csv (spin-up)
// 58%. It only takes over while locked and hands back to the rpm timing otherwise.

#define RSSI_HEADING_IDLE 0       // no usable RSSI rpm to start from
#define RSSI_HEADING_ACQUIRING 1  // tracking, not trusted yet
#define RSSI_HEADING_LOCKED 2     // trusted (drives the heading with RSSI_HEADING_DRIVE)

void rssi_heading_init(bot_state_t* user_bot_state);

// one RSSI sample (called from the link stats callback, same timestamps as rssi_rpm_add_sample)
void rssi_heading_add_sample(uint64_t timestamp_us, uint8_t rssi);

// call every main loop; does a bounded slice of work per call
void rssi_heading_update(void);

// For handle_spin. When locked (and RSSI_HEADING_DRIVE is defined) fills the rotation period and
// how far into the current rotation the bot is, like the rpm-based timing, and returns true.
// rpm_timing_fraction = where the rpm-based timing has the bot (0..1), used once per direction so
// taking over doesn't jump the heading. right_x_percent trims the heading like it trims the rpm.
bool rssi_heading_timing(uint64_t now_us, double right_x_percent, double rpm_timing_fraction,
                         double* us_per_rotation, double* elapsed_us);

uint8_t rssi_heading_state(void);
double rssi_heading_rpm(void);

#endif

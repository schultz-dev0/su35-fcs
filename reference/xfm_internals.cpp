// Builds the mod's flight model (su35_xfm.cpp) unchanged into a shared library for the reference harness, and adds
// a few extra entry points that read or set its internal state. The mod source is included from its own folder at
// build time (see build_reference.m); nothing from it is copied into this repo.
//
// It is included, not linked, so the helpers below can see the flight model's file-local globals. The flight model
// is not edited.
#include "windows.h"

#ifndef XFM_MAIN_SOURCE
#error "build_reference.m must define XFM_MAIN_SOURCE as the quoted path of su35_xfm.cpp"
#endif
#include XFM_MAIN_SOURCE

#define XFM_REF_EXPORT extern "C" __attribute__((visibility("default")))

// Points the flight model at its config folder (SU35XFM.cfg) and its log folder. Call before ed_fm_configure.
XFM_REF_EXPORT void xfm_ref_set_folders(const char *config_dir, const char *run_dir)
{
    shim::config_dir() = config_dir;
    shim::run_dir() = run_dir;
}

// Air start state the DCS start routines cannot give: gear already down, and the first-frame check that would
// otherwise pull the gear up again switched off. Call after ed_fm_hot_start_in_air().
XFM_REF_EXPORT void xfm_ref_start_gear_down(int gear_down)
{
    gear_cmd = gear_down ? 1 : 0;
    gear_pos = gear_cmd;
    start_pending = false;
}

// Engine state at the air start. ed_fm_hot_start_in_air() leaves both engines at 90 % rpm with no afterburner; a
// replay that starts in full afterburner needs the engines already there, or the first seconds lose thrust.
// Call after ed_fm_hot_start_in_air(). Index 0 is the left engine.
XFM_REF_EXPORT void xfm_ref_set_engines(double rpm_left, double rpm_right, double ab_left, double ab_right,
                                         double thrust_left, double thrust_right)
{
    eng[0].rpm = rpm_left;
    eng[1].rpm = rpm_right;
    eng[0].ab = ab_left;
    eng[1].ab = ab_right;
    eng[0].thrust = thrust_left;
    eng[1].thrust = thrust_right;
}

// Switches the mod reads from cockpit arguments, which do not exist here. Set them before every ed_fm_simulate.
// probe_out holds the refuelling probe out (the probe motion itself follows the cockpit, so it only moves the
// probe position the flight model reports).
XFM_REF_EXPORT void xfm_ref_set_switches(int tvc_enabled, int probe_out)
{
    tvc_on = tvc_enabled != 0;
    if (probe_out) probe_pos = 1;
}

// Number of values written by xfm_ref_get_internals. The order is listed in xfm_sfun.cpp and the README.
XFM_REF_EXPORT int xfm_ref_internal_count() { return 28; }

// The flight model's own state after a step: actuator positions (deg), thrust (N), positions (0..1).
XFM_REF_EXPORT void xfm_ref_get_internals(double *out)
{
    int i = 0;
    out[i++] = stab;            // symmetric stabilator, deg, + trailing edge down
    out[i++] = stab_diff;
    out[i++] = flaperon;
    out[i++] = rudder;
    out[i++] = le_flap;
    out[i++] = noz_pitch;
    out[i++] = noz_yaw;
    out[i++] = noz_roll;
    out[i++] = eng[0].thrust;
    out[i++] = eng[1].thrust;
    out[i++] = eng[0].rpm;
    out[i++] = eng[1].rpm;
    out[i++] = eng[0].ab;
    out[i++] = eng[1].ab;
    out[i++] = gear_pos;
    out[i++] = flap_pos;
    out[i++] = airbrake_pos;
    out[i++] = flap_brake_pos;
    out[i++] = probe_pos;
    out[i++] = internal_fuel;
    out[i++] = rec.q_dem;
    out[i++] = rec.p_dem;
    out[i++] = rec.yaw_right_dem;
    out[i++] = rec.man_w;
    out[i++] = rec.post_w;
    out[i++] = tvc_on ? 1.0 : 0.0;
    out[i++] = limiter_off ? 1.0 : 0.0;
    out[i++] = (double)ap_mode;
}

function slow = plant_slow_step(P, slow, cmd, st, dt)
% PLANT_SLOW_STEP  Gear, landing flaps and speed brake positions (cpp:4086-4184).
%   slow   struct: gear_pos, flap_pos, flap_brake_pos, airbrake_pos (0..1), airbrake_on_s (s), brake_inhibited (0/1)
%   cmd    struct: gear (0/1, 1 = down), airbrake (0/1)
%   st     struct: mach, alpha, nz, qbar
% Speed brake level 1 only (flaperons down). Level 2 (rudders out, from the РУД rocker) is not rebuilt, so
% airbrake_pos stays 0. The landing flaps follow the gear lever (flap_cmd is 0: no flap switch input).
C = P.act;
dt_gear = dt / C.gear_travel_s;
slow.gear_pos = slow.gear_pos + min(max(cmd.gear - slow.gear_pos, -dt_gear), dt_gear);
flap_target = double(cmd.gear > 0.5);
dt_flap = dt / C.flap_travel_s;
slow.flap_pos = slow.flap_pos + min(max(flap_target - slow.flap_pos, -dt_flap), dt_flap);

if cmd.airbrake > 0.5
    slow.airbrake_on_s = slow.airbrake_on_s + dt;
else
    slow.airbrake_on_s = 0;
end
% patent inhibits, with a little hysteresis so a jet flying on a limit doesn't chatter
inhibited = slow.brake_inhibited > 0.5;
mach_margin = C.ab_hyst_mach * double(inhibited);
aoa_margin = C.ab_hyst_aoa * double(inhibited);
g_margin = C.ab_hyst_g * double(inhibited);
mach_ok = st.mach < C.ab_mach_max - mach_margin;
aoa_ok = st.alpha > C.ab_aoa_min + aoa_margin && st.alpha < C.ab_aoa_max - aoa_margin;
g_ok = abs(st.nz - 1) < C.ab_nz_band - g_margin;
allowed = mach_ok && aoa_ok && g_ok;
slow.brake_inhibited = double(~allowed);
deflection = min(max(C.ab_q_ref / max(st.qbar, 1), C.ab_q_min_frac), 1);
flap_brake_target = 0;
if allowed && slow.airbrake_on_s >= C.ab_level1_s
    flap_brake_target = deflection;
end
dt_brake = dt / C.brake_flap_travel_s;
slow.flap_brake_pos = slow.flap_brake_pos + min(max(flap_brake_target - slow.flap_brake_pos, -dt_brake), dt_brake);
dt_rudder = dt / C.brake_rudder_travel_s;
slow.airbrake_pos = slow.airbrake_pos + min(max(0 - slow.airbrake_pos, -dt_rudder), dt_rudder);

end

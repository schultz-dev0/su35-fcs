function [force, moment, nz, ny_lat] = plant_forces_moments(P, st, cfg, act, aux, thrust_left, thrust_right)
% PLANT_FORCES_MOMENTS  Forces and moments about the centre of gravity, DCS body axes (cpp:3924-4034).
% Forces exclude gravity. Moments: x roll (+ right wing down), y yaw (+ nose LEFT), z pitch (+ nose up).
%   st    struct: V, alpha, beta, p, q, r_yaw, mach, qbar, ias_kmh, height (over ground), mass
%   cfg   struct: gear_pos, airbrake_pos, flap_pos, flap_brake_pos
%   act   struct, the actuators' positions in the flight model's own signs (rad): stab (+ = nose up moment), flaperon,
%         rudder, noz_pitch, noz_yaw, noz_roll
%   aux   struct from the FCS allocation: tvc_ok, man_w, yaw_arm (m), see fcs_allocation
% Not rebuilt: random buffet jitter, wing flex, ground contact and ground damper, lift dump, damage, drag chute.
A = P.aero;
L = P.alloc;
S = P.geom.S;
b = P.geom.b;
c = P.geom.c;
V = max(st.V, 30);
p_hat = st.p * b / (2 * V);
q_hat = st.q * c / (2 * V);
r_hat = -st.r_yaw * b / (2 * V);
ge = plant_ground_effect(P, st.height);
a0 = plant_aero_bare(P, st.alpha, st.beta, st.mach, p_hat, q_hat, r_hat, cfg, ge);

thrust = thrust_left + thrust_right;
thrust_asym = (thrust_left - thrust_right) * L.asym_arm;       % left stronger: nose right

% control power and the stabilator trim the transonic tuck needs (not charged as deflection drag)
stab_low_speed = 1 + A.stab_low_speed_gain * min(max((A.stab_low_speed_kmh - st.ias_kmh) / A.stab_low_speed_width_kmh, 0), 1);
Cm_d = A.Cm_d * plant_eff(P, st.mach, st.alpha) * stab_low_speed;
Cl_d = A.Cl_d * plant_eff_lat(P, st.mach, st.alpha);
Cn_d = A.Cn_d * plant_eff_lat(P, st.mach, st.alpha);
cm_extra = plant_cm_extra(P, st.mach, st.alpha, cfg.airbrake_pos);
if Cm_d > 1e-6
    stab_tuck = -L.cm_extra_feed * cm_extra / Cm_d;
else
    stab_tuck = 0;
end

store_dcd = 0;
if P.mass.store_kg > 0
    mach_wave = min(max((st.mach - 0.85) / 0.35, 0), 1);
    store_dcd = P.mass.store_cd_per_kg * P.mass.store_kg * (1 + P.mass.store_wave * mach_wave);
end
CL = a0.CL + A.flap_cl * cfg.flap_pos + A.flap_brake_cl * cfg.flap_brake_pos;
CD = a0.CD + A.cd_stab * abs(act.stab - stab_tuck) + A.cd_flaperon * abs(act.flaperon) + A.cd_rudder * abs(act.rudder) + store_dcd;
lift = CL * st.qbar * S;
drag = CD * st.qbar * S;
side = a0.CY * st.qbar * S;

ca = cos(st.alpha);
sa = sin(st.alpha);
cb = cos(st.beta);
sb = sin(st.beta);
force.x = -drag * ca * cb + lift * sa;
force.y = lift * ca + drag * sa * cb;
force.z = side - drag * sb;

% thrust: nozzles tilt it, and yawing fast with TVC on loses some along the jet
if aux.tvc_ok
    yaw_rate_deg = abs(st.r_yaw) * 180 / pi;
    yaw_thrust = 1 - (1 - min(max(1 - L.yawloss_half * (yaw_rate_deg - L.yawloss_start_deg) / L.yawloss_span_deg, L.yawloss_half), 1)) * (1 - aux.man_w);
else
    yaw_thrust = 1;
end
force.x = force.x + thrust * cos(act.noz_pitch) * cos(act.noz_yaw) * yaw_thrust;
% the nozzle sits behind the CG: a nose-up thrust moment needs a downward force on the tail
force.y = force.y - thrust * sin(act.noz_pitch);
force.z = force.z - thrust * sin(act.noz_yaw) * yaw_thrust;

% moments
qsc = st.qbar * S * c;
qsb = st.qbar * S * b;
Cm = a0.Cm + Cm_d * act.stab + A.flap_cm * cfg.flap_pos + cm_extra;
Cl = a0.Cl + Cl_d * act.flaperon;
Cn = a0.Cn + Cn_d * act.rudder;
moment.z = Cm * qsc + thrust * sin(act.noz_pitch) * L.tvc_arm;
moment.x = Cl * qsb + thrust * sin(act.noz_roll) * L.tvc_roll_arm;
moment.y = -(Cn * qsb + thrust * sin(act.noz_yaw) * aux.yaw_arm + thrust_asym);

% load factors as the flight model derives them from DCS's body accelerations (cpp:4545-4546)
weight = st.mass * P.const.g;
nz = force.y / weight;
ny_lat = force.z / weight;
end

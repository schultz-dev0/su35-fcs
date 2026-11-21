function [cmd_surf, aux, x] = fcs_allocation(P, cmd, st, qd, pd, rd, act, ab_level, x, dt)
% FCS_ALLOCATION  Dynamic inversion and control allocation: desired angular accelerations to surface and nozzle commands
% (cpp:3640-3880, 3902-3906). Surfaces first, the nozzles take what they cannot give. Also the TVC Maneuver mode.
%   cmd        from fcs_commands
%   qd, pd, rd desired accelerations from the Simulink loops (pitch, roll, yaw right), rad/s2
%   act        last frame's actuators: noz_pitch, noz_yaw (rad), as the nozzle feed-forward and the envelope read them
%   ab_level   larger afterburner level of the two engines (nozzle envelope)
% Outputs are in the flight model's own signs; the model converts at the Surfaces bus:
%   cmd_surf   stab (+ = nose-up moment), flaperon, rudder (+ nose right), noz_pitch, noz_yaw, noz_roll (rad)
%   aux        tvc_ok, man_w, yaw_arm (m), noz_rate_pitch, noz_rate_other (rad/s), for the plant and actuators
L = P.alloc;
A = P.aero;
G = P.const.g;
d2r = pi / 180;

thrust = st.thrust_left + st.thrust_right;
qsc = st.qbar * P.geom.S * P.geom.c;
qsb = st.qbar * P.geom.S * P.geom.b;
tvc_ok = cmd.tvc_ok > 0.5;
sp = cmd.sp;
sr = cmd.sr;
sy = cmd.sy;
alpha = st.alpha;

% the FBW's own model of the bare airframe: the same function the plant runs
p_hat = st.p * P.geom.b / (2 * max(st.V, 30));
q_hat = st.q * P.geom.c / (2 * max(st.V, 30));
r_hat = -st.r_yaw * P.geom.b / (2 * max(st.V, 30));
airframe.gear_pos = st.gear_pos;
airframe.airbrake_pos = st.airbrake_pos;
airframe.flap_pos = st.flap_pos;
airframe.flap_brake_pos = st.flap_brake_pos;
ground_effect = plant_ground_effect(P, st.altitude);
a0 = plant_aero_bare(P, alpha, st.beta, st.mach, p_hat, q_hat, r_hat, airframe, ground_effect);
cm_extra = plant_cm_extra(P, st.mach, alpha, st.airbrake_pos);

%% moments the loops ask for (cpp:3649)
thrust_asym = (st.thrust_left - st.thrust_right) * L.asym_arm;     % left stronger: nose right
Mz_need = st.Izz * qd - a0.Cm * qsc - L.cm_extra_feed * cm_extra * qsc;
Mx_need = st.Ixx * pd - cmd.lat_fb * a0.Cl * qsb;
Mn_need = st.Iyy * rd - cmd.lat_fb * a0.Cn * qsb - thrust_asym;     % + nose right

% inertial coupling, M = I w_dot + w x (I w) (cpp:3687-3692)
wx = st.p;
wy = st.r_yaw;
wz = st.q;
Mx_need = Mx_need + (st.Izz - st.Iyy) * wy * wz;
Mz_need = Mz_need + (st.Iyy - st.Ixx) * wx * wy;
Mn_need = Mn_need - (st.Ixx - st.Izz) * wz * wx;

%% control power (cpp:3694-3702)
eff_pitch = plant_eff(P, st.mach, alpha);
eff_lat = plant_eff_lat(P, st.mach, alpha);
stab_low_speed = 1 + A.stab_low_speed_gain * min(max((A.stab_low_speed_kmh - cmd.ias_kmh) / A.stab_low_speed_width_kmh, 0), 1);
Cm_d = A.Cm_d * eff_pitch * stab_low_speed;
Cl_d = A.Cl_d * eff_lat;
Cn_d = A.Cn_d * eff_lat;

%% thrust vectoring feed-forward and limits (cpp:3721-3754)
tvc_on = cmd.tvc_on > 0.5;
% with TVC off the laws that keep the full travel are always on (g_law 1), so every feed-forward below is zero
if tvc_on
    g_law = 0;
else
    g_law = 1;
end
gate_pitch = fcs_tvc_mach_gate(P, st.mach, sp, false) * cmd.tvc_stores;
gate_yaw = fcs_tvc_mach_gate(P, st.mach, sp, true) * cmd.tvc_stores;
g_p = max(gate_pitch, g_law);
g_y = max(gate_yaw, g_law);
if tvc_on
    yaw_boost = fcs_tvc_yaw_boost(P, st.mach, sy, abs(alpha), st.phi, -sy);
else
    yaw_boost = 0;
end
yaw_arm = L.tvc_arm * (1 + yaw_boost);
yaw_arm_now = (1 - cmd.man_w) * yaw_arm + cmd.man_w * L.tvc_arm;

tilt_pitch = L.tvc_pitch;
tilt_yaw = L.tvc_yaw;
if sp <= 0
    ff_limit = 1.0;
else
    ff_limit = min(max((cmd.aoa_lim - alpha) / L.tvc_ff_aoa_fade, 0), 1);
end
roll_share = 0;
if tvc_ok
    roll_share = abs(sr) * (1 - abs(sp)) * fcs_tvc_mach_gate(P, st.mach, sp, false) * cmd.tvc_stores;
end
np_ff = 0;
ny_ff = 0;
nr_ff = 0;
if tvc_ok
    np_ff = sp * sin(tilt_pitch) * ff_limit * g_p;
    ny_ff = sy * sin(tilt_yaw) * g_y;
    nr_ff = sign_or_one(sr) * sin(tilt_pitch * roll_share);
end
% the pitch nozzles move slowly in the normal law, so the stabilators trim around where they are, not where they go
noz_fast = g_law;
Mz_ff = (noz_fast * np_ff + (1 - noz_fast) * sin(act.noz_pitch)) * thrust * L.tvc_arm;
Mn_ff = ny_ff * thrust * yaw_arm;
Mx_ff = nr_ff * thrust * L.tvc_roll_arm;

stab_max = L.stab_max;
ail_max = L.ail_max;
rud_max = L.rud_max;
if qsc > 1
    d_stab = min(max((Mz_need - Mz_ff) / (Cm_d * qsc), -stab_max), stab_max);
else
    d_stab = 0;
end
if qsb > 1
    d_ail = min(max((Mx_need - Mx_ff) / (Cl_d * qsb), -ail_max), ail_max);
    d_rud = min(max((Mn_need - Mn_ff) / (Cn_d * qsb), -rud_max), rud_max);
else
    d_ail = 0;
    d_rud = 0;
end
Mz_aero = d_stab * Cm_d * qsc;
Mx_aero = d_ail * Cl_d * qsb;
Mn_aero = d_rud * Cn_d * qsb;

np = 0;
ny = 0;
nr = 0;
if tvc_ok
    % the stick feed-forward stays in the nozzles; only what the surfaces could not give is added on top
    np = min(max(np_ff + (Mz_need - Mz_ff - Mz_aero) / (thrust * L.tvc_arm), -sin(tilt_pitch) * g_p), sin(tilt_pitch) * g_p);
    ny = min(max(ny_ff + (Mn_need - Mn_ff - Mn_aero) / (thrust * yaw_arm), -sin(tilt_yaw) * g_y), sin(tilt_yaw) * g_y);
    roll_nozzle_max = sin(tilt_pitch) * g_p;
    nr = min(max(nr_ff + (Mx_need - Mx_ff - Mx_aero) / (thrust * L.tvc_roll_arm), -roll_nozzle_max), roll_nozzle_max);
end

%% nozzle envelope (cpp:3772-3782): none above 550 km/h or 12 000 m, and a time limit at full deflection with afterburner
afterburner = ab_level > L.ab_light_level;
nozzle_deflection = max(abs(act.noz_pitch), abs(act.noz_yaw));
if afterburner && nozzle_deflection > L.tvc_pitch - 1 * d2r
    x.ab_full_t = x.ab_full_t + dt;
elseif ~afterburner || nozzle_deflection < L.noz_ab_limit
    x.ab_full_t = 0;
end
ias_gate = min(max((L.noz_ias_gate_kmh - cmd.ias_kmh) / L.noz_ias_span_kmh, 0), 1);
alt_gate = min(max((L.noz_alt_gate - st.altitude) / L.noz_alt_span, 0), 1);
if x.ab_full_t > L.noz_ab_time
    noz_limit = min(L.noz_ab_limit, L.tvc_pitch);
else
    noz_limit = L.tvc_pitch;
end
noz_gate = ias_gate * alt_gate * cmd.tvc_stores;
man_noz_max = sin(noz_limit) * noz_gate;
man_noz_y_max = sin(min(noz_limit, L.tvc_yaw)) * noz_gate;

%% TVC Maneuver mode: every stick direction is a rate demand (cpp:3787-3879)
man_w = cmd.man_w;
if man_w > 0
    q_dem_max = L.man_q;
    q_dem = sp * q_dem_max;
    % pitch stability augmentor: near centre the stick blends from a rate demand to an AoA demand toward the 1 g AoA
    cl_slope = plant_table(A.mach_tab, A.cla_tab, st.mach);
    cl_one_g = st.mass * G / max(st.qbar * P.geom.S, 1.0);
    aoa_one_g = A.alpha0 + cl_one_g / cl_slope;
    aoa_trim = min(max(aoa_one_g, L.man_aoa_trim_lo), L.man_aoa_trim_hi);
    aoa_error = wrap_pi(aoa_trim - alpha);
    aoa_trusted = min(max((st.V - L.man_v_trust_lo) / L.man_v_trust_span, 0), 1);
    aug_w = (1 - min(max(abs(sp) / max(L.man_aoa_band, 0.01), 0), 1)) * aoa_trusted;
    q_aug = min(max(L.man_aoa_gain * aoa_error, -q_dem_max), q_dem_max);
    q_dem = q_dem + aug_w * q_aug;
    q_g_up = cmd.nz_top * G / max(st.V, 30);
    q_g_down = P.pitch.nz_min * G / max(st.V, 30);
    q_dem = min(max(q_dem, q_g_down), q_g_up);

    % roll stick: a roll about the flight path (pedal free); with the pedal in, a plain body roll
    pedal_free = 1 - min(max(abs(sy) / L.man_pedal_band, 0), 1);
    r_dem_max = L.man_r;
    path_roll = sr * L.man_p;
    aoa_path = min(max(alpha, -pi / 2), pi / 2);
    roll_body_share = pedal_free * cos(aoa_path) + (1 - pedal_free);
    spin_w = min(max((abs(aoa_path) - L.man_spin_start) / L.man_spin_span, 0), 1);
    pedal_rate = sy * r_dem_max;
    aoa_sign = sign_or_one(aoa_path);
    pedal_roll = pedal_rate * spin_w * cos(aoa_path) * aoa_sign;
    pedal_yaw = pedal_rate * ((1 - spin_w) + spin_w * abs(sin(aoa_path)));
    p_dem = path_roll * roll_body_share + pedal_roll;
    roll_yaw = pedal_free * min(max(path_roll * sin(aoa_path), -r_dem_max), r_dem_max);
    % directional stability augmentor: the sideslip feeds the yaw demand with the pedal released
    beta_trusted = min(max((st.V - L.man_v_trust_lo) / L.man_v_trust_span, 0), 1);
    beta_dem = min(max(L.man_beta_gain * st.beta, -r_dem_max), r_dem_max) * pedal_free * beta_trusted;
    yaw_right_dem = roll_yaw + pedal_yaw + beta_dem;
    yaw_right = -st.r_yaw;

    Mz_man = st.Izz * P.pitch.q_bandwidth * (q_dem - st.q) - a0.Cm * qsc - L.cm_extra_feed * cm_extra * qsc;
    Mx_man = st.Ixx * P.roll.p_bandwidth * (p_dem - st.p) - a0.Cl * qsb;
    Mn_man = st.Iyy * P.yaw.r_bandwidth * (yaw_right_dem - yaw_right) - a0.Cn * qsb;
    Mx_man = Mx_man + (st.Izz - st.Iyy) * st.r_yaw * st.q;
    Mz_man = Mz_man + (st.Iyy - st.Ixx) * st.p * st.r_yaw;
    Mn_man = Mn_man - (st.Ixx - st.Izz) * st.q * st.p;

    [stab_man, np_man] = allocate_axis(Mz_man, stab_max, Cm_d * qsc, man_noz_max, thrust * L.tvc_arm);
    [ail_man, nr_man] = allocate_axis(Mx_man, ail_max, Cl_d * qsb, man_noz_max, thrust * L.tvc_roll_arm);
    [rud_man, ny_man] = allocate_axis(Mn_man, rud_max, Cn_d * qsb, man_noz_y_max, thrust * L.tvc_arm);
    d_stab = (1 - man_w) * d_stab + man_w * stab_man;
    d_ail = (1 - man_w) * d_ail + man_w * ail_man;
    d_rud = (1 - man_w) * d_rud + man_w * rud_man;
    np = (1 - man_w) * np + man_w * np_man;
    ny = (1 - man_w) * ny + man_w * ny_man;
    nr = (1 - man_w) * nr + man_w * nr_man;
end

cmd_surf.stab = d_stab;
cmd_surf.flaperon = d_ail;
cmd_surf.rudder = d_rud;
cmd_surf.noz_pitch = asin(min(max(np, -1), 1));
cmd_surf.noz_yaw = asin(min(max(ny, -1), 1));
cmd_surf.noz_roll = asin(min(max(nr, -1), 1));

aux.tvc_ok = cmd.tvc_ok;
aux.man_w = man_w;
aux.yaw_arm = yaw_arm_now;
noz_rate_pitch = L.tvc_rate_normal + (L.tvc_rate - L.tvc_rate_normal) * noz_fast;
aux.noz_rate_pitch = (1 - man_w) * noz_rate_pitch + man_w * L.man_noz_rate;
aux.noz_rate_other = (1 - man_w) * L.tvc_rate + man_w * L.man_noz_rate;
end

function [surface, nozzle] = allocate_axis(moment, surface_max, surface_power, nozzle_max, nozzle_power)
% Surface first, the nozzle takes the rest (cpp:3855-3862).
surface = 0;
if surface_power > 1
    surface = min(max(moment / surface_power, -surface_max), surface_max);
end
rest = moment - surface * surface_power;
nozzle = 0;
if nozzle_power > 1
    nozzle = min(max(rest / nozzle_power, -nozzle_max), nozzle_max);
end
end

function s = sign_or_one(v)
% The mod's (v >= 0 ? 1 : -1): zero counts as positive.
if v >= 0
    s = 1;
else
    s = -1;
end
end

function a = wrap_pi(a)
a = mod(a + pi, 2 * pi) - pi;
end

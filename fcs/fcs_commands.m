function [cmd, x] = fcs_commands(P, pilot, st, noz_pitch, x, dt)
% FCS_COMMANDS  The three rate commands (pitch q_cmd, roll p_cmd, yaw r_cmd) and the mode weights, one frame.
% Port of simulate() cpp:2031-3285 for the airborne Flight, Takeoff-Landing, Refuelling and TVC Maneuver modes
% (the Maneuver demands themselves are in fcs_allocation). Everything the document lists as left out is not here:
% see docs/rebuild.md.
%   pilot       bus Pilot
%   st          bus State (nz and ny_lat arrive one frame late, as DCS delivers them)
%   noz_pitch   actual pitch nozzle angle, rad, last frame (the TVC limit share reads it, cpp:613)
%   x           state struct, see fcs_state_init; returned updated
% The loops (rate error times bandwidth plus integrator) are Simulink blocks around this function.
F = P.pitch;
R = P.roll;
Y = P.yaw;
L = P.alloc;
G = P.const.g;
d2r = pi / 180;

V = max(st.V, F.v_min_pitch);
ias_kmh = st.V * sqrt(st.rho / P.const.rho0) * 3.6;
thrust = st.thrust_left + st.thrust_right;
alpha = st.alpha;
beta = st.beta;
phi = st.phi;
theta = st.theta;

% flight-path angle from the velocity direction and "up" in body axes (the mod reads it from DCS's world velocity)
dir_x = cos(alpha) * cos(beta);
dir_y = -sin(alpha) * cos(beta);
dir_z = sin(beta);
up_x = sin(theta);
up_y = cos(theta) * cos(phi);
up_z = -cos(theta) * sin(phi);
sin_gamma = min(max(dir_x * up_x + dir_y * up_y + dir_z * up_z, -1), 1);
gamma = asin(sin_gamma);

%% stick, pedals and the Refuelling blend (cpp:2031-2090)
hold_deadzone = fcs_stick_shape(P, F.hold_deadzone_raw);
sp = fcs_stick_shape(P, min(max(pilot.stick_pitch, -1), 1));
sr = fcs_stick_shape(P, min(max(pilot.stick_roll, -1), 1));
sy = min(max(pilot.pedals, -1), 1);
% dead zone that grows with the pull, so a hand pulling hard that leaks sideways does not roll the jet
roll_db = R.roll_pull_db * min(max(abs(sp), 0), 1);
if abs(sr) > roll_db
    roll_mag = (abs(sr) - roll_db) / (1 - roll_db);
else
    roll_mag = 0;
end
sr = sign(sr) * roll_mag;

refuel_envelope = ias_kmh < F.rf_ias_max_kmh && st.mach <= F.rf_mach_max && st.gear_pos < 0.5;
refuel_want = F.rf_mode_on >= 0.5 && pilot.refuel_probe > 0.5 && refuel_envelope;
if refuel_want
    refuel_sign = 1;
else
    refuel_sign = -1;
end
x.rf_w = min(max(x.rf_w + refuel_sign * dt / max(F.rf_blend_s, 0.05), 0), 1);
rf_w = x.rf_w;
rf_sp = min(max(pilot.stick_pitch, -1), 1);
rf_sr = min(max(pilot.stick_roll, -1), 1);
sp = sp * (1 - rf_w);
sr = sr * (1 - rf_w);

% how long the stick has been centred, in seconds: gates the fast-roll g feedback and the vertical no-roll rule
hold_ok = st.V > R.int_v_min;
if abs(sp) < hold_deadzone && hold_ok
    x.pitch_free_t = x.pitch_free_t + dt;
else
    x.pitch_free_t = 0;
end
if abs(sr) < hold_deadzone && hold_ok
    x.roll_free_t = x.roll_free_t + dt;
else
    x.roll_free_t = 0;
end

%% thrust vectoring and limits (cpp:2121, 2186-2199)
tvc_ok = pilot.tvc_on > 0.5 && thrust > L.tvc_min_thrust;
tvc_w = fcs_tvc_limit_share(P, thrust, noz_pitch, st.altitude);
tvc_g_stores = L.tvc_g_stores_light;
tvc_stores = 1;
if tvc_ok
    nz_top = fcs_tvc_nz_limit(P, st.mach, tvc_w, tvc_g_stores);
    aoa_lim = max(fcs_tvc_aoa_limit(P, st.mach, tvc_w) * d2r, F.aoa_limit);
else
    nz_top = fcs_nz_limit_tvc_off(P, st.mach, st.mass);
    aoa_lim = F.aoa_limit;
end
if st.gear_pos > 0.5
    aoa_lim = F.aoa_limit_gear;
end

%% pitch: g command (cpp:2627-2635)
% a centred stick asks for the g that keeps the flight path straight: cos(path) / cos(bank), exact to 60 deg of bank
bank = abs(phi);
level_g = cos(gamma) / cos(min(bank, F.bank_neutral_max));
bank_w = min(max((F.bank_neutral_fade_start - bank) / F.bank_neutral_fade_width, 0), 1);
roll_w = min(max((F.roll_neutral_fade_start - abs(st.p)) / F.roll_neutral_fade_width, 0), 1);
nz_neutral = 1 + bank_w * roll_w * (level_g - 1);
if sp >= 0
    nz_cmd = nz_neutral + sp * (nz_top - nz_neutral);
else
    nz_cmd = nz_neutral + (-sp) * (F.nz_min - nz_neutral);
end

% Takeoff-Landing law: the stick moves an AoA reference, hands off the AoA stays (cpp:3004-3020)
tl_w = min(max(st.gear_pos, 0), 1);
if tl_w > 0 && x.tl_was < 0.5
    x.tl_aoa_ref = alpha;
end
x.tl_was = double(tl_w > 0);
if tl_w > 0
    x.tl_aoa_ref = x.tl_aoa_ref + sp * F.tl_aoa_rate * dt;
    % the reference stays near the jet so holding the stick past what it can do does not wind it up
    x.tl_aoa_ref = min(max(x.tl_aoa_ref, alpha - F.tl_aoa_follow), alpha + F.tl_aoa_follow);
    x.tl_aoa_ref = min(max(x.tl_aoa_ref, F.tl_aoa_min), F.tl_aoa_max);
    nz_tl = st.nz + F.tl_aoa_gain * (x.tl_aoa_ref - alpha) * V / G;
    nz_cmd = (1 - tl_w) * nz_cmd + tl_w * min(max(nz_tl, F.nz_min), F.nz_max);
end

% Refuelling law: the stick sets the pitch attitude of level flight plus a climb angle (cpp:3021-3043)
if rf_w > 0
    if x.rf_pitch_was < 0.5
        x.rf_level_pitch = theta - gamma;
    end
    trim_k = min(max(dt / max(F.rf_trim_s, 0.1), 0), 1);
    x.rf_level_pitch = x.rf_level_pitch + (theta - gamma - x.rf_level_pitch) * trim_k;
    climb = min(max(rf_sp * F.rf_pitch_deg_full, -F.rf_pitch_max_deg), F.rf_pitch_max_deg) * d2r;
    pitch_ref = x.rf_level_pitch + climb;
    bank_cos = cos(min(abs(phi), F.bank_neutral_max));
    q_want = F.rf_att_gain * (pitch_ref - theta);
    nz_rf = cos(gamma) / bank_cos + q_want * V / G;
    nz_cmd = (1 - rf_w) * nz_cmd + rf_w * min(max(nz_rf, 0), F.rf_nz_max);
end
x.rf_pitch_was = double(rf_w > 0);

%% pitch: rate command (cpp:3044-3100)
q_cmd = (nz_cmd - cos(theta) * cos(phi)) * G / V;

% slow, stick in hand: the stick also asks for a pitch rate so the nose always follows it
w_low = min(max((F.slow_stick_ias_hi_kmh - ias_kmh) / F.slow_stick_ias_span_kmh, 0), 1);
if w_low > 0 && abs(sp) >= hold_deadzone && st.gear_pos < F.slow_stick_gear_pos
    q_floor = sp * F.q_rate_max * w_low;
    if sp < 0
        q_cmd = min(q_cmd, q_floor);
    else
        q_cmd = max(q_cmd, q_floor);
    end
end

% pedal sliding the nose sideways: the nose stays put and the path comes to it (cpp:2584-2592, 3065-3068)
if abs(sy) < Y.ped_free_db
    x.ped_off_t = x.ped_off_t + dt;
else
    x.ped_off_t = 0;
end
beta_big = abs(beta) > F.ped_bends_beta;
if tvc_ok
    slip_left = beta_big && x.ped_off_t < Y.tvc_point_hold_s + 6.0;
    ped_bends_path = st.gear_pos < 0.5 && (x.ped_off_t < Y.tvc_point_hold_s || slip_left);
else
    ped_bends_path = st.gear_pos < 0.5 && (x.ped_off_t < F.ped_bends_off_s || (beta_big && x.ped_off_t < F.ped_bends_slip_s));
end
if ped_bends_path && x.pitch_free_t >= F.fast_roll_hold_time
    pedal_slip_w = min(max(abs(beta) / F.ped_slip_full, 0), 1);
    q_cmd = q_cmd * (1 - pedal_slip_w);
end

if tvc_ok
    q_lim = fcs_tvc_q_limit(P, st.mach, tvc_w) * d2r;
else
    q_lim = F.q_rate_max;
end

% fast rolls: above ~60 deg/s of roll the loop also holds the g directly (cpp:3074-3084)
fast_roll_w = min(max((abs(st.p) - F.fast_roll_p_start) / F.fast_roll_p_width, 0), 1);
if x.pitch_free_t >= F.fast_roll_hold_time
    push_w = min(max((ias_kmh - F.fast_roll_ias_lo_kmh) / F.fast_roll_ias_span_kmh, 0), 1);
else
    push_w = 0;
end
nz_roll = push_w * nz_cmd + (1 - push_w) * max(nz_cmd, 1.0);
q_cmd = q_cmd + fast_roll_w * F.fast_roll_g_gain * (nz_roll - st.nz) * G / max(V, F.v_min_roll_g);
q_cmd = min(max(q_cmd, -q_lim), q_lim);

% AoA limiter, anticipating: the allowed nose-up rate shrinks to zero at the limit (cpp:3092-3100)
if pilot.limiter_off < 0.5
    q_path = (st.nz - cos(theta) * cos(phi)) * G / V;
    if tvc_ok
        lead = F.aoa_lead_tvc_s;
    else
        lead = F.aoa_lead_s;
    end
    q_cmd = min(q_cmd, q_path + F.aoa_limiter_gain * (aoa_lim - alpha - lead * (st.q - q_path)));
elseif alpha > aoa_lim
    q_cmd = q_cmd - F.aoa_over_gain * (alpha - aoa_lim);
end
if alpha < F.aoa_neg_limit
    q_cmd = q_cmd + F.aoa_neg_gain * (F.aoa_neg_limit - alpha);
end

%% roll rate command (cpp:3104-3201)
roll_hi = 0;
if tvc_ok
    roll_hi = L.tvc_roll_aoa_gain * min(max((abs(alpha) / d2r - L.tvc_roll_aoa_start) / L.tvc_roll_aoa_span, 0), 1);
end
roll_hi = min(roll_hi, 1);
mx_tvc = 0;
if tvc_ok
    mx_tvc = fcs_tvc_roll_moment(P, sr, sp * (1 - roll_hi), sy, st.mach, st.nz, st.thrust_left, st.thrust_right) * tvc_stores;
end
x.p_ref = fcs_team_roll_step(P, x.p_ref, sr, st.mach, st.V, alpha, beta, st.qbar, st.Ixx, st.mass, dt, mx_tvc, double(tvc_ok), roll_hi);
% stick back at centre: stop the roll where it is, F-16 style
if abs(sr) < hold_deadzone
    x.p_ref = x.p_ref * exp(-dt / R.stop_tau);
end
p_cmd = x.p_ref;

% head-g limiter: soft knee, untouched below roll_knee of the limit, rounds off toward it
x.nz_f = x.nz_f + (st.nz - x.nz_f) * min(max(dt / R.head_g_tau, 0), 1);
head_limit = sqrt(max(x.nz_f - R.head_g_min, 0.3) * G / max(R.head_arm_m, 0.1));
knee = R.roll_knee * head_limit;
if abs(p_cmd) > knee && head_limit > knee
    p_cmd = sign(p_cmd) * (knee + (head_limit - knee) * tanh((abs(p_cmd) - knee) / (head_limit - knee)));
end

% near vertical the bank means nothing: no roll
if x.roll_free_t >= 0.4 && abs(theta) > 75 * d2r
    p_cmd = 0;
end

% Refuelling: roll rate linear in the stick up to a bank limit, bank held hands off (cpp:3157-3178)
if rf_w > 0
    bank_max = F.rf_bank_max;
    if x.rf_roll_on < 0.5
        x.rf_bank_target = min(max(phi, -bank_max), bank_max);
    end
    x.rf_roll_on = 1;
    if abs(rf_sr) > F.rf_stick_db
        p_rf = rf_sr * F.rf_bank_rate;
        p_rf = min(max(p_rf, F.rf_bank_gain * (-bank_max - phi)), F.rf_bank_gain * (bank_max - phi));
        x.rf_bank_target = min(max(phi, -bank_max), bank_max);
    else
        p_rf = min(max(F.rf_bank_gain * (x.rf_bank_target - phi), -F.rf_hold_rate_max), F.rf_hold_rate_max);
    end
    p_cmd = (1 - rf_w) * p_cmd + rf_w * p_rf;
else
    x.rf_roll_on = 0;
end

% roll acceleration and rate limited to what the rudders (and yaw nozzles) can coordinate (cpp:3183-3200)
eff_lat = plant_eff_lat(P, st.mach, alpha);
yaw_authority = R.slew_rudder_deg * R.slew_cn_d * eff_lat * st.qbar * P.geom.S * P.geom.b;
if tvc_ok
    yaw_authority = yaw_authority + thrust * sin(L.tvc_yaw) * L.tvc_arm;
end
roll_accel_max = max(R.slew_floor, R.slew_yaw_share * yaw_authority / st.Iyy / max(tan(abs(alpha)), R.slew_tan_min));
% rolling while pitching makes a yaw moment the rudders must cancel
coupling_per_p = (st.Izz - st.Ixx) * abs(st.q);
if coupling_per_p > 1.0
    p_yaw_max = R.coupling_yaw_share * yaw_authority / coupling_per_p;
    p_cmd = min(max(p_cmd, -p_yaw_max), p_yaw_max);
end
x.p_cmd_s = x.p_cmd_s + min(max(p_cmd - x.p_cmd_s, -roll_accel_max * dt), roll_accel_max * dt);
p_cmd = x.p_cmd_s;

%% yaw rate command (cpp:3202-3285)
lat_fb = min(max((Y.lat_fb_aoa_start - abs(alpha)) / Y.lat_fb_aoa_width, 0), 1);
beta_fb = Y.beta_gain * beta * lat_fb;
% TVC on, pedal in: the FBW no longer cancels the sideslip the pedal makes
if tvc_ok && st.gear_pos < 0.5
    beta_fb = beta_fb * (1 - min(max((abs(sy) - Y.tvc_beta_fade_db) / Y.tvc_beta_fade_width, 0), 1));
end
pedal_free = abs(sy) < Y.ped_free_db;
if pedal_free
    x.ped_free_t = x.ped_free_t + dt;
else
    x.ped_free_t = 0;
end
pointing = tvc_ok && Y.tvc_point_hold_s > 0 && x.ped_free_t < Y.tvc_point_hold_s && abs(sr) < 0.04;
if pedal_free && pointing
    beta_fb = beta_fb * min(max(x.ped_free_t - (Y.tvc_point_hold_s - 1.0), 0), 1);
elseif pedal_free && Y.yaw_release_s > 0
    % pedal let go: the sideslip feedback comes back gradually so the nose does not snap back
    beta_fb = beta_fb * min(max(x.ped_free_t / Y.yaw_release_s, 0), 1);
end
if ~pointing && abs(sr) >= 0.04
    x.ped_free_t = max(x.ped_free_t, Y.tvc_point_hold_s);
end
if tvc_ok && x.ped_off_t < Y.tvc_point_hold_s + Y.tvc_beta_clamp_extra_s
    beta_fb = min(max(beta_fb, -Y.tvc_beta_clamp), Y.tvc_beta_clamp);
end

% the pedal's yaw command ramps in and out, no step either way
ped_slew = dt / Y.ped_slew_s;
x.sy_s = x.sy_s + min(max(sy - x.sy_s, -ped_slew), ped_slew);

ped_spd = min(max((ias_kmh - Y.ped_fade_lo_kmh) / max(Y.ped_fade_hi_kmh - Y.ped_fade_lo_kmh, 1), 0), 1);
ped_fade_yaw = 1 - (1 - Y.ped_fade_yaw_min) * ped_spd;
ped_rate = Y.yaw_rate_max;
slip_lim = 1.0;
if tvc_ok && st.gear_pos < 0.5
    % TVC pedal: a yaw-rate demand with a sideslip limiter that eases the rate to zero over the last half of the limit
    speed_share = min(max((ias_kmh - Y.tvc_rot_full_kmh) / max(Y.ped_beta_ias_hi_kmh - Y.tvc_rot_full_kmh, 1), 0), 1);
    beta_level = Y.ped_beta_lo + (Y.ped_beta_hi - Y.ped_beta_lo) * speed_share;
    level_pitch = min(max((Y.level_pitch_start - abs(theta)) / Y.level_pitch_width, 0), 1);
    level_bank = min(max((cos(phi) - cos(Y.level_bank_hi)) / (cos(Y.level_bank_lo) - cos(Y.level_bank_hi)), 0), 1);
    out_of_level = 1 - level_pitch * level_bank;
    beta_limit = beta_level * (1 + (Y.tvc_ped_beta_free - 1) * out_of_level);
    if sy >= 0
        slip_into = -beta;
    else
        slip_into = beta;
    end
    ped_rate = Y.tvc_ped_rate;
    slip_lim = min(max((beta_limit - slip_into) / (Y.slip_lim_share * beta_limit), 0), 1);
end
r_cmd = x.sy_s * ped_rate * ped_fade_yaw * slip_lim + beta_fb;

% gravity turns the flight path sideways in a bank: without this term the yaw lagged the sideslip feedback
r_grav = G / max(st.V, Y.v_min_grav) * cos(theta) * sin(phi) * cos(alpha) * lat_fb;
if tvc_ok && st.gear_pos < 0.5
    r_grav = r_grav * (1 - min(max((abs(sy) - Y.tvc_beta_fade_db) / Y.tvc_beta_fade_width, 0), 1));
end
r_cmd = r_cmd + r_grav;

% roll about the flight path: yaw at p tan(AoA) keeps the AoA while rolling (+ nose right)
r_coord = Y.roll_coord * st.p * tan(min(max(alpha, -Y.coord_aoa_clip), Y.coord_aoa_clip));
r_cmd = r_cmd + r_coord;
% its rate goes straight to the rudders as feed-forward, the 3 rad/s yaw loop alone lags a fast roll
coord_rate = min(max((r_coord - x.r_coord_prev) / max(dt, 1e-3), -Y.coord_dot_clip), Y.coord_dot_clip);
x.r_coord_dot = x.r_coord_dot + (coord_rate - x.r_coord_dot) * min(max(dt / Y.coord_dot_tau, 0), 1);
x.r_coord_prev = r_coord;

%% roll into the pedal (cpp:3322-3394): plain rudder with the roll stick free rolls the jet toward the yaw
% rot_v is the TVC pedal-rotation window (cpp:3309); its law is not rebuilt, but the roll gives way to it as in the C++
rot_scale = 1.0;
if tvc_ok && cos(phi) < 0
    rot_scale = Y.tvc_rot_inverted_scale;
end
rot_v = 0;
if tvc_ok && st.gear_pos < 0.5
    rot_end_kmh = Y.tvc_rot_end_kmh * rot_scale;
    rot_v = min(max((rot_end_kmh - ias_kmh) / max(rot_end_kmh - Y.tvc_rot_full_kmh, 1), 0), 1);
end
ped_roll_w = 0;
if st.gear_pos < 0.5
    ped_roll_w = 1 - rot_v;
end
ped_in = min(max((abs(sy) - Y.ped_in_db) / Y.ped_in_span, 0), 1);
if ped_roll_w > 0 && abs(sr) < 0.1 && ped_in > 0 && Y.ped_roll > 0
    if sy > 0
        ped_side = 1;
    else
        ped_side = -1;
    end
    if cos(phi) >= 0
        upright_sign = 1;
    else
        upright_sign = -1;
    end
    ped_fade_roll = 1 - (1 - Y.ped_fade_roll_min) * ped_spd;
    ped_bank_max = (90 * d2r) + (Y.ped_bank_hi - 90 * d2r) * ped_spd;
    bank_room_span = max(cos(ped_bank_max - 20 * d2r) - cos(ped_bank_max), 1e-3);
    roll_bank_room = min(max((cos(phi) - cos(ped_bank_max)) / bank_room_span, 0), 1);
    slip_toward = max(-beta * ped_side, 0);
    % r_yaw is + nose left in DCS axes, so -r_yaw * ped_side is the yaw rate toward the pedal
    yaw_into = max(max(-st.r_yaw * ped_side, 0), Y.ped_roll_beta_k * slip_toward);
    deeper = ped_side * upright_sign * sin(phi) > 0;
    if deeper
        room = roll_bank_room;
    else
        room = 1;
    end
    path_span = max(Y.ped_path_none - Y.ped_path_full, 1 * d2r);
    level_path = min(max((Y.ped_path_none - abs(gamma)) / path_span, 0), 1);
    % this replaces the roll command after its acceleration limit, and the limiter's state keeps its own value (cpp:3392)
    p_cmd = Y.ped_roll * ped_roll_w * ped_fade_roll * ped_in * yaw_into * ped_side * upright_sign * level_path * room;
end

%% TVC Maneuver weight (cpp:3710-3715): slow, TVC on, airborne, gear up, not refuelling
man_w = 0;
if L.tvc_maneuver > 0 && tvc_ok && st.gear_pos < 0.5 && rf_w <= 0
    man_w = min(max((L.man_end_kmh - ias_kmh) / max(L.man_end_kmh - L.man_full_kmh, 1), 0), 1);
end

cmd.sp = sp;
cmd.sr = sr;
cmd.sy = sy;
cmd.q_cmd = q_cmd;
cmd.p_cmd = p_cmd;
cmd.r_cmd = r_cmd;
cmd.r_coord_dot = x.r_coord_dot;
cmd.lat_fb = lat_fb;
cmd.tvc_ok = double(tvc_ok);
cmd.tvc_on = double(pilot.tvc_on > 0.5);
cmd.tvc_stores = tvc_stores;
cmd.man_w = man_w;
cmd.nz_top = nz_top;
cmd.aoa_lim = aoa_lim;
cmd.ias_kmh = ias_kmh;
cmd.sin_gamma = sin_gamma;
end

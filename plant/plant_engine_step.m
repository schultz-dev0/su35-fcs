function [rpm_new, ab_new, thrust] = plant_engine_step(P, throttle, rpm, ab, atmo, V, dt)
% PLANT_ENGINE_STEP  One engine, running, on an ISA day, one flight-model step (the mod's engine_step, cpp:777-931).
%   throttle   lever 0..1; the afterburner zone is the top of it
%   rpm, ab    states: core rpm (share of max) and afterburner level 0..1
%   atmo       struct: rho (kg/m3), T (K), p (Pa), a (m/s)
%   V          true airspeed, m/s
% Not rebuilt: start-up and run-down, fuel flow, EGT, the non-standard-day correction (env_model, = 1 on an ISA day),
% training-mode derate, damage. Both engines are always running.
E = P.engine;
rho0 = P.const.rho0;
mach = V / atmo.a;

% rpm: first-order lag whose time constant grows with height
intake_t = atmo.T * (1 + 0.2 * mach^2);
rpm_max = min(max(1 - E.rpm_cold_k * (E.t_sea_level - intake_t), E.rpm_min_cold), 1.0);
target = E.idle_rpm + (rpm_max - E.idle_rpm) * min(max(throttle / E.rpm_max_dry_throttle, 0), 1);
density_ratio = min(max(atmo.rho / rho0, E.density_ratio_min), 1.0);
if target > rpm
    tau = E.spool_up;
else
    tau = E.spool_down;
end
tau = tau * density_ratio^(-E.spool_alt_exp);
rpm_new = rpm + (target - rpm) * min(max(dt / tau, 0), 1);
rpm_rel = rpm_new / rpm_max;

% afterburner: lights only at max dry rpm, and not below the lighting speed unless it is already lit
ab_range = max(E.ab_throttle_span - E.ab_full_zone, 0.02);
ab_target = 0;
if throttle > E.ab_throttle_start
    ab_target = E.ab_min + (1 - E.ab_min) * min(max((throttle - E.ab_throttle_start) / ab_range, 0), 1);
end
ab_target = ab_target * min(max((rpm_rel - E.ab_rpm_on) / E.ab_rpm_window, 0), 1);
ias_kmh = V * sqrt(atmo.rho / rho0) * 3.6;
if atmo.p < E.p_stratosphere
    light_min_kmh = E.ab_light_hi_kmh;
else
    light_min_kmh = E.ab_light_kmh;
end
if ab < E.ab_light_level && ias_kmh < light_min_kmh
    ab_target = 0;
end
ab_new = ab + (ab_target - ab) * min(max(dt / E.ab_tau, 0), 1);

% thrust lapse with density, ram gain with Mach, installed losses
sigma = atmo.rho / rho0;
ram = 1 - E.ram_cut_sl * min(max(sigma, 0), 1);
if sigma >= E.sigma_11km
    lapse_dry = sigma^E.lapse_dry;
    lapse_ab = sigma^E.lapse_ab;
else
    lapse_dry = E.sigma_11km^E.lapse_dry * sigma / E.sigma_11km;
    lapse_ab = E.sigma_11km^E.lapse_ab * sigma / E.sigma_11km;
end
strat = 1 + E.strat_boost * min(max((E.sigma_11km - sigma) / (E.sigma_11km - E.sigma_18km), 0), 1);
ram_weight = min(max((mach - E.thrust_low_mach_full) / max(E.thrust_low_mach_end - E.thrust_low_mach_full, 0.01), 0), 1);
power = E.thrust_low_scale + (E.thrust_scale - E.thrust_low_scale) * ram_weight;
lapse_dry = lapse_dry * strat * power;
lapse_ab = lapse_ab * strat * power;
mach_installed = min(max(mach / max(E.static_mach_end, 0.05), 0), 1);
installed = E.static_installed + (E.installed_thrust - E.static_installed) * mach_installed^2;
rpm_share = min(max((rpm_rel - E.rpm_share_floor) / E.rpm_share_span, 0), 1)^E.dry_rpm_exp;
dry = installed * E.dry_thrust * lapse_dry * (1 + E.dry_mach_gain * ram * mach) * rpm_share;
ab_thrust = installed * (E.ab_thrust - E.dry_thrust) * lapse_ab * (1 + E.ab_mach_gain * ram * mach) * ab_new;

% engine limiters: thrust tapers to half past the max indicated speed and max Mach
if ias_kmh > E.max_ias_kmh
    limit_ias = min(max(1 - (ias_kmh - E.max_ias_kmh) / E.limit_ias_span_kmh, E.limit_floor), 1);
else
    limit_ias = 1;
end
t2_max = E.t_stratosphere * (1 + 0.2 * E.max_mach^2);
mach_max = sqrt(max(0, (t2_max / atmo.T - 1) / 0.2));
if mach > mach_max
    limit_mach = min(max(1 - (mach - mach_max) / E.limit_mach_span, E.limit_floor), 1);
else
    limit_mach = 1;
end
limit = min(limit_ias, limit_mach);
dry = dry * limit;
ab_thrust = ab_thrust * limit;

% subsonic afterburner gain, faded out toward M 1.1
fade_span = E.ab_subsonic_mach_end - E.ab_subsonic_mach_full;
subsonic_fade = min(max((E.ab_subsonic_mach_end - mach) / fade_span, 0), 1);
ab_thrust = ab_thrust + E.ab_subsonic_gain * subsonic_fade * ram_weight * ab_new * (dry + ab_thrust);
thrust = dry + ab_thrust;
end

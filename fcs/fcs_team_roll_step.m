function p_new = fcs_team_roll_step(P, p_ref, stick_roll, mach, V, alpha, beta, qbar, Ixx, mass, dt, mx_tvc, tvc, hi)
% FCS_TEAM_ROLL_STEP  Roll-rate reference model: aileron moment against roll damping, plus extra damping near centre stick
% (the team EFM's RollProfile; cpp:690-723). Returns the roll rate (rad/s) after dt. It sets the rate at each speed, the
% stick feel, and how fast the rate builds and stops. Implicit in the damping, so it stays stable at the lowest speeds.
%   mx_tvc   nozzle roll moment, N m, + roll right (0 with TVC off); tvc: TVC on (0/1); hi: TVC roll share at high AoA
R = P.roll;
a_deg = alpha * 180 / pi;
aileron = stick_roll * R.team_aileron_max;
% subsonic roll gain, faded out from M 0.95 to 1.05
subsonic_gain = 1 + (R.roll_gain_subsonic - 1) * min(max((R.team_subsonic_mach_end - mach) / R.team_subsonic_mach_span, 0), 1);
% external stores lower the top roll rate
store_roll = 1 - R.roll_store_cut * min(max(P.mass.store_kg / max(R.roll_store_full_kg, 1), 0), 1);
clda_a = plant_table(R.team_aoa_k, R.team_clda, a_deg);
clp_a = plant_table(R.team_aoa_k, R.team_clp, a_deg);
if clda_a < 1
    clda_eff = clda_a + (1 - clda_a) * hi;
else
    clda_eff = clda_a;
end
if clp_a > 1
    clp_eff = clp_a - (clp_a - 1) * hi;
else
    clp_eff = clp_a;
end
beta_room = min(max((R.team_beta_fade_start - abs(beta)) / R.team_beta_fade_width, 0), 1);
Cl_da = store_roll * subsonic_gain * R.team_cl_da * plant_table(R.team_mach_k, R.team_mach_ail, mach) * clda_eff * aileron * beta_room;
damping = R.team_damp * clp_eff * R.team_b / (2 * max(V, R.team_v_min));
% centre stick brakes the roll at any rate (roll_stop = 1), like an F-16's roll-rate loop
stop_weight = R.roll_stop * min(max(1 - abs(stick_roll) / R.roll_stop_band, 0), 1);
centre_weight = max(min(max(1 - abs(p_ref) / R.team_centre_rate, 0), 1), stop_weight);
centre_mach = min(max(1 - 0.9 * (mach - R.team_centre_mach) / R.team_centre_mach_span, 0.1), 1);
centre_stick = min(max(1 - abs(stick_roll) * R.team_centre_stick, 0), 1);
centre_beta = min(max((R.team_centre_beta_start - abs(beta)) / R.team_centre_beta_width, 0), 1);
damping = damping + R.team_centre_damp * centre_mach * centre_weight * centre_stick * centre_beta;
mass_factor = min(max(1 - R.team_mass_k * (mass - R.team_mass_lo) / (R.team_mass_hi - R.team_mass_lo), R.team_mass_floor), 1);
k = qbar * R.team_S * R.team_b * mass_factor / max(Ixx, 1);   % heavier: slower to build, same steady rate
if tvc > 0.5
    limit = min(max(R.team_tvc_lim_hi - R.team_tvc_lim_drop * (mach - R.team_tvc_lim_mach0) / R.team_tvc_lim_mach_span, R.team_tvc_lim_lo), R.team_tvc_lim_hi);
    damp_hi = min(max(R.team_tvc_damp_hi - R.team_tvc_damp_k * mach / R.team_tvc_damp_mach, R.team_tvc_damp_lo), R.team_tvc_damp_hi);
    over = min(max((abs(p_ref) - limit) / (R.team_tvc_damp_span * limit), 0), 1);
    damping = damping + damp_hi * over;
end
p_new = (p_ref + dt * (k * Cl_da + mx_tvc * mass_factor / max(Ixx, 1))) / (1 + dt * k * damping);
end

function a0 = plant_aero_bare(P, alpha, beta, mach, p_hat, q_hat, r_hat, cfg, ge)
% PLANT_AERO_BARE  Bare airframe coefficients, no surface deflection (the mod's bare(), cpp:540-585). The FCS runs the
% same function as its onboard model for dynamic inversion, the plant runs it as the truth.
%   alpha, beta     rad; r_hat is + nose right
%   p_hat, q_hat, r_hat   rates x reference length / (2 V)
%   cfg             struct: gear_pos, airbrake_pos, flap_pos, flap_brake_pos (0..1)
%   ge              struct: phi (induced drag factor), dcl (lift increment factor), the ground effect
A = P.aero;
sin_beta = sin(beta);
cos_beta_sq = cos(beta) * cos(beta);
% the fins lose their grip past 20-60 deg of sideslip
fin = 1 - A.fin_loss * min(max((abs(beta) - A.fin_beta0) / A.fin_beta_width, 0), 1);

CL = plant_cl_of(P, alpha, mach) * cos_beta_sq;
cd0 = plant_table(A.mach_tab, A.cd0_tab, mach);
k_induced = plant_table(A.mach_tab, A.k_tab, mach);
stall = min(max((abs(alpha) - A.cd_stall_start) / A.cd_stall_width, 0), 1);
CL = CL * (1 + ge.dcl);
CD = cd0 + k_induced * CL^2 * (1 - stall) * ge.phi + stall * (A.cd_flat_plate * sin(alpha)^2) + A.cd_side * sin_beta^2;
config_drag = A.cd_gear * cfg.gear_pos + A.cd_airbrake * cfg.airbrake_pos + A.cd_flap * cfg.flap_pos ...
    + A.cd_flap_brake * cfg.flap_brake_pos;
CD = CD + config_drag;

% deep stall braking, faded in past 35 deg AoA
deep = min(max((abs(alpha) - A.deep_start) / A.deep_width, 0), 1);
CD = CD + deep * (A.cd_90 - A.cd_flat_plate) * sin(alpha)^2;
% flying tail first: blunt nozzles and fins meet the air
if abs(alpha) > pi / 2
    CD = CD + A.cd_tail_first * cos(alpha)^2;
end
% drag that goes with the lift past the linear range (normal force x sin alpha), faded in 20..28 deg
if A.cd_hi_k > 0
    fa = abs(alpha);
    if fa > pi
        fa = 2 * pi - fa;
    end
    normal_drag = A.cd_90 * sin(fa);
    if fa < 80 * pi / 180
        normal_drag = min(abs(CL) * tan(fa), normal_drag);
    end
    core = CD - cd0 - A.cd_side * sin_beta^2 - config_drag;
    weight = min(max((fa - A.cd_hi_lo) / max(A.cd_hi_hi - A.cd_hi_lo, 0.1 * pi / 180), 0), 1);
    CD = CD + A.cd_hi_k * weight * max(normal_drag * cos_beta_sq - core, 0);
end

CY = A.CY_beta * sin_beta;
% stability degrades at high AoA, pitch damping fades in the deep stall
hi = min(max((abs(alpha) - A.hi_start) / A.hi_width, 0), 1);
Cm = A.Cm_alpha * alpha * (1 - hi) + A.Cm_hi * hi * sin(alpha) + A.Cm_q * q_hat * (1 - A.Cm_q_hi_loss * hi);
Cl = A.Cl_beta * sin_beta * fin + A.Cl_p * p_hat + A.Cl_r * r_hat;
Cn = A.Cn_beta * sin_beta * fin * (1 - A.Cn_hi_loss * hi) + A.Cn_r * r_hat + A.Cn_p * p_hat;   % + nose right
a0 = struct('CL', CL, 'CD', CD, 'CY', CY, 'Cl', Cl, 'Cm', Cm, 'Cn', Cn);
end

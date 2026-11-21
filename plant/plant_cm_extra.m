function Cm = plant_cm_extra(P, mach, alpha, airbrake_pos)
% PLANT_CM_EXTRA  Transonic trim change, stiffer stability past M 0.9, and the dorsal airbrake's nose-up moment (cpp:523-527).
% The FCS feeds 85 % of it forward (Mz_need in fcs_allocation), the plant applies all of it.
A = P.aero;
t = min(max((mach - A.tuck_mach_start) / A.tuck_mach_width, 0), 1);
t = t * t * (3 - 2 * t);
alpha_clipped = min(max(alpha, -A.tuck_alpha_clip), A.tuck_alpha_clip);
Cm = A.Cm_tuck * t + A.Cm_tuck_alpha * t * alpha_clipped + A.Cm_airbrake * airbrake_pos;
end

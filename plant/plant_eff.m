function e = plant_eff(P, mach, alpha)
% PLANT_EFF  Stabilator effectiveness: transonic loss times AoA loss (cpp:725-731). The floor falls with AoA past 60 deg.
A = P.aero;
fa = abs(alpha);
if fa > pi
    fa = 2 * pi - fa;      % tail first (180 deg) counts as 0 for the tail
end
floor_a = A.eff_floor0 - A.eff_floor_drop * min(max((fa - pi / 3) / (pi / 6), 0), 1);
if fa > pi / 2
    floor_a = (A.eff_floor0 - A.eff_floor_drop) + A.eff_floor_drop * min(max((fa - 5 * pi / 6) / (pi / 6), 0), 1);
end
mach_loss = min(max(1 - A.eff_mach_loss * min(max((mach - A.eff_mach_start) / A.eff_mach_width, 0), 1), A.eff_mach_min), 1);
aoa_loss = min(max(A.eff_aoa_zero - fa / A.eff_aoa_slope, floor_a), 1);
e = mach_loss * aoa_loss;
end

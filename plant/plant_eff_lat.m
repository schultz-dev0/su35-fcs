function e = plant_eff_lat(P, mach, alpha)
% PLANT_EFF_LAT  Roll and yaw surface effectiveness (cpp:594): transonic loss times AoA loss, floor 0.25.
A = P.aero;
mach_loss = min(max(1 - A.eff_mach_loss * min(max((mach - A.eff_mach_start) / A.eff_mach_width, 0), 1), A.eff_mach_min), 1);
aoa_loss = min(max(A.eff_aoa_zero - abs(alpha) / A.eff_aoa_slope, A.eff_lat_floor), 1);
e = mach_loss * aoa_loss;
end

function CL = plant_cl_of(P, alpha, mach)
% PLANT_CL_OF  Lift coefficient through the stall: linear in alpha, then a flat plate past ~30 deg (cpp:528-536).
A = P.aero;
cla = plant_table(A.mach_tab, A.cla_tab, mach);
linear = cla * (alpha - A.alpha0);
flat_plate = A.cl_flat_plate * sin(2 * alpha);
blend = min(max((abs(alpha) - A.cl_blend_start) / A.cl_blend_width, 0), 1);
CL = (1 - blend) * linear + blend * flat_plate;
CL = min(max(CL, -A.cl_max), A.cl_max);
end

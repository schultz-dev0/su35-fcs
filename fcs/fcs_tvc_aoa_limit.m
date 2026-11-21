function limit_deg = fcs_tvc_aoa_limit(P, mach, w)
% FCS_TVC_AOA_LIMIT  AoA limit with TVC on, deg: the table for no nozzle thrust, a Mach curve for full (cpp:617-623).
L = P.alloc;
base = L.aoa_tab_scale * plant_table(L.aoa_tab_mach, L.aoa_tab_deg, mach);
full = L.aoa_ptr_scale * (L.aoa_ptr_a / (mach + L.aoa_ptr_mach0)^L.aoa_ptr_exp + L.aoa_ptr_b);
limit_deg = base + abs(full - base) * w;
end

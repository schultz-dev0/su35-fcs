function limit = fcs_tvc_nz_limit(P, mach, w, g_stores)
% FCS_TVC_NZ_LIMIT  g limit with TVC on: the table (cut by the stores), opened by the nozzle thrust share (cpp:644-648).
L = P.alloc;
limit = min(plant_table(L.pp_mach, L.ny_tab, mach), g_stores) + L.nz_tvc_open * w;
end

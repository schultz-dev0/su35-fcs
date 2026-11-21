function limit_deg = fcs_tvc_q_limit(P, mach, w)
% FCS_TVC_Q_LIMIT  Pitch rate limit with TVC on, deg/s (cpp:625-630).
L = P.alloc;
base = L.q_tab_scale * plant_table(L.pp_mach, L.q_tab_deg, mach);
full = L.q_ptr_scale * (L.q_ptr_a / (mach + L.q_ptr_mach0)^L.q_ptr_exp + L.q_ptr_b);
limit_deg = base + abs(full - base) * w;
end

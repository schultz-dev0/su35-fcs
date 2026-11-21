function w = fcs_tvc_limit_share(P, thrust, nozzle_pitch_rad, altitude)
% FCS_TVC_LIMIT_SHARE  How much of the thrust goes through the pitch nozzles, 0..1. It opens the AoA, rate and g
% limits (cpp:611-616).
L = P.alloc;
a = min(max((altitude - L.tvc_share_alt_lo) / L.tvc_share_alt_span, 0), 1);
lo = L.tvc_share_lo - L.tvc_share_lo_drop * a;
hi = L.tvc_share_hi - L.tvc_share_hi_drop * a;
w = min(max((thrust * sin(abs(nozzle_pitch_rad)) - lo) / (hi - lo), 0), 1);
end

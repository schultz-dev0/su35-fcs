function g = fcs_tvc_mach_gate(P, mach, stick_pitch, is_yaw)
% FCS_TVC_MACH_GATE  Share of nozzle travel by Mach: all off past M 1; pulling or level off from M 0.6 to 0.9, pushing
% from M 0.15 to 0.4 (yaw is not cut when pushing). Airborne form (cpp:605-610).
L = P.alloc;
if mach > L.gate_mach_off
    g = 0;
    return
end
if stick_pitch >= 0
    g = min(max((L.gate_pull_hi - mach) / L.gate_pull_span, 0), 1);
elseif is_yaw
    g = 1;
else
    g = min(max((L.gate_push_hi - mach) / L.gate_push_span, 0), 1);
end
end

function mx = fcs_tvc_roll_moment(P, stick_roll, stick_pitch, pedal, mach, nz, thrust_left, thrust_right)
% FCS_TVC_ROLL_MOMENT  TVC roll moment, N m, + = roll right, as the team's EFM source: each nozzle tilts by the pitch
% command, the roll command goes on one side only, both gated by Mach and g (cpp:669-686).
L = P.alloc;
if mach > 1.0
    mx = 0;
    return
end
if stick_pitch >= 0
    gate = min(max((L.gate_pull_hi - mach) / L.gate_pull_span, 0), 1);
    if nz > L.tr_nz_hi
        gate = gate * min(max(L.tr_pitch_gate + L.tr_pitch_gate * (nz - L.tr_nz_hi) / L.tr_nz_span, L.tr_pitch_gate), 1);
    else
        gate = gate * min(max(L.tr_pitch_gate - 0.25 * (L.tr_nz_hi - nz) / L.tr_nz_lo_span, 0.25), L.tr_pitch_gate);
    end
else
    gate = min(max((L.gate_push_hi - mach) / L.gate_push_span, 0), 1);
end
pitch_cmd = stick_pitch * gate;
pedal_share = 1 - 2 * abs(pedal) * (1 - min(max(L.ped_roll_mix, 0), 1));
roll_cmd = abs(stick_roll) * pedal_share * (1 - abs(stick_pitch)) * gate;
yaw_cmd = 2 * abs(pedal) * gate;
angle_roll_side = L.tr_nozzle * min(max(abs(pitch_cmd + yaw_cmd + roll_cmd), 0), 1);
angle_other = L.tr_nozzle * min(max(abs(pitch_cmd - yaw_cmd), 0), 1);
if stick_roll >= 0
    thrust_roll_side = thrust_right;
    thrust_other = thrust_left;
else
    thrust_roll_side = thrust_left;
    thrust_other = thrust_right;
end
mx = L.tvc_roll_arm_moment * (thrust_roll_side * sin(angle_roll_side) + thrust_other * sin(angle_other)) ...
    * stick_roll * (1 - abs(stick_pitch));
end

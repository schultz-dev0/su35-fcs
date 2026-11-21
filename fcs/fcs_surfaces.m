function surf = fcs_surfaces(P, alloc, st)
% FCS_SURFACES  The Surfaces bus (docs/interfaces.md) from the allocation's commands, which are in the mod's own signs.
%   stab: the mod's + is nose-up moment, the bus's + is trailing edge down, so the sign flips here.
%   rudder (+ nose right) and flaperon_diff (+ roll right) are the same as the mod's. stab_diff is not used (0).
%   flaperon_sym is the landing flap (15 deg times its position). le_flap is the slat schedule, shown without the
%   0.5 s lag: it is drawn only and has no effect on the forces (cpp:3912-3916).
%   Nozzle left/right pitch = pitch -/+ roll (the differential), both yaw nozzles = yaw.
C = P.act;
surf.stab = -alloc.stab;
surf.stab_diff = 0;
surf.flaperon_diff = alloc.flaperon;
surf.flaperon_sym = st.flap_pos * C.flap_landing_deg * pi / 180;
slat = min(max(st.alpha * C.le_flap_gain, 0), C.le_flap_max);
if st.mach >= C.le_flap_mach
    slat = slat * C.le_flap_mach_scale;
end
surf.rudder = alloc.rudder;
surf.le_flap = max(slat, st.gear_pos * C.le_flap_gear);
surf.nozzle_pitch_left = alloc.noz_pitch - alloc.noz_roll;
surf.nozzle_pitch_right = alloc.noz_pitch + alloc.noz_roll;
surf.nozzle_yaw_left = alloc.noz_yaw;
surf.nozzle_yaw_right = alloc.noz_yaw;
end

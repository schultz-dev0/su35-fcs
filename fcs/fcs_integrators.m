function [q_int, p_int, r_int, x] = fcs_integrators(P, cmd, st, x, dt)
% FCS_INTEGRATORS  Trim integrators of the three rate loops, one frame (cpp:3499-3530, reset 3874-3878).
% They trim out steady asymmetric moments. Each one freezes or bleeds in the cases the mod lists, so a roll in progress
% or an air start does not wind them up. Forward Euler, updated before use, as in the mod.
%   cmd   the rate commands from fcs_commands
% In Maneuver mode (man_w > 0.5) the mod zeroes them after the allocation; here that happens at the start of the next
% frame, which gives the same values.
R = P.roll;
Y = P.yaw;
F = P.pitch;

if x.man_w_last > 0.5
    x.q_int = 0;
    x.p_int = 0;
    x.r_int = 0;
end

x.q_int = min(max(x.q_int + (cmd.q_cmd - st.q) * dt, -F.q_int_limit), F.q_int_limit);

airborne_fast = st.V > R.int_v_min;
roll_int_ok = abs(cmd.sr) < R.int_stick_limit && (cmd.tvc_on > 0.5 || abs(st.p) < R.int_rate_limit);
yaw_int_ok = abs(cmd.sr) < R.int_stick_limit && abs(st.p) < R.int_rate_limit;

if airborne_fast
    % what was built up fades over 0.5 s while the loop is held off, instead of staying frozen
    if ~roll_int_ok
        x.p_int = x.p_int * exp(-dt / R.int_bleed_s);
    end
    if ~yaw_int_ok
        x.r_int = x.r_int * exp(-dt / R.int_bleed_s);
    end
    if roll_int_ok
        x.p_int = min(max(x.p_int + (cmd.p_cmd - st.p) * dt, -R.p_int_limit), R.p_int_limit);
    end
    if yaw_int_ok
        yaw_right = -st.r_yaw;
        x.r_int = min(max(x.r_int + (cmd.r_cmd - yaw_right) * dt, -Y.r_int_limit), Y.r_int_limit);
    end
else
    x.p_int = x.p_int * (1 - min(max(dt * R.int_ground_bleed, 0), 1));
    x.r_int = x.r_int * (1 - min(max(dt * R.int_ground_bleed, 0), 1));
end

x.man_w_last = cmd.man_w;
q_int = x.q_int;
p_int = x.p_int;
r_int = x.r_int;
end

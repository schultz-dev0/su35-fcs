function [surf_actual, v_new, noz_now] = plant_actuators_step(P, v, surf, aux, dt)
% PLANT_ACTUATORS_STEP  Rate limiters on the surfaces and nozzles (cpp:3888-3911). Works in the Surfaces bus's signs.
%   v = [stab flaperon rudder noz_pitch noz_yaw noz_roll] last frame's positions, rad (nozzle pitch is the mean of the
%   two, roll the half difference). The limits update before the plant reads them, as in the mod's simulate().
%   noz_now = [noz_pitch noz_yaw] after the step, for the FCS (it reads them one frame late).
C = P.act;
noz_pitch_cmd = 0.5 * (surf.nozzle_pitch_left + surf.nozzle_pitch_right);
noz_roll_cmd = 0.5 * (surf.nozzle_pitch_right - surf.nozzle_pitch_left);
noz_yaw_cmd = 0.5 * (surf.nozzle_yaw_left + surf.nozzle_yaw_right);
stab = step_toward(v(1), surf.stab, C.stab_rate * dt);
flaperon = step_toward(v(2), surf.flaperon_diff, C.flaperon_rate * dt);
rudder = step_toward(v(3), surf.rudder, C.rudder_rate * dt);
noz_pitch = step_toward(v(4), noz_pitch_cmd, aux.noz_rate_pitch * dt);
noz_yaw = step_toward(v(5), noz_yaw_cmd, aux.noz_rate_other * dt);
noz_roll = step_toward(v(6), noz_roll_cmd, aux.noz_rate_other * dt);
surf_actual.stab = stab;
surf_actual.stab_diff = surf.stab_diff;
surf_actual.flaperon_diff = flaperon;
surf_actual.flaperon_sym = surf.flaperon_sym;
surf_actual.rudder = rudder;
surf_actual.le_flap = surf.le_flap;
surf_actual.nozzle_pitch_left = noz_pitch - noz_roll;
surf_actual.nozzle_pitch_right = noz_pitch + noz_roll;
surf_actual.nozzle_yaw_left = noz_yaw;
surf_actual.nozzle_yaw_right = noz_yaw;
v_new = [stab; flaperon; rudder; noz_pitch; noz_yaw; noz_roll];
noz_now = [noz_pitch; noz_yaw];
end

function x = step_toward(x, target, limit)
x = x + min(max(target - x, -limit), limit);
end

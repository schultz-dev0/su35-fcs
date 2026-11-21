function [force, moment, nz_ny] = plant_step_forces(P, st, surf, aux)
% PLANT_STEP_FORCES  Forces and moments for the 6-DOF block, aerospace axes, gravity included.
% Wraps plant_forces_moments (DCS axes, no gravity). Conversions, DCS (x forward, y up, z right) to aerospace
% (x forward, y right, z down):
%   force  = [Fx, Fz_dcs, -Fy_dcs]        moment = [Mx, Mz_dcs, -My_dcs]
% The 6-DOF Euler block has no gravity, so the weight is added here: m g [-sin(theta), cos(theta) sin(phi),
% cos(theta) cos(phi)] in aerospace body axes. nz_ny = [nz ny_lat] in g, from the non-gravity forces.
air.V = st.V;
air.alpha = st.alpha;
air.beta = st.beta;
air.p = st.p;
air.q = st.q;
air.r_yaw = st.r_yaw;
air.mach = st.mach;
air.qbar = st.qbar;
air.ias_kmh = st.V * sqrt(st.rho / P.const.rho0) * 3.6;
air.height = st.altitude;   % flat ground at sea level
air.mass = st.mass;
config.gear_pos = st.gear_pos;
config.airbrake_pos = st.airbrake_pos;
config.flap_pos = st.flap_pos;
config.flap_brake_pos = st.flap_brake_pos;
% the mod's stabilator is positive nose up; the Surfaces bus is positive trailing edge down
act.stab = -surf.stab;
act.flaperon = surf.flaperon_diff;
act.rudder = surf.rudder;
act.noz_pitch = 0.5 * (surf.nozzle_pitch_left + surf.nozzle_pitch_right);
act.noz_roll = 0.5 * (surf.nozzle_pitch_right - surf.nozzle_pitch_left);
act.noz_yaw = 0.5 * (surf.nozzle_yaw_left + surf.nozzle_yaw_right);
[f, m, nz, ny_lat] = plant_forces_moments(P, air, config, act, aux, st.thrust_left, st.thrust_right);
weight = st.mass * P.const.g;
gravity = weight * [-sin(st.theta); cos(st.theta) * sin(st.phi); cos(st.theta) * cos(st.phi)];
force = [f.x; f.z; -f.y] + gravity;
moment = [m.x; m.z; -m.y];
nz_ny = [nz; ny_lat];
end

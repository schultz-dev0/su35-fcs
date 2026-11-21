function [init, info] = trim_level_flight(speed, altitude, gear_down, P, fixed)
% TRIM_LEVEL_FLIGHT  Level, wings-level, hands-off trim for the rebuilt model, and the air start for it.
%   speed       true airspeed, m/s
%   altitude    m above sea level
%   gear_down   0 or 1 (gear and landing flaps down)
%   P           parameter struct (default plant_get_params())
%   fixed       optional struct with alpha (rad) and throttle: skip the solve and build the air start at that point
%               (used to start the rebuild from the reference harness's own trim; the force balance is then not exact)
% Unknowns are the angle of attack and the throttle; the two equations are the forces along the body x and z axes.
% The surfaces are what the FCS allocation commands at that state, so the trim includes the control laws. The
% pitch moment balance is the FCS's job (dynamic inversion), the trim does not solve for it.
% init is what make_rebuild_model starts from; info has the trim point for printing.
if nargin < 4
    P = plant_get_params();
end
atmo = trim_atmosphere(altitude);
unknown = [0.05; 0.6];
max_iterations = 40;
if nargin >= 5
    unknown = [fixed.alpha; fixed.throttle];
    max_iterations = 0;
end
for iteration = 1:max_iterations
    residual = trim_residual(unknown, speed, altitude, gear_down, atmo, P);
    jacobian = zeros(2, 2);
    for k = 1:2
        step = zeros(2, 1);
        step(k) = 1e-5;
        jacobian(:, k) = (trim_residual(unknown + step, speed, altitude, gear_down, atmo, P) - residual) / 1e-5;
    end
    delta = -jacobian \ residual;
    delta(1) = min(max(delta(1), -0.05), 0.05);
    delta(2) = min(max(delta(2), -0.2), 0.2);
    unknown = unknown + delta;
    unknown(2) = min(max(unknown(2), 0.05), 0.99);
    if norm(residual) < 1
        break
    end
end
[residual, point] = trim_residual(unknown, speed, altitude, gear_down, atmo, P);
alpha = unknown(1);
theta = alpha;
st = point.st;

init.altitude = altitude;
init.V = speed;
init.alpha = alpha;
init.theta = theta;
init.xme_0 = [0; 0; -altitude];
init.Vm_0 = [speed * cos(alpha); 0; speed * sin(alpha)];
init.eul_0 = [0; theta; 0];
init.pm_0 = [0; 0; 0];
init.mass = st.mass;
init.inertia = diag([st.Ixx, st.Izz, st.Iyy]);
init.nz_ny = [point.nz; 0];
init.engine = point.engine;
init.engine_thrust = [st.thrust_left, st.thrust_right];
init.slow = point.slow;
init.slow_in = [st.mach; alpha; point.nz; st.qbar];
init.fcs = point.fcs;
init.actuators = point.actuators;
init.noz = [0; 0];
init.throttle = unknown(2);
info.alpha_deg = alpha * 180 / pi;
info.throttle = unknown(2);
info.thrust_total = st.thrust_left + st.thrust_right;
info.stab_deg = point.surf.stab * 180 / pi;
info.nz = point.nz;
info.residual = residual;
info.mach = st.mach;
info.ias_kmh = speed * sqrt(atmo(4) / P.const.rho0) * 3.6;
end

function [residual, point] = trim_residual(unknown, speed, altitude, gear_down, atmo, P)
alpha = unknown(1);
throttle = unknown(2);
dt = P.dt;
% engines at the steady state for this throttle: run them until they settle
air.T = atmo(1);
air.a = atmo(2);
air.p = atmo(3);
air.rho = atmo(4);
rpm = 0.9;
ab = 0;
thrust = 0;
for k = 1:1500
    [rpm, ab, thrust] = plant_engine_step(P, throttle, rpm, ab, air, speed, 0.05);
end
slow = [double(gear_down); double(gear_down); 0; 0; 0; 0];

st.V = speed;
st.alpha = alpha;
st.beta = 0;
st.p = 0;
st.q = 0;
st.r_yaw = 0;
st.phi = 0;
st.theta = alpha;
st.psi = 0;
st.nz = 1;
st.ny_lat = 0;
st.mach = speed / atmo(2);
st.rho = atmo(4);
st.qbar = 0.5 * atmo(4) * speed^2;
st.altitude = altitude;
[st.mass, st.Ixx, st.Iyy, st.Izz] = plant_mass_properties(P);
st.gear_pos = slow(1);
st.flap_pos = slow(2);
st.flap_brake_pos = 0;
st.airbrake_pos = 0;
st.thrust_left = thrust;
st.thrust_right = thrust;

pilot.stick_pitch = 0;
pilot.stick_roll = 0;
pilot.pedals = 0;
pilot.throttle_left = throttle;
pilot.throttle_right = throttle;
pilot.gear_cmd = double(gear_down);
pilot.airbrake_cmd = 0;
pilot.tvc_on = 0;
pilot.limiter_off = 0;
pilot.refuel_probe = 0;

x = fcs_state_init();
x.pitch_free_t = 1;
x.roll_free_t = 1;
[cmd, x] = fcs_commands(P, pilot, st, 0, x, dt);
[q_int, p_int, r_int, x] = fcs_integrators(P, cmd, st, x, dt);
qd = P.pitch.q_bandwidth * (cmd.q_cmd - st.q) + P.pitch.q_int_gain * q_int;
pd = P.roll.p_bandwidth * (cmd.p_cmd - st.p) + P.roll.p_int_gain * p_int;
rd = P.yaw.r_bandwidth * (cmd.r_cmd + st.r_yaw) + P.yaw.r_int_gain * r_int + cmd.r_coord_dot;
act.noz_pitch = 0;
act.noz_yaw = 0;
[alloc, aux, x] = fcs_allocation(P, cmd, st, qd, pd, rd, act, ab, x, dt);
surf = fcs_surfaces(P, alloc, st);
[force, ~, nz_ny] = plant_step_forces(P, st, surf, aux);
residual = [force(1); force(3)];

point.st = st;
point.nz = nz_ny(1);
point.engine = [rpm; rpm; ab; ab];
point.slow = slow;
point.fcs = fcs_state_pack(x);
point.surf = surf;
point.actuators = [surf.stab; surf.flaperon_diff; surf.rudder; 0; 0; 0];
end

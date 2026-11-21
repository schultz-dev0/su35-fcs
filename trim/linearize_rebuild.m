function lin = linearize_rebuild(model, init)
% LINEARIZE_REBUILD  Linear models of the rebuilt jet about a hands-off level-flight trim, open and closed loop.
%   model   name of the built model (make_rebuild_model returns it)
%   init    the struct trim_level_flight returned; the model must have been built from it
% Both come from the same operating point, a snapshot 10 s into the hands-off run. Returns discrete state-space
% systems (Ts = dt_fcs) and their poles in rad/s (continuous equivalent):
%   closed_pitch   pitch stick   -> [V alpha q theta nz]
%   closed_lat     roll stick    -> [beta p r_yaw phi]
%   open_pitch     stabilator command -> same pitch outputs (control laws cut at the Surfaces bus)
%   open_lat       aileron and rudder commands -> same lateral outputs
% The closed loop uses the numerical-perturbation algorithm on the whole model (root ports from
% add_linearization_ports). The default block-by-block algorithm gives a fast unstable lateral pair
% (about +32 and +6 1/s) that neither the simulation nor a finite-difference Jacobian of the simulation has: see
% docs/rebuild.md. The open loop has no control laws in it and the default algorithm is fine there.
load_system(model);
dt = evalin('base', 'dt_fcs');
snapshot = 10;
state_channels = struct('V', 1, 'alpha', 2, 'beta', 3, 'p', 4, 'q', 5, 'r_yaw', 6, 'phi', 7, 'theta', 8, 'nz', 10);
pitch_outputs = [state_channels.V, state_channels.alpha, state_channels.q, state_channels.theta, state_channels.nz];
lat_outputs = [state_channels.beta, state_channels.p, state_channels.r_yaw, state_channels.phi];

surface_point = linio([model '/Surfaces_builder'], 1, 'openinput');
state_point = linio([model '/State_builder'], 1, 'output');
block_options = linearizeOptions('RateConversionMethod', 'tustin');
open_all = linearize(model, [surface_point; state_point], snapshot, block_options);
lin.open_pitch = reduce_outputs(open_all, pitch_outputs);
lin.open_lat = reduce_outputs(open_all, lat_outputs);

% the root ports exist only in memory: the model is reloaded from disk afterwards
add_linearization_ports(model);
operating_point = findop(model, snapshot);
perturb_options = linearizeOptions('LinearizationAlgorithm', 'numericalpert', 'RateConversionMethod', 'zoh');
closed_all = linearize(model, operating_point, perturb_options);
close_system(model, 0);
load_system(model);
% closed_all: inputs stick pitch, roll, pedals; outputs V alpha beta p q r_yaw phi theta nz
% The stick dead zone makes the stick-to-state gains zero for a small perturbation, so the closed-loop modes are not
% picked by what a stick reaches. They are sorted by which states carry them (below).
lin.closed = closed_all;
lin.dt = dt;
lin.modes = classify_modes(closed_all, dt);
lin.poles = struct();
lin.poles.closed_pitch = [lin.modes([lin.modes.group] == "longitudinal").s].';
lin.poles.closed_lat = [lin.modes([lin.modes.group] == "lateral").s].';
lin.poles.open_pitch = continuous_poles(lin.open_pitch, dt);
lin.poles.open_lat = continuous_poles(lin.open_lat, dt);
end

function modes = classify_modes(sys, dt)
% Each mode of the closed loop with its pole (rad/s) and the share of its eigenvector on longitudinal states
% (u, w, theta, q) and lateral states (v, phi, p, r). The control laws couple the two a little, so a mode is
% "longitudinal" or "lateral" when one share is more than twice the other and "mixed" otherwise.
names = string(sys.StateName);
longitudinal = contains(names, 'ub,vb,wb(1)') | contains(names, 'ub,vb,wb(3)') | contains(names, 'psi(2)') | ...
    contains(names, 'p,q,r (2)');
lateral = contains(names, 'ub,vb,wb(2)') | contains(names, 'psi(1)') | contains(names, 'p,q,r (1)') | ...
    contains(names, 'p,q,r (3)');
[vectors, z] = eig(sys.A);
z = diag(z);
modes = struct('s', {}, 'group', {}, 'long_share', {}, 'lat_share', {});
for k = 1:numel(z)
    weights = abs(vectors(:, k));
    long_share = norm(weights(longitudinal));
    lat_share = norm(weights(lateral));
    group = "mixed";
    if long_share > 2 * lat_share
        group = "longitudinal";
    elseif lat_share > 2 * long_share
        group = "lateral";
    end
    pole_s = log(z(k)) / dt;
    if abs(pole_s) < 1e3
        modes(end + 1) = struct('s', pole_s, 'group', group, 'long_share', long_share, 'lat_share', lat_share); %#ok<AGROW>
    end
end
end

function s = continuous_poles(sys, dt)
% z poles to s = ln(z) / dt, without the fast numerical ones (the delay states of the tustin conversion)
z = pole(sys);
s = log(z) / dt;
s = s(abs(s) < 1000);
end

function sys = reduce_outputs(sys, channels)
% Keep the wanted outputs, then drop states that neither input nor these outputs can see
sys = sys(channels, :);
sys = minreal(sys, 1e-6, false);
end

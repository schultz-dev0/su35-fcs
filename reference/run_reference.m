% Runs the reference model once and returns the logged signals in named fields.
%
%   result = run_reference(cfg, pilot, t_end)
%
% cfg     from xfm_default_config
% pilot   timeseries from pilot_timeseries (default: hands off, throttle 0.5)
% t_end   run length, s
%
% Each run loads a fresh copy of the flight model library (see README), so runs do not affect each other.
function result = run_reference(cfg, pilot, t_end)
    arguments
        cfg struct
        pilot timeseries = pilot_timeseries.hold()
        t_end (1,1) double = 20
    end
    reference_dir = fileparts(mfilename("fullpath"));
    addpath(fullfile(reference_dir, "build"));   % the S-function MEX
    model = "xfm_reference";
    if ~isfile(fullfile(reference_dir, model + ".slx"))
        make_reference_model();
    end
    load_system(fullfile(reference_dir, model + ".slx"));

    pilot_with_gear = pilot;
    if cfg.gear_down
        pilot_with_gear.Data(:, 6) = 1;   % the gear lever follows the air start
    end

    input = Simulink.SimulationInput(model);
    input = input.setVariable("xfm_cfg", cfg);
    input = input.setVariable("pilot_ts", pilot_with_gear);
    input = input.setModelParameter(StopTime = num2str(t_end, "%.6f"));
    run = sim(input);
    result = unpack_outputs(run, cfg);
end

function result = unpack_outputs(run, cfg)
    names = ["state", "derived", "force", "moment", "internals", "draw", "mass", "pilot", "euler", "pqr_dot"];
    logged = run.yout;
    data = struct();
    for k = 1:numel(names)
        element = logged{k};   % element k is Outport k, in the order make_reference_model creates them
        data.(names(k)) = squeeze(element.Values.Data);
        if size(data.(names(k)), 1) ~= numel(run.tout)
            data.(names(k)) = data.(names(k)).';
        end
    end
    % Row 1 is the state at t = 0, before the flight model has run once: its outputs are all zero. Copy row 2 up
    % so the first sample is usable (6 ms later, same state to within a step).
    for k = 1:numel(names)
        data.(names(k))(1, :) = data.(names(k))(2, :);
    end
    data.mass(1, :) = data.mass(2, :);
    result.t = run.tout;
    result.cfg = cfg;

    state = data.state;                       % DCS axes: vb(3), w(3), euler(3), pos N/up/E (3), world vel (3)
    result.vb = state(:, 1:3);
    result.w = state(:, 4:6);                 % wx roll, wy yaw (+ nose left), wz pitch (+ nose up)
    result.roll = state(:, 7);
    result.pitch = state(:, 8);
    result.heading = state(:, 9);
    result.pos = state(:, 10:12);
    result.vel_world = state(:, 13:15);
    result.altitude = state(:, 11);

    derived = data.derived;
    result.tas = derived(:, 1);
    result.alpha = derived(:, 2);
    result.beta = derived(:, 3);
    result.mach = derived(:, 4);
    result.qbar = derived(:, 5);
    result.nz = derived(:, 6);
    result.ny = derived(:, 7);

    result.force = data.force;                % N, DCS body axes at the CG
    result.moment = data.moment;              % N m, DCS body axes about the CG
    result.mass = data.mass(:, 1);
    result.inertia = data.mass(:, 2:4);       % Ixx roll, Iyy yaw, Izz pitch
    result.pilot = data.pilot;
    result.euler = data.euler;
    result.pqr_dot = data.pqr_dot;
    result.draw = data.draw;

    internal = data.internals;
    result.stab_deg = internal(:, 1);         % symmetric stabilator, + trailing edge down
    result.stab_diff_deg = internal(:, 2);
    result.flaperon_deg = internal(:, 3);
    result.rudder_deg = internal(:, 4);
    result.le_flap_deg = internal(:, 5);
    result.nozzle_pitch_deg = internal(:, 6);
    result.nozzle_yaw_deg = internal(:, 7);
    result.nozzle_roll_deg = internal(:, 8);
    result.thrust_left = internal(:, 9);
    result.thrust_right = internal(:, 10);
    result.rpm_left = internal(:, 11);
    result.rpm_right = internal(:, 12);
    result.ab_left = internal(:, 13);
    result.ab_right = internal(:, 14);
    result.gear_pos = internal(:, 15);
    result.flap_pos = internal(:, 16);
    result.airbrake_pos = internal(:, 17);
    result.fuel_kg = internal(:, 20);
    result.internals = internal;
end

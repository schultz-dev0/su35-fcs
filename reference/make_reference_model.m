% Builds reference/xfm_reference.slx: the mod's flight model and FCS in a Simulink loop.
%
%   make_reference_model
%
% Simulink does what DCS does: it integrates the 6-DOF motion from the force and moment the flight model returns,
% and feeds the new state back in. The flight model (xfm_sfun) works in DCS axes (x forward, y up, z right); the
% Aerospace Blockset 6-DOF block works in aerospace axes (x forward, y right, z down). The two meet at gain blocks:
%   DCS -> aerospace   [x; y; z] -> [x; z; -y]    (matrix dcs_to_aero)
%   aerospace -> DCS   [x; y; z] -> [x; -z; y]    (matrix aero_to_dcs, the transpose)
% The same matrices convert forces, moments, velocities, rates and the position. A rate about DCS y (up, + nose
% left) is minus the aerospace yaw rate r; a rate about DCS z (right, + nose up) is the aerospace pitch rate q.
% Euler angles are the aerospace ones: roll + right wing down, pitch + nose up, yaw = heading from north. The
% world is north / up / east in DCS, north / east / down in the 6-DOF block.
%
% Workspace variables the model reads (run_reference.m sets them for each run):
%   xfm_cfg    from xfm_default_config: paths, mass data, air start state
%   pilot_ts   timeseries of the 10 Pilot channels (docs/interfaces.md), see pilot_timeseries.m
%
% Not modelled: ground contact (airborne only: the model has no ground, a run that goes below the ground just
% keeps falling), wind, the cockpit parameters (autopilot, switches), damage. Mass is fixed at its air start value
% in the 6-DOF block (fuel burn over a run is under 1 %); the flight model itself sees the burning mass.
function make_reference_model()
    reference_dir = fileparts(mfilename("fullpath"));
    addpath(fullfile(reference_dir, "build"));   % the S-function MEX
    model = "xfm_reference";
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    % The S-function block can only create its ports once its parameters evaluate, so the workspace needs them now.
    assignin("base", "xfm_cfg", xfm_default_config());
    assignin("base", "pilot_ts", pilot_timeseries.hold());
    new_system(model);
    open_system(model);

    dcs_to_aero = "[1 0 0; 0 0 1; 0 -1 0]";
    aero_to_dcs = "[1 0 0; 0 0 -1; 0 1 0]";

    pilot = add_block("simulink/Sources/From Workspace", model + "/Pilot", ...
        VariableName = "pilot_ts", SampleTime = "xfm_cfg.dt", Interpolate = "off", ...
        OutputAfterFinalValue = "Holding final value", Position = [30 40 120 80]);

    xfm = add_block("simulink/User-Defined Functions/S-Function", model + "/XFM flight model", ...
        FunctionName = "xfm_sfun", ...
        Parameters = "xfm_cfg.library, xfm_cfg.config_dir, xfm_cfg.run_dir, xfm_cfg.settings, xfm_cfg.dt, xfm_cfg.engines", ...
        Position = [330 60 470 260]);

    % forces and moments: DCS axes at the flight model, aerospace axes at the 6-DOF block
    force_to_aero = add_gain(model, "Force to aero axes", dcs_to_aero, [540 40 620 70]);
    moment_to_aero = add_gain(model, "Moment to aero axes", dcs_to_aero, [540 100 620 130]);

    % Weight in body axes: DCM (earth -> body) times [0; 0; m g] (down)
    weight_down = add_block("simulink/Sources/Constant", model + "/Weight down", ...
        Value = "[0; 0; xfm_cfg.mass0 * xfm_cfg.ic.g]", Position = [540 180 620 210]);
    weight_body = add_block("simulink/Math Operations/Product", model + "/Weight in body axes", ...
        Inputs = "2", Multiplication = "Matrix(*)", Position = [660 170 700 230]);
    total_force = add_block("simulink/Math Operations/Sum", model + "/Total force", ...
        Inputs = "++", Position = [740 40 770 100]);

    dof = add_block("aerolib6dof/6DoF (Euler Angles)", model + "/6-DOF (aerospace axes)", ...
        xme_0 = "xfm_cfg.ic.xe0", Vm_0 = "xfm_cfg.ic.vb0_aero", eul_0 = "xfm_cfg.ic.eul0", ...
        pm_0 = "xfm_cfg.ic.pqr0", Mass = "xfm_cfg.mass0", Inertia = "xfm_cfg.inertia_aero", ...
        Position = [830 40 1000 260]);

    % state back to DCS axes. 6-DOF outputs: 1 Ve, 2 Xe, 3 Euler, 4 DCM, 5 Vb, 6 p q r, 7 pdot
    vb_dcs = add_gain(model, "Body velocity to DCS", aero_to_dcs, [1060 150 1140 180]);
    rates_dcs = add_gain(model, "Body rates to DCS", aero_to_dcs, [1060 210 1140 240]);
    position_dcs = add_gain(model, "Position to DCS", aero_to_dcs, [1060 90 1140 120]);
    velocity_dcs = add_gain(model, "World velocity to DCS", aero_to_dcs, [1060 30 1140 60]);

    state = add_block("simulink/Signal Routing/Mux", model + "/State (DCS axes)", ...
        Inputs = "5", Position = [1200 40 1210 260]);
    altitude = add_block("simulink/Signal Routing/Selector", model + "/Altitude", ...
        IndexMode = "One-based", Indices = "2", InputPortWidth = "3", Position = [1200 300 1250 330]);
    atmosphere = add_block("aerolibatmos/ISA Atmosphere Model", model + "/ISA atmosphere", ...
        Position = [1290 290 1390 350]);
    air = add_block("simulink/Signal Routing/Mux", model + "/Air", ...
        Inputs = "4", Position = [1430 290 1440 350]);

    % wiring
    connect(model, "Pilot/1", "XFM flight model/1");
    connect(model, "XFM flight model/1", "Force to aero axes/1");
    connect(model, "XFM flight model/2", "Moment to aero axes/1");
    connect(model, "Weight down/1", "Weight in body axes/2");
    connect(model, "6-DOF (aerospace axes)/4", "Weight in body axes/1");
    connect(model, "Force to aero axes/1", "Total force/1");
    connect(model, "Weight in body axes/1", "Total force/2");
    connect(model, "Total force/1", "6-DOF (aerospace axes)/1");
    connect(model, "Moment to aero axes/1", "6-DOF (aerospace axes)/2");

    connect(model, "6-DOF (aerospace axes)/1", "World velocity to DCS/1");
    connect(model, "6-DOF (aerospace axes)/2", "Position to DCS/1");
    connect(model, "6-DOF (aerospace axes)/5", "Body velocity to DCS/1");
    connect(model, "6-DOF (aerospace axes)/6", "Body rates to DCS/1");
    connect(model, "Body velocity to DCS/1", "State (DCS axes)/1");
    connect(model, "Body rates to DCS/1", "State (DCS axes)/2");
    connect(model, "6-DOF (aerospace axes)/3", "State (DCS axes)/3");
    connect(model, "Position to DCS/1", "State (DCS axes)/4");
    connect(model, "World velocity to DCS/1", "State (DCS axes)/5");
    connect(model, "State (DCS axes)/1", "XFM flight model/2");

    connect(model, "Position to DCS/1", "Altitude/1");
    connect(model, "Altitude/1", "ISA atmosphere/1");
    % Non-standard day: the ISA temperature plus a constant offset (xfm_cfg.temperature_offset_k) and the ISA
    % pressure times a constant ratio (xfm_cfg.pressure_ratio). 0 and 1 give the standard day. The speed of sound follows the square root of the temperature
    % and the density follows the gas law, rho = p / (R T).
    add_block("simulink/Sources/Constant", model + "/Temperature offset", ...
        Value = "xfm_cfg.temperature_offset_k", Position = [1290 380 1370 400]);
    add_block("simulink/Math Operations/Sum", model + "/Offset temperature", Inputs = "++", ...
        Position = [1410 380 1440 410]);
    add_block("simulink/Math Operations/Product", model + "/Temperature ratio", Inputs = "*/", ...
        Position = [1470 380 1500 410]);
    add_block("simulink/Math Operations/Math Function", model + "/Root of ratio", Operator = "sqrt", ...
        Position = [1530 380 1560 410]);
    add_block("simulink/Math Operations/Product", model + "/Offset speed of sound", Inputs = "**", ...
        Position = [1590 380 1620 410]);
    add_block("simulink/Math Operations/Gain", model + "/Offset pressure", Gain = "xfm_cfg.pressure_ratio", ...
        Position = [1410 440 1440 470]);
    add_block("simulink/Math Operations/Gain", model + "/Gas constant", Gain = "287.05", ...
        Position = [1470 440 1500 470]);
    add_block("simulink/Math Operations/Product", model + "/Offset density", Inputs = "*/", ...
        Position = [1530 440 1560 470]);
    connect(model, "ISA atmosphere/3", "Offset pressure/1");
    connect(model, "ISA atmosphere/1", "Offset temperature/1");
    connect(model, "Temperature offset/1", "Offset temperature/2");
    connect(model, "Offset temperature/1", "Temperature ratio/1");
    connect(model, "ISA atmosphere/1", "Temperature ratio/2");
    connect(model, "Temperature ratio/1", "Root of ratio/1");
    connect(model, "ISA atmosphere/2", "Offset speed of sound/1");
    connect(model, "Root of ratio/1", "Offset speed of sound/2");
    connect(model, "Offset temperature/1", "Gas constant/1");
    connect(model, "Offset pressure/1", "Offset density/1");
    connect(model, "Gas constant/1", "Offset density/2");
    connect(model, "Offset temperature/1", "Air/1");
    connect(model, "Offset speed of sound/1", "Air/2");
    connect(model, "Offset pressure/1", "Air/3");
    connect(model, "Offset density/1", "Air/4");

    connect(model, "Air/1", "XFM flight model/3");

    % logged signals (Outport blocks, saved as the Dataset yout)
    outputs = {
        "state", "State (DCS axes)/1"
        "derived", "XFM flight model/6"
        "force", "XFM flight model/1"
        "moment", "XFM flight model/2"
        "internals", "XFM flight model/4"
        "draw", "XFM flight model/5"
        "mass", "XFM flight model/3"
        "pilot", "Pilot/1"
        "euler", "6-DOF (aerospace axes)/3"
        "pqr_dot", "6-DOF (aerospace axes)/7"
        };
    for k = 1:size(outputs, 1)
        name = outputs{k, 1};
        add_block("simulink/Sinks/Out1", model + "/" + name, Port = string(k), Position = [1500 40 + 50 * k 1530 60 + 50 * k]);
        connect(model, outputs{k, 2}, name + "/1");
    end

    set_param(model, Solver = "ode4", SolverType = "Fixed-step", FixedStep = "xfm_cfg.dt", ...
        StopTime = "20", SaveOutput = "on", OutputSaveName = "yout", SaveFormat = "Dataset", ...
        SaveTime = "on", TimeSaveName = "tout", ReturnWorkspaceOutputs = "on", ...
        SignalLogging = "off", SimulationMode = "normal");

    save_system(model, fullfile(reference_dir, model + ".slx"));
    close_system(model, 0);
    fprintf("make_reference_model: wrote %s\n", fullfile(reference_dir, model + ".slx"));
end

function block = add_gain(model, name, matrix_text, position)
    block = add_block("simulink/Math Operations/Gain", model + "/" + name, Gain = matrix_text, ...
        Multiplication = "Matrix(K*u)", Position = position);
end

% "Block A/2" -> "Block B/1": output port 2 of block A to input port 1 of block B
function connect(model, from, to)
    [from_block, from_port] = split_port(from);
    [to_block, to_port] = split_port(to);
    add_line(model, from_block + "/" + from_port, to_block + "/" + to_port, autorouting = "smart");
end

function [block, port] = split_port(text)
    parts = split(text, "/");
    block = join(parts(1:end - 1), "/");
    port = parts(end);
end

function model = make_rebuild_model(init)
% MAKE_REBUILD_MODEL  Builds models/su35_rebuild.slx: the mod's flight control system and airframe as blocks.
%
%   Pilot (workspace timeseries) -> FCS -> actuators -> plant forces -> 6-DOF (Aerospace Blockset) -> State -> FCS
%
% init is the air start, from trim/trim_level_flight (fields used here):
%   altitude (m), V (m/s), alpha (rad), theta (rad), engine = [rpm_l rpm_r ab_l ab_r], slow = gear/flap states,
%   actuators = [stab flaperon rudder noz_pitch noz_yaw noz_roll] (Surfaces signs), fcs = FCS state vector.
% The workspace variable su35_pilot (a timeseries, see rebuild_pilot) drives the Pilot bus.
% The build puts P (the flight model's parameters) and the bus definitions in the base workspace.
%
% Layout of the diagram:
%   Pilot            From Workspace, split into the Pilot bus
%   FCS/Commands     fcs_commands: the three rate commands, one MATLAB Function (state in, state out)
%   FCS/Integrators  fcs_integrators: the trim integrators
%   FCS/Loops        Simulink blocks: rate error x bandwidth + integrator, per channel (qd, pd, rd)
%   FCS/Allocation   fcs_allocation: dynamic inversion, surfaces first then nozzles, and the Maneuver mode
%   FCS/Surfaces     the Surfaces bus (interface signs)
%   Actuators        rate limiters
%   Plant            forces and moments, engines, gear and flaps, ISA atmosphere, 6-DOF, State builder
dt = 0.006;
model = 'su35_rebuild';
P = plant_get_params();
assignin('base', 'P', P);
assignin('base', 'dt_fcs', dt);
assignin('base', 'su35_init', init);
make_buses();

if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);
open_system(model);
load_system('aerolib6dof');
load_system('aerolibatmos');

%% Pilot
add_block('simulink/Sources/From Workspace', [model '/Pilot_input'], 'VariableName', 'su35_pilot', ...
    'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', 'SampleTime', 'dt_fcs');
add_block('simulink/Signal Routing/Demux', [model '/Pilot_split'], 'Outputs', '10');
add_block('simulink/Signal Routing/Bus Creator', [model '/Pilot'], 'Inputs', '10', 'OutDataTypeStr', 'Bus: Pilot');
add_line(model, 'Pilot_input/1', 'Pilot_split/1');
pilot_names = {'stick_pitch', 'stick_roll', 'pedals', 'throttle_left', 'throttle_right', 'gear_cmd', ...
    'airbrake_cmd', 'tvc_on', 'limiter_off', 'refuel_probe'};
for k = 1:numel(pilot_names)
    line = add_line(model, sprintf('Pilot_split/%d', k), sprintf('Pilot/%d', k));
    set_param(line, 'Name', pilot_names{k});
end

%% Plant blocks that make the State
add_mf(model, 'State_builder', ...
    {'function st = f(Vb, eul, pqr, xe, atmo, nz_ny, slow, thrust)', ...
     'P = coder.const(plant_get_params());', ...
     'st = plant_make_state(P, Vb, eul, pqr, xe, atmo, nz_ny, slow, thrust);', 'end'}, ...
    struct('st', 'Bus: State'), dt);
add_block('aerolib6dof/6DoF (Euler Angles)', [model '/Sixdof'], ...
    'xme_0', 'su35_init.xme_0', 'Vm_0', 'su35_init.Vm_0', 'eul_0', 'su35_init.eul_0', 'pm_0', 'su35_init.pm_0', ...
    'Mass', 'su35_init.mass', 'Inertia', 'su35_init.inertia');
add_block('aerolibatmos/ISA Atmosphere Model', [model '/ISA']);
add_block('simulink/Signal Routing/Mux', [model '/Atmo'], 'Inputs', '4');
add_block('simulink/Math Operations/Gain', [model '/Altitude_from_down'], 'Gain', '-1');
add_block('simulink/Signal Routing/Selector', [model '/Down'], 'InputPortWidth', '3', 'Indices', '3', 'IndexMode', 'One-based');
add_block('simulink/Sinks/Terminator', [model '/Unused_ve']);
add_block('simulink/Sinks/Terminator', [model '/Unused_dcm']);
add_block('simulink/Sinks/Terminator', [model '/Unused_dwb']);
add_block('simulink/Discrete/Unit Delay', [model '/nz_ny_delay'], 'InitialCondition', 'su35_init.nz_ny', 'SampleTime', 'dt_fcs');

add_mf(model, 'Engines', ...
    {'function [thrust, ab_level, v_new] = f(v, pilot, Vb, atmo)', ...
     'P = coder.const(plant_get_params());', ...
     '[thrust, ab_level, v_new] = plant_engines_step(P, v, pilot, Vb, atmo, 0.006);', 'end'}, ...
    struct('pilot', 'Bus: Pilot'), dt);
add_block('simulink/Discrete/Unit Delay', [model '/Engines_state'], 'InitialCondition', 'su35_init.engine', 'SampleTime', 'dt_fcs');

add_mf(model, 'Slow_systems', ...
    {'function [slow_now, v_new] = f(v, pilot, st_prev)', ...
     'P = coder.const(plant_get_params());', ...
     '[slow_now, v_new] = plant_slow_vec_step(P, v, pilot, st_prev, 0.006);', 'end'}, ...
    struct('pilot', 'Bus: Pilot'), dt);
add_block('simulink/Discrete/Unit Delay', [model '/Slow_state'], 'InitialCondition', 'su35_init.slow', 'SampleTime', 'dt_fcs');
add_block('simulink/Discrete/Unit Delay', [model '/Slow_inputs_delay'], 'InitialCondition', 'su35_init.slow_in', 'SampleTime', 'dt_fcs');
add_mf(model, 'Slow_inputs', ...
    {'function y = f(st)', 'y = [st.mach; st.alpha; st.nz; st.qbar];', 'end'}, struct('st', 'Bus: State'), dt);

%% FCS
add_mf(model, 'Commands', ...
    {'function [cmd, x_new] = f(pilot, st, noz, x_vec)', ...
     'P = coder.const(plant_get_params());', ...
     'x = fcs_state_unpack(x_vec);', ...
     '[cmd, x] = fcs_commands(P, pilot, st, noz(1), x, 0.006);', ...
     'x_new = fcs_state_pack(x);', 'end'}, ...
    struct('pilot', 'Bus: Pilot', 'st', 'Bus: State', 'cmd', 'Bus: FcsCmd'), dt);
add_mf(model, 'Integrators', ...
    {'function [q_int, p_int, r_int, x_new] = f(cmd, st, x_vec)', ...
     'P = coder.const(plant_get_params());', ...
     'x = fcs_state_unpack(x_vec);', ...
     '[q_int, p_int, r_int, x] = fcs_integrators(P, cmd, st, x, 0.006);', ...
     'x_new = fcs_state_pack(x);', 'end'}, ...
    struct('cmd', 'Bus: FcsCmd', 'st', 'Bus: State'), dt);
add_mf(model, 'Allocation', ...
    {'function [alloc, aux, x_new] = f(cmd, st, qd, pd, rd, noz, ab_level, x_vec)', ...
     'P = coder.const(plant_get_params());', ...
     'x = fcs_state_unpack(x_vec);', ...
     'act.noz_pitch = noz(1);', ...
     'act.noz_yaw = noz(2);', ...
     '[alloc, aux, x] = fcs_allocation(P, cmd, st, qd, pd, rd, act, ab_level, x, 0.006);', ...
     'x_new = fcs_state_pack(x);', 'end'}, ...
    struct('cmd', 'Bus: FcsCmd', 'st', 'Bus: State', 'alloc', 'Bus: AllocOut', 'aux', 'Bus: Aux'), dt);
add_block('simulink/Discrete/Unit Delay', [model '/FCS_state'], 'InitialCondition', 'su35_init.fcs', 'SampleTime', 'dt_fcs');
add_mf(model, 'Surfaces_builder', ...
    {'function surf = f(alloc, st)', ...
     'P = coder.const(plant_get_params());', ...
     'surf = fcs_surfaces(P, alloc, st);', 'end'}, ...
    struct('alloc', 'Bus: AllocOut', 'st', 'Bus: State', 'surf', 'Bus: Surfaces'), dt);

% the loops, in Simulink blocks: desired acceleration = bandwidth x rate error + trim integrator
add_block('simulink/Signal Routing/Bus Selector', [model '/Cmd_rates'], 'OutputSignals', 'q_cmd,p_cmd,r_cmd,r_coord_dot');
add_block('simulink/Signal Routing/Bus Selector', [model '/State_rates'], 'OutputSignals', 'q,p,r_yaw');
loops = {'Pitch', 'Roll', 'Yaw'};
bandwidth = {'P.pitch.q_bandwidth', 'P.roll.p_bandwidth', 'P.yaw.r_bandwidth'};
int_gain = {'P.pitch.q_int_gain', 'P.roll.p_int_gain', 'P.yaw.r_int_gain'};
for k = 1:3
    name = loops{k};
    add_block('simulink/Math Operations/Sum', [model '/' name '_error'], 'Inputs', '+-');
    add_block('simulink/Math Operations/Gain', [model '/' name '_bandwidth'], 'Gain', bandwidth{k});
    add_block('simulink/Math Operations/Gain', [model '/' name '_integrator_gain'], 'Gain', int_gain{k});
    if k == 3
        add_block('simulink/Math Operations/Sum', [model '/' name '_accel'], 'Inputs', '+++');
    else
        add_block('simulink/Math Operations/Sum', [model '/' name '_accel'], 'Inputs', '++');
    end
    add_line(model, [name '_error/1'], [name '_bandwidth/1']);
    add_line(model, [name '_bandwidth/1'], [name '_accel/1']);
    add_line(model, [name '_integrator_gain/1'], [name '_accel/2']);
end
% yaw rate error is against yaw to the right, which is -r_yaw: r_cmd - (-r_yaw) = r_cmd + r_yaw
set_param([model '/Yaw_error'], 'Inputs', '++');

%% Actuators and the plant forces
add_mf(model, 'Actuators', ...
    {'function [surf_actual, v_new, noz_now] = f(v, surf, aux)', ...
     'P = coder.const(plant_get_params());', ...
     '[surf_actual, v_new, noz_now] = plant_actuators_step(P, v, surf, aux, 0.006);', 'end'}, ...
    struct('surf', 'Bus: Surfaces', 'aux', 'Bus: Aux', 'surf_actual', 'Bus: Surfaces'), dt);
add_block('simulink/Discrete/Unit Delay', [model '/Actuator_state'], 'InitialCondition', 'su35_init.actuators', 'SampleTime', 'dt_fcs');
add_block('simulink/Discrete/Unit Delay', [model '/Nozzle_delay'], 'InitialCondition', 'su35_init.noz', 'SampleTime', 'dt_fcs');
add_mf(model, 'Forces', ...
    {'function [force, moment, nz_ny] = f(st, surf, aux)', ...
     'P = coder.const(plant_get_params());', ...
     '[force, moment, nz_ny] = plant_step_forces(P, st, surf, aux);', 'end'}, ...
    struct('st', 'Bus: State', 'surf', 'Bus: Surfaces', 'aux', 'Bus: Aux'), dt);

%% Wiring
w = @(src, dst) add_line(model, src, dst, 'autorouting', 'smart');

% 6-DOF and atmosphere
w('Forces/1', 'Sixdof/1');
w('Forces/2', 'Sixdof/2');
w('Sixdof/1', 'Unused_ve/1');
w('Sixdof/4', 'Unused_dcm/1');
w('Sixdof/7', 'Unused_dwb/1');
w('Sixdof/2', 'Down/1');
w('Down/1', 'Altitude_from_down/1');
w('Altitude_from_down/1', 'ISA/1');
for k = 1:4
    w(sprintf('ISA/%d', k), sprintf('Atmo/%d', k));
end
% State builder
w('Sixdof/5', 'State_builder/1');
w('Sixdof/3', 'State_builder/2');
w('Sixdof/6', 'State_builder/3');
w('Sixdof/2', 'State_builder/4');
w('Atmo/1', 'State_builder/5');
w('nz_ny_delay/1', 'State_builder/6');
w('Slow_systems/1', 'State_builder/7');
w('Engines/1', 'State_builder/8');
w('Forces/3', 'nz_ny_delay/1');
% engines
w('Engines_state/1', 'Engines/1');
w('Pilot/1', 'Engines/2');
w('Sixdof/5', 'Engines/3');
w('Atmo/1', 'Engines/4');
w('Engines/3', 'Engines_state/1');
% slow systems
w('Slow_state/1', 'Slow_systems/1');
w('Pilot/1', 'Slow_systems/2');
w('Slow_inputs_delay/1', 'Slow_systems/3');
w('Slow_systems/2', 'Slow_state/1');
w('State_builder/1', 'Slow_inputs/1');
w('Slow_inputs/1', 'Slow_inputs_delay/1');
% FCS commands, integrators, loops, allocation
w('Pilot/1', 'Commands/1');
w('State_builder/1', 'Commands/2');
w('Nozzle_delay/1', 'Commands/3');
w('FCS_state/1', 'Commands/4');
w('Commands/1', 'Integrators/1');
w('State_builder/1', 'Integrators/2');
w('Commands/2', 'Integrators/3');
w('Commands/1', 'Cmd_rates/1');
w('State_builder/1', 'State_rates/1');
w('Cmd_rates/1', 'Pitch_error/1');
w('State_rates/1', 'Pitch_error/2');
w('Integrators/1', 'Pitch_integrator_gain/1');
w('Pitch_accel/1', 'Allocation/3');
w('Cmd_rates/2', 'Roll_error/1');
w('State_rates/2', 'Roll_error/2');
w('Integrators/2', 'Roll_integrator_gain/1');
w('Roll_accel/1', 'Allocation/4');
w('Cmd_rates/3', 'Yaw_error/1');
w('State_rates/3', 'Yaw_error/2');
w('Integrators/3', 'Yaw_integrator_gain/1');
w('Cmd_rates/4', 'Yaw_accel/3');
w('Yaw_accel/1', 'Allocation/5');
w('Commands/1', 'Allocation/1');
w('State_builder/1', 'Allocation/2');
w('Nozzle_delay/1', 'Allocation/6');
w('Engines/2', 'Allocation/7');
w('Integrators/4', 'Allocation/8');
w('Allocation/3', 'FCS_state/1');
% surfaces, actuators, forces
w('Allocation/1', 'Surfaces_builder/1');
w('State_builder/1', 'Surfaces_builder/2');
w('Actuator_state/1', 'Actuators/1');
w('Surfaces_builder/1', 'Actuators/2');
w('Allocation/2', 'Actuators/3');
w('Actuators/2', 'Actuator_state/1');
w('Actuators/3', 'Nozzle_delay/1');
w('State_builder/1', 'Forces/1');
w('Actuators/1', 'Forces/2');
w('Allocation/2', 'Forces/3');
% signal logging on the three bus signals (the logged names are used by analysis/)
log_signal(model, 'State_builder', 1, 'State');
log_signal(model, 'Actuators', 1, 'Surfaces_actual');
log_signal(model, 'Commands', 1, 'Commands');

set_param(model, 'Solver', 'ode4', 'SolverType', 'Fixed-step', 'FixedStep', 'dt_fcs', 'StopTime', 'su35_stop_time', ...
    'SaveOutput', 'off', 'SaveTime', 'off', 'SaveState', 'off', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
assignin('base', 'su35_stop_time', 20);
try
    Simulink.BlockDiagram.arrangeSystem(model);
catch
    % arranging is cosmetic
end
save_system(model, fullfile(fileparts(mfilename('fullpath')), model));
end

function log_signal(model, block, port, name)
% Turns on logging for one output port, under the given name (read back from out.logsout).
handles = get_param([model '/' block], 'PortHandles');
set_param(handles.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

function add_mf(model, name, script_lines, bus_types, sample_time)
% Adds a MATLAB Function block with the given script. bus_types maps a data name (input or output) to 'Bus: X'.
path = [model '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', path);
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', path);
chart.Script = strjoin(script_lines, newline);
chart.SampleTime = num2str(sample_time);
data = chart.find('-isa', 'Stateflow.Data');
for k = 1:numel(data)
    if isfield(bus_types, data(k).Name)
        data(k).DataType = bus_types.(data(k).Name);
    end
end
end

function make_buses()
% Bus definitions, one element per field, in the order the functions assign them (a struct must match its bus's order). The templates give the names; the values are placeholders.
pilot = struct('stick_pitch', 0, 'stick_roll', 0, 'pedals', 0, 'throttle_left', 0, 'throttle_right', 0, ...
    'gear_cmd', 0, 'airbrake_cmd', 0, 'tvc_on', 0, 'limiter_off', 0, 'refuel_probe', 0);
make_bus('Pilot', pilot);
state_names = {'V', 'alpha', 'beta', 'p', 'q', 'r_yaw', 'phi', 'theta', 'psi', 'nz', 'ny_lat', 'mach', 'rho', ...
    'qbar', 'altitude', 'mass', 'Ixx', 'Iyy', 'Izz', 'gear_pos', 'flap_pos', 'flap_brake_pos', 'airbrake_pos', ...
    'thrust_left', 'thrust_right'};
make_bus('State', zero_struct(state_names));
surface_names = {'stab', 'stab_diff', 'flaperon_diff', 'flaperon_sym', 'rudder', 'le_flap', 'nozzle_pitch_left', ...
    'nozzle_pitch_right', 'nozzle_yaw_left', 'nozzle_yaw_right'};
make_bus('Surfaces', zero_struct(surface_names));
command_names = {'sp', 'sr', 'sy', 'q_cmd', 'p_cmd', 'r_cmd', 'r_coord_dot', 'lat_fb', 'tvc_ok', 'tvc_on', ...
    'tvc_stores', 'man_w', 'nz_top', 'aoa_lim', 'ias_kmh', 'sin_gamma'};
make_bus('FcsCmd', zero_struct(command_names));
make_bus('AllocOut', zero_struct({'stab', 'flaperon', 'rudder', 'noz_pitch', 'noz_yaw', 'noz_roll'}));
make_bus('Aux', zero_struct({'tvc_ok', 'man_w', 'yaw_arm', 'noz_rate_pitch', 'noz_rate_other'}));
end

function s = zero_struct(names)
s = struct();
for k = 1:numel(names)
    s.(names{k}) = 0;
end
end

function results = compare_with_reference(tag)
% COMPARE_WITH_REFERENCE  Overlays the rebuild (plant/ + fcs/) on the reference harness (reference/, the mod's C++).
%   results = compare_with_reference(tag)
% tag only names the output table file (table_<tag>.txt), so a run before and after a fix can be kept side by side.
% Two flight conditions at 3000 m, speeds are indicated (IAS): 600 km/h clean and 320 km/h gear down.
% Both models start from the same state: the harness is trimmed with its own find_level_trim, then the rebuild is
% started at that angle of attack, speed and throttle, with the harness's engines started from the rebuild's settled
% engine state so neither side spends its first seconds spooling. Mass, inertia and centre of gravity come from the
% installed mod via the harness (cfg); the rebuild's parameters hold the same mass.
% Scenarios per condition: hands off 20 s; pitch doublet +-0.3 for 1 s each; roll step 0.5 for 2 s and full for 2 s;
% pedal step 0.3 and full for 3 s. PNGs and the table go to analysis/compare/out/.
if nargin < 1
    tag = 'run';
end
here = fileparts(mfilename('fullpath'));
out_dir = fullfile(here, 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
root = fileparts(fileparts(here));
addpath(fullfile(root, 'reference'));
P = plant_get_params();

conditions = struct('name', {'clean_600', 'gear_320'}, 'ias_kmh', {600, 320}, 'gear_down', {false, true});
scenarios = scenario_list();
results = struct();
table_lines = {};
for k = 1:numel(conditions)
    condition = conditions(k);
    fprintf('\n=== %s: %d km/h IAS, 3000 m, gear %s ===\n', condition.name, condition.ias_kmh, ...
        ternary(condition.gear_down, 'down', 'up'));
    [cfg, throttle] = reference_trim(condition, out_dir);
    [rebuild_trim, rebuild_info] = trim_level_flight(cfg.ic.tas_ms, 3000, double(condition.gear_down), P);
    print_trim_difference(cfg, throttle, rebuild_info);

    fixed = struct('alpha', cfg.ic.alpha, 'throttle', throttle);
    init = trim_level_flight(cfg.ic.tas_ms, 3000, double(condition.gear_down), P, fixed);
    % The harness's gear-down air start leaves the landing flaps up (xfm_ref_start_gear_down sets only the gear), and they
    % extend over 4 s while the jet flies. DCS would start with them down. The rebuild starts the same way here so the
    % flap transient is compared too; it is not a difference between the models.
    if condition.gear_down
        init.slow(2) = 0;
    end
    % harness engines start where the rebuild's engines are settled for this throttle
    cfg.engines = [1, init.engine(1), init.engine(2), init.engine(3), init.engine(4), ...
        init.engine_thrust(1), init.engine_thrust(2)];
    model = make_rebuild_model(init);

    for s = 1:numel(scenarios)
        scenario = scenarios(s);
        time = (0:0.006:scenario.t_end)';
        inputs = scenario.inputs(time);
        reference = run_harness(cfg, time, inputs, throttle, condition.gear_down);
        rebuild = run_rebuild(model, time, inputs, throttle, condition.gear_down);
        [stats, signals] = difference_table(reference, rebuild);
        key = [condition.name '_' scenario.name];
        results.(key) = struct('stats', stats, 'reference', reference, 'rebuild', rebuild);
        plot_overlay(reference, rebuild, signals, key, fullfile(out_dir, [key '.png']));
        table_lines = [table_lines, format_table(key, signals, stats)]; %#ok<AGROW>
    end
end
save(fullfile(out_dir, ['results_' tag '.mat']), 'results');
fid = fopen(fullfile(out_dir, ['table_' tag '.txt']), 'w');
fprintf(fid, '%s\n', table_lines{:});
fclose(fid);
fprintf('%s\n', table_lines{:});
end

function scenarios = scenario_list()
step = @(time, t0, t1, value) value * double(time >= t0 & time < t1);
scenarios = struct('name', {'hands_off', 'pitch_doublet', 'roll_half', 'roll_full', 'pedal_03', 'pedal_full'}, ...
    't_end', {20, 10, 8, 8, 10, 10}, ...
    'inputs', {@(time) struct(), ...
    @(time) struct('stick_pitch', step(time, 2, 3, 0.3) + step(time, 3, 4, -0.3)), ...
    @(time) struct('stick_roll', step(time, 2, 4, 0.5)), ...
    @(time) struct('stick_roll', step(time, 2, 4, 1.0)), ...
    @(time) struct('pedals', step(time, 2, 5, 0.3)), ...
    @(time) struct('pedals', step(time, 2, 5, 1.0))});
end

function [cfg, throttle] = reference_trim(condition, out_dir)
% find_level_trim runs about 20 harness runs, so keep the result
file = fullfile(out_dir, ['harness_trim_' condition.name '.mat']);
if isfile(file)
    saved = load(file);
    cfg = saved.cfg;
    throttle = saved.throttle;
    return
end
[cfg, throttle] = find_level_trim(altitude_m = 3000, ias_kmh = condition.ias_kmh, gear_down = condition.gear_down);
save(file, 'cfg', 'throttle');
end

function print_trim_difference(cfg, throttle, info)
fprintf('trim at the same IAS (TAS %.2f m/s):\n', cfg.ic.tas_ms);
fprintf('  harness: alpha %.2f deg, throttle lever %.3f\n', cfg.ic.alpha * 180 / pi, throttle);
fprintf('  rebuild: alpha %.2f deg, throttle %.3f, stab %.2f deg (TE down), thrust %.1f kN\n', info.alpha_deg, ...
    info.throttle, info.stab_deg, info.thrust_total / 1000);
end

function result = run_harness(cfg, time, inputs, throttle, gear_down)
values = zeros(numel(time), 10);
names = {'stick_pitch', 'stick_roll', 'pedals'};
for k = 1:numel(names)
    if isfield(inputs, names{k})
        values(:, k) = inputs.(names{k});
    end
end
values(:, 4) = throttle;
values(:, 5) = throttle;
values(:, 6) = double(gear_down);
pilot = pilot_timeseries.make(time, values);
raw = run_reference(cfg, pilot, time(end));
result.t = raw.t;
result.alpha = raw.alpha * 180 / pi;
result.beta = raw.beta * 180 / pi;
result.p = raw.w(:, 1) * 180 / pi;
result.q = raw.w(:, 3) * 180 / pi;
result.r = raw.w(:, 2) * 180 / pi;
result.nz = raw.nz;
result.phi = raw.roll * 180 / pi;
result.theta = raw.pitch * 180 / pi;
result.V = raw.tas;
result.altitude = raw.altitude;
% the flight model's `stab` is + nose-up moment, which is a trailing edge UP: the rebuild's Surfaces.stab is + TE down.
% (reference/README.md calls it + trailing edge down; the signs here and the matching forces show it is not.)
result.stab = -raw.stab_deg;
result.flaperon = raw.flaperon_deg;
result.rudder = raw.rudder_deg;
result.noz_pitch = raw.nozzle_pitch_deg;
result.noz_yaw = raw.nozzle_yaw_deg;
result.thrust = (raw.thrust_left + raw.thrust_right) / 1000;
result.raw = raw;
end

function result = run_rebuild(model, time, inputs, throttle, gear_down)
inputs.gear_cmd = double(gear_down) * ones(size(time));
rebuild_pilot(time, inputs, throttle);
out = sim(model, 'StopTime', num2str(time(end)));
state = out.logsout.getElement('State').Values;
surfaces = out.logsout.getElement('Surfaces_actual').Values;
t = state.V.Time;
result.t = t;
result.alpha = state.alpha.Data * 180 / pi;
result.beta = state.beta.Data * 180 / pi;
result.p = state.p.Data * 180 / pi;
result.q = state.q.Data * 180 / pi;
result.r = state.r_yaw.Data * 180 / pi;
result.nz = state.nz.Data;
result.phi = state.phi.Data * 180 / pi;
result.theta = state.theta.Data * 180 / pi;
result.V = state.V.Data;
result.altitude = state.altitude.Data;
result.stab = surfaces.stab.Data * 180 / pi;
result.flaperon = surfaces.flaperon_diff.Data * 180 / pi;
result.rudder = surfaces.rudder.Data * 180 / pi;
result.noz_pitch = 0.5 * (surfaces.nozzle_pitch_left.Data + surfaces.nozzle_pitch_right.Data) * 180 / pi;
result.noz_yaw = 0.5 * (surfaces.nozzle_yaw_left.Data + surfaces.nozzle_yaw_right.Data) * 180 / pi;
result.thrust = (state.thrust_left.Data + state.thrust_right.Data) / 1000;
end

function [stats, signals] = difference_table(reference, rebuild)
signals = {'alpha', 'beta', 'p', 'q', 'r', 'nz', 'phi', 'theta', 'V', 'altitude', 'stab', 'flaperon', 'rudder', ...
    'noz_pitch', 'noz_yaw', 'thrust'};
stats = zeros(numel(signals), 3);   % max abs difference, RMS difference, RMS of the reference signal about its start
for k = 1:numel(signals)
    name = signals{k};
    mine = interp1(rebuild.t, rebuild.(name), reference.t, 'linear', 'extrap');
    % the harness's first sample is a copy of its second (no flight model output yet), so start at sample 3
    valid = 3:numel(reference.t);
    difference = mine(valid) - reference.(name)(valid);
    spread = reference.(name)(valid) - reference.(name)(3);
    stats(k, :) = [max(abs(difference)), sqrt(mean(difference .^ 2)), sqrt(mean(spread .^ 2))];
end
end

function lines = format_table(key, signals, stats)
lines = {sprintf('--- %s: max |diff|, rms diff, rms of the reference change', key)};
for k = 1:numel(signals)
    lines{end + 1} = sprintf('  %-10s %10.4f %10.4f %10.4f', signals{k}, stats(k, 1), stats(k, 2), stats(k, 3)); %#ok<AGROW>
end
end

function plot_overlay(reference, rebuild, signals, title_text, file)
fig = figure('Visible', 'off', 'Position', [50 50 1400 900]);
for k = 1:numel(signals)
    subplot(4, 4, k);
    plot(reference.t, reference.(signals{k}), 'b', 'LineWidth', 1.2);
    hold on;
    plot(rebuild.t, rebuild.(signals{k}), 'r--', 'LineWidth', 1.0);
    title(strrep(signals{k}, '_', ' '));
    grid on;
    if k == 1
        legend('reference', 'rebuild', 'Location', 'best');
    end
end
sgtitle(strrep(title_text, '_', ' '));
exportgraphics(fig, file, 'Resolution', 80);
close(fig);
end

function text = ternary(condition, yes, no)
if condition
    text = yes;
else
    text = no;
end
end

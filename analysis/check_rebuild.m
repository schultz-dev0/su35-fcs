function check_rebuild()
% CHECK_REBUILD  Runs the rebuilt model: hands-off level flight, pitch, roll and pedal steps, closed-loop poles.
% Two trim points at 3000 m: 600 km/h clean and 320 km/h gear down (speeds are indicated, IAS). Prints the numbers and
% writes PNGs to analysis/out/. Run from anywhere with the repo folders on the path (see README).
here = fileparts(mfilename('fullpath'));
out_dir = fullfile(here, 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
P = plant_get_params();
cases = struct('name', {'clean_600', 'gear_320'}, 'speed_kmh', {600, 320}, 'gear_down', {0, 1}, ...
    'pitch_step', {0.3, 0.2}, 'roll_step', {0.5, 0.5}, 'pedal_step', {0.3, 0.3});
for k = 1:numel(cases)
    run_case(cases(k), P, out_dir);
end
end

function run_case(c, P, out_dir)
fprintf('\n=== %s: %d km/h IAS, 3000 m, gear %s ===\n', c.name, c.speed_kmh, ternary(c.gear_down, 'down', 'up'));
tas = trim_ias_to_tas(c.speed_kmh, 3000, P);
[init, info] = trim_level_flight(tas, 3000, c.gear_down, P);
fprintf('trim: alpha %.2f deg, throttle %.3f, thrust %.1f kN\n', info.alpha_deg, info.throttle, ...
    info.thrust_total / 1000);
model = make_rebuild_model(init);

% hands-off: nothing should move
hands_off = run_pilot(model, init, 20, struct());
st = hands_off.State;
fprintf('hands-off 20 s: V %.2f -> %.2f m/s, altitude %.1f -> %.1f m, alpha %.2f -> %.2f deg, nz %.3f -> %.3f\n', ...
    st.V.Data(1), st.V.Data(end), st.altitude.Data(1), st.altitude.Data(end), ...
    st.alpha.Data(1) * 180 / pi, st.alpha.Data(end) * 180 / pi, st.nz.Data(1), st.nz.Data(end));
plot_run(hands_off, {'V', 'altitude', 'alpha', 'nz'}, [1, 1, 180 / pi, 1], {'V m/s', 'altitude m', 'alpha deg', 'nz'}, ...
    [c.name ' hands off'], fullfile(out_dir, [c.name '_hands_off.png']));

t_step = 2;
step_time = (0:0.006:8)';
on = double(step_time >= t_step);

pitch = run_pilot(model, init, 8, struct('stick_pitch', c.pitch_step * on, 'time', step_time));
roll = run_pilot(model, init, 8, struct('stick_roll', c.roll_step * on, 'time', step_time));
pedal = run_pilot(model, init, 8, struct('pedals', c.pedal_step * on, 'time', step_time));

st = pitch.State;
fprintf('pitch step %.2f: peak nz %.2f, alpha %.2f -> peak %.2f deg, q at +3 s %.2f deg/s\n', c.pitch_step, ...
    max(st.nz.Data), st.alpha.Data(1) * 180 / pi, max(st.alpha.Data) * 180 / pi, at_time(st.q, t_step + 3) * 180 / pi);
plot_run(pitch, {'nz', 'alpha', 'q', 'theta'}, [1, 180 / pi, 180 / pi, 180 / pi], ...
    {'nz', 'alpha deg', 'q deg/s', 'theta deg'}, [c.name ' pitch step'], fullfile(out_dir, [c.name '_pitch_step.png']));

st = roll.State;
fprintf('roll step %.2f: p at +1 s %.1f deg/s, at +3 s %.1f deg/s, bank at +3 s %.1f deg, peak |beta| %.2f deg\n', ...
    c.roll_step, at_time(st.p, t_step + 1) * 180 / pi, at_time(st.p, t_step + 3) * 180 / pi, ...
    at_time(st.phi, t_step + 3) * 180 / pi, max(abs(st.beta.Data)) * 180 / pi);
plot_run(roll, {'p', 'phi', 'beta', 'r_yaw'}, [180 / pi, 180 / pi, 180 / pi, 180 / pi], ...
    {'p deg/s', 'bank deg', 'beta deg', 'r deg/s'}, [c.name ' roll step'], fullfile(out_dir, [c.name '_roll_step.png']));

st = pedal.State;
fprintf('pedal step %.2f: peak beta %.2f deg, beta at +3 s %.2f deg, peak yaw rate %.2f deg/s, peak bank %.2f deg\n', ...
    c.pedal_step, max(abs(st.beta.Data)) * 180 / pi, at_time(st.beta, t_step + 3) * 180 / pi, ...
    max(abs(st.r_yaw.Data)) * 180 / pi, max(abs(st.phi.Data)) * 180 / pi);
plot_run(pedal, {'beta', 'r_yaw', 'p', 'phi'}, [180 / pi, 180 / pi, 180 / pi, 180 / pi], ...
    {'beta deg', 'r deg/s', 'p deg/s', 'bank deg'}, [c.name ' pedal step'], fullfile(out_dir, [c.name '_pedal_step.png']));

% poles: the model is rebuilt with a hands-off pilot first, linearize_rebuild reads su35_pilot from the base workspace
rebuild_pilot([0; 60], struct('gear_cmd', init.slow(1) * [1; 1]), init.throttle);
lin = linearize_rebuild(model, init);
print_poles(lin);
plot_poles(lin, [c.name ' poles'], fullfile(out_dir, [c.name '_poles.png']));
end

function result = run_pilot(model, init, stop_time, inputs)
if isfield(inputs, 'time')
    time = inputs.time;
    inputs = rmfield(inputs, 'time');
else
    time = (0:0.006:stop_time)';
end
% the gear lever follows the gear the air start has, or the gear-down cases retract it
inputs.gear_cmd = init.slow(1) * ones(size(time));
rebuild_pilot(time, inputs, init.throttle);
out = sim(model, 'StopTime', num2str(stop_time));
result.State = out.logsout.getElement('State').Values;
result.Commands = out.logsout.getElement('Commands').Values;
end

function value = at_time(series, t)
index = find(series.Time >= t, 1);
value = series.Data(index);
end

function plot_run(run, names, scales, labels, title_text, file)
fig = figure('Visible', 'off', 'Position', [100 100 800 700]);
for k = 1:numel(names)
    subplot(numel(names), 1, k);
    series = run.State.(names{k});
    plot(series.Time, series.Data * scales(k));
    ylabel(labels{k});
    grid on;
    if k == 1
        title(title_text);
    end
end
xlabel('time, s');
exportgraphics(fig, file, 'Resolution', 110);
close(fig);
end

function print_poles(lin)
fprintf('closed-loop modes, rad/s (continuous equivalent; |s| < 40), sorted by the states that carry them\n');
modes = lin.modes;
[~, order] = sort(abs([modes.s]));
for k = order
    s = modes(k).s;
    if abs(s) >= 40 || imag(s) < -1e-6
        continue
    end
    if imag(s) > 1e-6
        fprintf('  %-12s %.3f +-%.3fi (wn %.2f, zeta %.2f)\n', modes(k).group, real(s), imag(s), abs(s), -real(s) / abs(s));
    else
        fprintf('  %-12s %.3f\n', modes(k).group, real(s));
    end
end
fprintf('open-loop poles (control laws cut at the Surfaces bus)\n');
names = {'open_pitch', 'open_lat'};
for k = 1:numel(names)
    s = lin.poles.(names{k});
    [~, order] = sort(abs(s));
    s = s(order);
    fprintf('  %-11s', names{k});
    for j = 1:numel(s)
        if imag(s(j)) > 1e-6
            fprintf('  %.3f+-%.3fi (wn %.2f, zeta %.2f)', real(s(j)), imag(s(j)), abs(s(j)), -real(s(j)) / abs(s(j)));
        elseif abs(imag(s(j))) <= 1e-6
            fprintf('  %.3f', real(s(j)));
        end
    end
    fprintf('\n');
end
end

function plot_poles(lin, title_text, file)
fig = figure('Visible', 'off', 'Position', [100 100 800 400]);
groups = {'longitudinal', 'lateral', 'mixed'};
markers = {'x', 'o', '+'};
hold on;
for k = 1:numel(groups)
    s = [lin.modes([lin.modes.group] == groups{k}).s];
    s = s(abs(s) < 40);
    plot(real(s), imag(s), markers{k}, 'MarkerSize', 9);
end
xline(0);
grid on;
xlabel('real, 1/s');
ylabel('imag, rad/s');
legend(groups, 'Location', 'best');
title(['closed-loop modes ' title_text(1:find(title_text == ' ', 1) - 1)]);
exportgraphics(fig, file, 'Resolution', 110);
close(fig);
end

function text = ternary(condition, yes, no)
if condition
    text = yes;
else
    text = no;
end
end

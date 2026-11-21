% Checks the reference model: hands-off level flight, then pitch, roll and pedal steps.
%
%   check_reference
%
% Plots go to reference/build/ as PNG. Returns a struct of the measured numbers; prints PASS/FAIL per check.
% Speeds are indicated airspeed (what the mod's recorder logs as ias_kmh), 3000 m, standard day, half fuel.
function summary = check_reference()
    reference_dir = fileparts(mfilename("fullpath"));
    build_dir = fullfile(reference_dir, "build");
    summary = struct();

    summary.level_clean = check_level("level_600_clean", 3000, 600, false, 60, build_dir);
    % Gear down the FCS holds the angle of attack, not the flight path, so a slow phugoid builds that the pilot
    % would trim out with the throttle: the altitude limit is wider
    summary.level_gear = check_level("level_320_gear_down", 3000, 320, true, 100, build_dir);
    summary.steps = check_steps(3000, 600, build_dir);

    all_passed = summary.level_clean.passed && summary.level_gear.passed && summary.steps.passed;
    fprintf("check_reference: %s\n", ternary(all_passed, "ALL PASSED", "SOME CHECKS FAILED"));
    save(fullfile(build_dir, "check_summary.mat"), "summary");
end

function report = check_level(name, altitude_m, ias_kmh, gear_down, altitude_limit_m, build_dir)
    [cfg, throttle] = find_level_trim(altitude_m = altitude_m, ias_kmh = ias_kmh, gear_down = gear_down, ...
        verbose = false);
    result = run_reference(cfg, pilot_timeseries.hold(throttle = throttle, gear_cmd = double(gear_down)), 20);

    report.name = name;
    report.throttle = throttle;
    report.altitude_change_m = result.altitude(end) - result.altitude(1);
    report.altitude_range_m = max(result.altitude) - min(result.altitude);
    report.max_bank_deg = max(abs(rad2deg(result.roll)));
    report.heading_drift_deg = rad2deg(result.heading(end) - result.heading(1));
    report.speed_change_kmh = (result.tas(end) - result.tas(1)) * 3.6;
    report.alpha_deg = rad2deg(mean(result.alpha(result.t > 18)));
    report.max_beta_deg = max(abs(rad2deg(result.beta)));
    report.stab_deg = mean(result.stab_deg(result.t > 18));
    report.max_pitch_rate_degs = max(abs(rad2deg(result.w(result.t > 2, 3))));

    % Tolerances: a trimmed jet with a g-command FCS should hold within these over 20 s hands off
    report.passed = report.altitude_range_m < altitude_limit_m && report.max_bank_deg < 1.5 ...
        && abs(report.heading_drift_deg) < 2 && abs(report.speed_change_kmh) < 20;
    fprintf("%s: throttle %.3f, altitude %+.1f m (range %.1f), max bank %.2f deg, heading %+.2f deg, speed %+.1f km/h, alpha %.2f deg, stab %.2f deg -> %s\n", ...
        name, throttle, report.altitude_change_m, report.altitude_range_m, report.max_bank_deg, ...
        report.heading_drift_deg, report.speed_change_kmh, report.alpha_deg, report.stab_deg, ...
        ternary(report.passed, "PASS", "FAIL"));

    figure_handle = figure(Visible = "off", Position = [100 100 900 900]);
    tiledlayout(4, 1, TileSpacing = "compact");
    nexttile; plot(result.t, result.altitude); ylabel("altitude, m"); title(name + ": hands off"); grid on;
    nexttile; plot(result.t, result.tas * 3.6); ylabel("TAS, km/h"); grid on;
    nexttile; plot(result.t, rad2deg(result.roll), result.t, rad2deg(result.pitch), result.t, rad2deg(result.alpha));
    ylabel("deg"); legend("bank", "pitch", "alpha"); grid on;
    nexttile; plot(result.t, result.stab_deg, result.t, result.flaperon_deg, result.t, result.rudder_deg);
    ylabel("surface, deg"); xlabel("time, s"); legend("stab", "flaperon", "rudder"); grid on;
    exportgraphics(figure_handle, fullfile(build_dir, name + ".png"), Resolution = 110);
    close(figure_handle);
end

function report = check_steps(altitude_m, ias_kmh, build_dir)
    [cfg, throttle] = find_level_trim(altitude_m = altitude_m, ias_kmh = ias_kmh, verbose = false);
    base = pilot_timeseries.hold(throttle = throttle);

    pitch_input = pilot_timeseries.step(pilot_timeseries.step(base, "stick_pitch", 2, 0.3), "stick_pitch", 6, 0);
    pitch = run_reference(cfg, pitch_input, 10);
    roll_input = pilot_timeseries.step(pilot_timeseries.step(base, "stick_roll", 2, 1), "stick_roll", 4, 0);
    roll = run_reference(cfg, roll_input, 8);
    pedal_input = pilot_timeseries.step(pilot_timeseries.step(base, "pedals", 2, 0.5), "pedals", 6, 0);
    pedal = run_reference(cfg, pedal_input, 10);

    report.pitch_peak_nz = max(pitch.nz);
    report.pitch_peak_rate_degs = max(rad2deg(pitch.w(:, 3)));
    report.pitch_peak_alpha_deg = max(rad2deg(pitch.alpha));
    report.roll_peak_rate_degs = max(rad2deg(roll.w(:, 1)));
    report.roll_bank_at_4s_deg = interp1(roll.t, rad2deg(unwrap(roll.roll)), 4);   % unwrapped: the full roll passes 180 deg
    report.roll_peak_beta_deg = max(abs(rad2deg(roll.beta)));
    report.pedal_peak_yaw_rate_degs = min(rad2deg(pedal.w(:, 2)));        % DCS wy: negative = nose right
    report.pedal_peak_beta_deg = min(rad2deg(pedal.beta));                % negative = nose right of the flow
    report.pedal_peak_bank_deg = max(abs(rad2deg(pedal.roll)));

    report.roll_rate_limit_degs = read_cfg_number(cfg.config_dir, "roll_rate_max_deg");

    signs_ok = report.pitch_peak_rate_degs > 1 && report.pitch_peak_nz > 1.2 ...
        && report.roll_peak_rate_degs > 20 && report.pedal_peak_yaw_rate_degs < -0.5 && report.pedal_peak_beta_deg < -0.3;
    magnitude_ok = report.pitch_peak_nz < 9.5 && report.roll_peak_rate_degs < 1.2 * report.roll_rate_limit_degs ...
        && report.pitch_peak_alpha_deg < 35;
    report.passed = signs_ok && magnitude_ok;

    fprintf("pitch step (+0.3 stick, 4 s): peak nz %.2f g, peak pitch rate %.1f deg/s, peak alpha %.1f deg\n", ...
        report.pitch_peak_nz, report.pitch_peak_rate_degs, report.pitch_peak_alpha_deg);
    fprintf("roll step (full right, 2 s): peak roll rate %.1f deg/s (cfg roll_rate_max_deg %.0f), bank at 4 s %.0f deg, peak beta %.2f deg\n", ...
        report.roll_peak_rate_degs, report.roll_rate_limit_degs, report.roll_bank_at_4s_deg, report.roll_peak_beta_deg);
    fprintf("pedal step (+0.5, 4 s): peak yaw rate %.2f deg/s (nose right = negative), peak beta %.2f deg, bank %.2f deg -> %s\n", ...
        report.pedal_peak_yaw_rate_degs, report.pedal_peak_beta_deg, report.pedal_peak_bank_deg, ...
        ternary(report.passed, "PASS", "FAIL"));

    figure_handle = figure(Visible = "off", Position = [100 100 1200 900]);
    tiledlayout(3, 3, TileSpacing = "compact");
    nexttile; plot(pitch.t, pitch.nz); ylabel("nz, g"); title("pitch step"); grid on;
    nexttile; plot(roll.t, rad2deg(roll.w(:, 1))); ylabel("roll rate, deg/s"); title("roll step"); grid on;
    nexttile; plot(pedal.t, rad2deg(pedal.w(:, 2))); ylabel("yaw rate (left +), deg/s"); title("pedal step"); grid on;
    nexttile; plot(pitch.t, rad2deg(pitch.w(:, 3)), pitch.t, rad2deg(pitch.alpha)); ylabel("deg/s, deg");
    legend("pitch rate", "alpha"); grid on;
    nexttile; plot(roll.t, rad2deg(roll.roll)); ylabel("bank, deg"); grid on;
    nexttile; plot(pedal.t, rad2deg(pedal.beta), pedal.t, rad2deg(pedal.roll)); ylabel("deg"); legend("beta", "bank"); grid on;
    nexttile; plot(pitch.t, pitch.stab_deg, pitch.t, pitch.pilot(:, 1) * 10); ylabel("stab deg, stick x10");
    xlabel("time, s"); grid on;
    nexttile; plot(roll.t, roll.flaperon_deg, roll.t, roll.stab_diff_deg); ylabel("deg"); legend("flaperon", "stab diff");
    xlabel("time, s"); grid on;
    nexttile; plot(pedal.t, pedal.rudder_deg); ylabel("rudder, deg"); xlabel("time, s"); grid on;
    exportgraphics(figure_handle, fullfile(build_dir, "steps_600.png"), Resolution = 110);
    close(figure_handle);
end

function value = read_cfg_number(config_dir, key)
    value = NaN;
    lines = splitlines(string(fileread(fullfile(config_dir, "SU35XFM.cfg"))));
    for line = lines.'
        token = regexp(line, "^\s*" + key + "\s*=\s*([-0-9.eE+]+)", "tokens", "once");
        if ~isempty(token)
            value = str2double(token{1});
        end
    end
end

function text = ternary(condition, yes, no)
    if condition
        text = yes;
    else
        text = no;
    end
end

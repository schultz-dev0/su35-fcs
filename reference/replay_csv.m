% Replays a recorded flight through the reference model and compares the rates.
%
%   result = replay_csv()                          newest suitable recording, most active airborne stretch
%   result = replay_csv(csv_file, start_s = 120, duration_s = 20)
%
% The recording is the mod's flight recorder CSV (DCS Saved Games\DCS\Logs\xfm-*.csv, 50 rows a second). The model
% starts from the recorded state (altitude, speed, angle of attack, sideslip, attitude, rates, fuel, mass) and is
% flown with the recorded stick, pedal and throttle values. It cannot reproduce what the cockpit did: a stretch with
% the autopilot or autothrottle on is not picked automatically, and one picked by hand gives a warning. Heading
% starts at 0 (the recording has none). The CSV is read in place and never copied or committed.
%
% Plot: reference/build/replay_<file name>.png. Returns the compared signals and RMS differences.
function result = replay_csv(csv_file, options)
    arguments
        csv_file (1,1) string = ""
        options.start_s (1,1) double = NaN        % recorded time (column t) to start at, default: automatic
        options.duration_s (1,1) double = 20
        options.match_engines (1,1) logical = true   % start the engines at the recorded rpm, afterburner and thrust
        options.match_atmosphere (1,1) logical = true   % hold the recorded temperature and pressure offsets from ISA
    end
    reference_dir = fileparts(mfilename("fullpath"));
    build_dir = fullfile(reference_dir, "build");

    if csv_file == ""
        csv_file = newest_recording();
    end
    fprintf("replay_csv: reading %s\n", csv_file);
    recorded = readtable(csv_file);

    start_s = options.start_s;
    if isnan(start_s)
        start_s = most_active_airborne_start(recorded, options.duration_s);
    end
    in_window = recorded.t >= start_s & recorded.t <= start_s + options.duration_s;
    window = recorded(in_window, :);
    assert(height(window) > 10, "replay_csv: no rows between %.1f s and %.1f s", start_s, start_s + options.duration_s);
    if any(window.ap_mode ~= 0) || any(window.autothrottle ~= 0)
        warning("replay_csv: the autopilot or autothrottle was on in this stretch; the model cannot follow it");
    end
    first = window(1, :);
    fprintf("replay_csv: from t = %.1f s, %.0f m, %.0f km/h IAS, %.1f s long\n", start_s, first.alt_m, ...
        first.ias_kmh, options.duration_s);

    gear_down = first.gear_pos > 0.5;
    % The recording has no temperature or pressure column, but speed of sound = TAS / Mach gives the temperature and
    % qbar = rho V^2 / 2 the density, so pressure = rho R T. Their offsets from ISA at the recorded altitude are held
    % for the whole replay (the logged days were +12 K and +18 K, with pressure 0 to 4 % above ISA).
    recorded_temperature = (first.tas_ms / first.mach)^2 / (1.4 * 287.05);
    recorded_density = 2 * first.qbar_pa / first.tas_ms^2;
    [isa_temperature, ~, isa_pressure] = atmosisa(first.alt_m);
    temperature_offset = recorded_temperature - isa_temperature;
    pressure_ratio = recorded_density * 287.05 * recorded_temperature / isa_pressure;
    fprintf("replay_csv: air %.1f K (ISA %+.1f K), pressure %.3f x ISA\n", recorded_temperature, temperature_offset, pressure_ratio);
    cfg = xfm_default_config(altitude_m = first.alt_m, tas_ms = first.tas_ms, gear_down = gear_down, ...
        fuel_kg = first.fuel_int_kg, temperature_offset_k = temperature_offset * options.match_atmosphere, ...
        pressure_ratio = 1 + (pressure_ratio - 1) * options.match_atmosphere);
    cfg = start_from_recorded_state(cfg, first, options.match_engines);

    local_time = window.t - window.t(1);
    values = [window.stick_pitch, window.stick_roll, window.pedals, window.throttle_l, window.throttle_r, ...
        window.gear_cmd, double(window.airbrake_pos > 0.5), window.tvc_on, window.limiter_off, zeros(height(window), 1)];
    pilot = pilot_timeseries.make(local_time, values);
    simulated = run_reference(cfg, pilot, local_time(end));

    result = compare_and_plot(simulated, window, local_time, csv_file, start_s, build_dir);
end

function cfg = start_from_recorded_state(cfg, first, match_engines)
    % The recording has the exact attitude and body-axis flow angles, so use them instead of the level start.
    alpha = deg2rad(first.aoa_deg);
    beta = deg2rad(first.beta_deg);
    speed = first.tas_ms;
    cfg.ic.vb0_aero = speed * [cos(alpha) * cos(beta), sin(beta), sin(alpha) * cos(beta)];
    cfg.ic.eul0 = [deg2rad(first.roll_deg), deg2rad(first.pitch_deg), 0];
    % recorded yaw rate is + nose left (DCS wy); the aerospace yaw rate r is the opposite sign
    cfg.ic.pqr0 = deg2rad([first.roll_rate_degs, first.pitch_rate_degs, -first.yaw_rate_left_degs]);
    % mass: the recording's mass is the airplane with its stores and fuel. DCS hands the flight model "empty plus
    % stores" and the flight model adds the fuel itself, so the empty mass given here is the recorded mass minus fuel.
    cfg.settings(2) = first.mass_kg - first.fuel_int_kg;
    cfg.mass0 = first.mass_kg;

    % The start routine leaves the engines at 90 % rpm with no afterburner. A jet recorded in full afterburner would
    % lose its first seconds of thrust (the afterburner takes several seconds to light), so start them as recorded.
    if match_engines
        cfg.engines = [1, first.rpm_l, first.rpm_r, first.ab_l, first.ab_r, first.thrust_l_N, first.thrust_r_N];
    end
end

function file = newest_recording()
    log_dir = fullfile(fileparts(installed_mod_folder("Bin")), "..", "..", "..", "Logs");
    listing = dir(fullfile(log_dir, "xfm-*.csv"));
    assert(~isempty(listing), "replay_csv: no xfm-*.csv in %s", log_dir);
    [~, order] = sort([listing.datenum], "descend");
    for k = order
        if listing(k).bytes > 1.0e6
            file = string(fullfile(listing(k).folder, listing(k).name));
            return
        end
    end
    file = string(fullfile(listing(order(1)).folder, listing(order(1)).name));
end

% Window of the recording, airborne with no autopilot, where the jet was manoeuvred most (rate variation)
function start_s = most_active_airborne_start(recorded, duration_s)
    clean = recorded.alt_m > 300 & recorded.tas_ms > 100 & recorded.ap_mode == 0 & recorded.autothrottle == 0 ...
        & recorded.gear_pos < 0.05 & abs(recorded.roll_deg) < 170;
    rows_per_window = round(duration_s * 50);
    best_score = -1;
    start_s = recorded.t(1);
    for first_row = 1:50:height(recorded) - rows_per_window
        rows = first_row:first_row + rows_per_window;
        if ~all(clean(rows))
            continue
        end
        score = std(recorded.roll_rate_degs(rows)) + std(recorded.pitch_rate_degs(rows)) ...
            + std(recorded.yaw_rate_left_degs(rows));
        if score > best_score
            best_score = score;
            start_s = recorded.t(first_row);
        end
    end
    assert(best_score >= 0, "replay_csv: no airborne stretch without autopilot found; pass start_s");
end

function result = compare_and_plot(simulated, window, local_time, csv_file, start_s, build_dir)
    sim_time = simulated.t;
    recorded_at = @(column) interp1(local_time, window.(column), sim_time, "linear", "extrap");

    result.t = sim_time;
    result.recorded.roll_rate_degs = recorded_at("roll_rate_degs");
    result.recorded.pitch_rate_degs = recorded_at("pitch_rate_degs");
    result.recorded.yaw_rate_left_degs = recorded_at("yaw_rate_left_degs");
    result.recorded.aoa_deg = recorded_at("aoa_deg");
    result.recorded.nz_g = recorded_at("nz_g");
    result.recorded.alt_m = recorded_at("alt_m");
    result.recorded.ias_kmh = recorded_at("ias_kmh");
    result.recorded.tas_kmh = recorded_at("tas_ms") * 3.6;
    result.recorded.thrust_left_N = recorded_at("thrust_l_N");
    result.recorded.thrust_right_N = recorded_at("thrust_r_N");
    result.recorded.mass_kg = recorded_at("mass_kg");
    result.simulated = simulated;

    sim_rates_degs = rad2deg(simulated.w);
    result.rms.roll_rate_degs = rms(sim_rates_degs(:, 1) - result.recorded.roll_rate_degs);
    result.rms.pitch_rate_degs = rms(sim_rates_degs(:, 3) - result.recorded.pitch_rate_degs);
    result.rms.yaw_rate_left_degs = rms(sim_rates_degs(:, 2) - result.recorded.yaw_rate_left_degs);
    result.rms.aoa_deg = rms(rad2deg(simulated.alpha) - result.recorded.aoa_deg);
    result.rms.nz_g = rms(simulated.nz - result.recorded.nz_g);
    for at_s = [10 15]
        if at_s <= sim_time(end)
            k = find(sim_time >= at_s, 1);
            result.error_at(at_s).tas_kmh = simulated.tas(k) * 3.6 - result.recorded.tas_kmh(k);
            result.error_at(at_s).altitude_m = simulated.altitude(k) - result.recorded.alt_m(k);
            fprintf("replay_csv: at %d s, sim minus recording: TAS %+.1f km/h, altitude %+.0f m, thrust L %+.0f N, mass %+.0f kg\n", ...
                at_s, result.error_at(at_s).tas_kmh, result.error_at(at_s).altitude_m, ...
                simulated.thrust_left(k) - result.recorded.thrust_left_N(k), simulated.mass(k) - result.recorded.mass_kg(k));
        end
    end
    fprintf("replay_csv: RMS difference, sim minus recording: roll rate %.1f deg/s, pitch rate %.1f deg/s, yaw rate %.1f deg/s, alpha %.2f deg, nz %.2f g\n", ...
        result.rms.roll_rate_degs, result.rms.pitch_rate_degs, result.rms.yaw_rate_left_degs, ...
        result.rms.aoa_deg, result.rms.nz_g);

    figure_handle = figure(Visible = "off", Position = [100 100 1100 1250]);
    tiledlayout(5, 2, TileSpacing = "compact");
    plot_pair(sim_time, sim_rates_degs(:, 1), result.recorded.roll_rate_degs, "roll rate, deg/s");
    title("replay of " + start_s + " s from " + extractAfter(csv_file, strlength(fileparts(csv_file)) + 1), Interpreter = "none");
    plot_pair(sim_time, sim_rates_degs(:, 3), result.recorded.pitch_rate_degs, "pitch rate, deg/s");
    plot_pair(sim_time, sim_rates_degs(:, 2), result.recorded.yaw_rate_left_degs, "yaw rate (left +), deg/s");
    plot_pair(sim_time, rad2deg(simulated.alpha), result.recorded.aoa_deg, "alpha, deg");
    plot_pair(sim_time, simulated.nz, result.recorded.nz_g, "nz, g");
    plot_pair(sim_time, simulated.tas * 3.6, result.recorded.tas_kmh, "TAS, km/h");
    plot_pair(sim_time, simulated.altitude, result.recorded.alt_m, "altitude, m");
    nexttile;
    plot(sim_time, result.recorded.thrust_left_N / 1e3, "b", sim_time, simulated.thrust_left / 1e3, "r", ...
        sim_time, result.recorded.thrust_right_N / 1e3, "b--", sim_time, simulated.thrust_right / 1e3, "r--");
    ylabel("thrust, kN"); legend("recorded L", "simulated L", "recorded R", "simulated R", Location = "best"); grid on;
    plot_pair(sim_time, simulated.mass, result.recorded.mass_kg, "mass, kg");
    nexttile;
    plot(local_time, window.stick_pitch, local_time, window.stick_roll, local_time, window.pedals, ...
        local_time, window.throttle_l);
    ylabel("recorded inputs"); xlabel("time, s"); legend("pitch", "roll", "pedals", "throttle L"); grid on;
    [~, stem] = fileparts(csv_file);
    exportgraphics(figure_handle, fullfile(build_dir, "replay_" + stem + ".png"), Resolution = 110);
    close(figure_handle);
end

function plot_pair(time, simulated, recorded, label)
    nexttile;
    plot(time, recorded, time, simulated);
    ylabel(label);
    legend("recorded", "simulated", Location = "best");
    grid on;
end

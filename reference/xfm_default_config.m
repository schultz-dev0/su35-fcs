% Settings for one reference run: where the flight model lives, the airplane's mass data, and the air start.
%
%   cfg = xfm_default_config()
%   cfg = xfm_default_config(altitude_m = 3000, ias_kmh = 600, gear_down = false)
%
% Mass, inertia and centre of gravity are read from the installed mod's files at run time (FlankerCFG.lua, and
% SU35XFM.cfg or the flight model's built-in default for the empty mass and fuel capacity). They end up in this
% struct in the workspace, never in the saved model, so no mod parameters are committed.
function cfg = xfm_default_config(options)
    arguments
        options.altitude_m (1,1) double = 3000
        options.ias_kmh (1,1) double = 600           % indicated airspeed at the air start
        options.tas_ms (1,1) double = NaN            % true airspeed, overrides ias_kmh when given
        options.heading_deg (1,1) double = 0
        options.alpha_deg (1,1) double = NaN         % NaN: a typical level-flight value (see below)
        options.flight_path_deg (1,1) double = 0
        options.bank_deg (1,1) double = 0
        options.rates_degs (1,3) double = [0 0 0]    % roll, pitch, yaw rate, aerospace signs (p q r)
        options.gear_down (1,1) logical = false
        options.fuel_kg (1,1) double = NaN           % NaN: half the internal tanks
        options.temperature_offset_k (1,1) double = 0   % hot day (+) or cold day (-) against ISA
        options.pressure_ratio (1,1) double = 1         % pressure against ISA at the altitude
        options.dt (1,1) double = 0.006              % DCS flight model step
        options.config_dir (1,1) string = ""         % folder with SU35XFM.cfg, default: the installed mod's Bin
        options.run_dir (1,1) string = ""            % where the flight model writes its log, default reference/build
    end

    reference_dir = fileparts(mfilename("fullpath"));
    build_dir = fullfile(reference_dir, "build");
    settings_file = fullfile(build_dir, "build_settings.mat");
    assert(isfile(settings_file), "xfm_default_config: run build_reference first");
    built = load(settings_file);

    config_dir = options.config_dir;
    if config_dir == ""
        config_dir = built.config_dir;
    end
    run_dir = options.run_dir;
    if run_dir == ""
        run_dir = string(build_dir);
    end

    cfg.library = char(fullfile(build_dir, "libxfm_ref.so"));
    cfg.config_dir = char(config_dir);
    cfg.run_dir = char(run_dir);
    cfg.dt = options.dt;
    cfg.temperature_offset_k = options.temperature_offset_k;
    cfg.pressure_ratio = options.pressure_ratio;

    mass_data = read_mass_data(installed_mod_folder("FM"));
    empty_mass = read_cfg_value(config_dir, "empty_mass_kg", built.empty_mass_kg_default);
    fuel_capacity = read_cfg_value(config_dir, "fuel_internal_max", built.fuel_internal_max_default);
    fuel = options.fuel_kg;
    if isnan(fuel)
        fuel = 0.5 * fuel_capacity;
    end
    cfg.fuel_kg = fuel;
    cfg.empty_mass_kg = empty_mass;
    cfg.mass0 = empty_mass + fuel;
    cfg.cg = mass_data.cg;
    cfg.inertia_dcs = mass_data.inertia;   % Ixx (roll), Iyy (yaw), Izz (pitch)
    cfg.product_dcs = mass_data.product;   % the config's fourth number, not used unless use_product is set
    cfg.use_product = false;
    cfg.settings = [fuel, empty_mass, mass_data.inertia, mass_data.cg, double(options.gear_down)];

    % Inertia matrix for the 6-DOF block, aerospace axes (x forward, y right, z down): pitch is DCS z, yaw is DCS y
    cfg.inertia_aero = diag([mass_data.inertia(1), mass_data.inertia(3), mass_data.inertia(2)]);

    cfg.engines = zeros(1, 7);   % [use, rpm_l, rpm_r, ab_l, ab_r, thrust_l, thrust_r]; use = 0: start routine's state
    cfg.gear_down = options.gear_down;
    cfg.ic = air_start_state(options);
end

function ic = air_start_state(options)
    g0 = 9.80665;
    altitude = options.altitude_m;
    [temperature, speed_of_sound, ~, density] = atmosisa(altitude);
    % the offset day: warmer air is thinner and its speed of sound higher (pressure kept at ISA)
    warm = temperature + options.temperature_offset_k;
    speed_of_sound = speed_of_sound * sqrt(warm / temperature);
    density = density * options.pressure_ratio * temperature / warm;
    temperature = warm;
    density_sea_level = 1.225;
    if isnan(options.tas_ms)
        true_airspeed = options.ias_kmh / 3.6 * sqrt(density_sea_level / density);
    else
        true_airspeed = options.tas_ms;
    end
    alpha = deg2rad(options.alpha_deg);
    if isnan(alpha)
        alpha = deg2rad(4);   % starting guess only; find_level_trim refines it
    end
    flight_path = deg2rad(options.flight_path_deg);
    pitch = flight_path + alpha;

    ic.altitude_m = altitude;
    ic.tas_ms = true_airspeed;
    ic.mach = true_airspeed / speed_of_sound;
    ic.temperature_k = temperature;
    ic.alpha = alpha;
    ic.theta = pitch;
    ic.phi = deg2rad(options.bank_deg);
    ic.psi = deg2rad(options.heading_deg);
    ic.eul0 = [ic.phi, ic.theta, ic.psi];
    ic.vb0_aero = true_airspeed * [cos(alpha), 0, sin(alpha)];   % u forward, v right, w down
    ic.xe0 = [0, 0, -altitude];                                  % north, east, down
    ic.pqr0 = deg2rad(options.rates_degs);
    ic.g = g0;
end

function mass_data = read_mass_data(fm_folder)
    text = fileread(fullfile(fm_folder, "FlankerCFG.lua"));
    lines = splitlines(string(text));
    lines = lines(~startsWith(strtrim(lines), "--"));
    live = strjoin(lines, newline);
    mass_data.cg = first_number_list(live, "center_of_mass", 3);
    inertia_numbers = first_number_list(live, "moment_of_inertia", 4);
    mass_data.inertia = inertia_numbers(1:3);
    mass_data.product = inertia_numbers(4);
end

function numbers = first_number_list(text, name, count)
    pattern = name + "\s*=\s*\{([^}]*)\}";
    token = regexp(text, pattern, "tokens", "once");
    assert(~isempty(token), "xfm_default_config: %s not found in FlankerCFG.lua", name);
    body = regexprep(token{1}, "--[^\n]*", "");
    numbers = sscanf(strrep(body, ",", " "), "%f").';
    assert(numel(numbers) >= count, "xfm_default_config: %s needs %d numbers", name, count);
    numbers = numbers(1:count);
end

function value = read_cfg_value(config_dir, key, fallback)
    value = fallback;
    cfg_file = fullfile(config_dir, "SU35XFM.cfg");
    if ~isfile(cfg_file)
        return
    end
    lines = splitlines(string(fileread(cfg_file)));
    for line = lines.'
        token = regexp(line, "^\s*" + key + "\s*=\s*([-0-9.eE+]+)", "tokens", "once");
        if ~isempty(token)
            value = str2double(token{1});
        end
    end
end

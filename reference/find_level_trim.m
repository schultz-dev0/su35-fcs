% Finds the throttle and starting angle of attack that hold level flight in the reference model.
%
%   [cfg, throttle] = find_level_trim(altitude_m = 3000, ias_kmh = 600, gear_down = false)
%
% The mod's FCS trims the pitch axis itself (it integrates the pitch rate error), but it holds the flight path (gear
% up) or the angle of attack (gear down) it starts with. So level flight at a given speed needs the right starting
% angle of attack and the right throttle together. Newton's method on those two numbers, with the Jacobian from
% finite differences, drives both the climb rate and the acceleration (fitted over seconds 3 to 10 of a 10 s run) to
% zero. The result is a starting point, not a perfect equilibrium; the check looks at how well it holds.
function [cfg, throttle] = find_level_trim(options)
    arguments
        options.altitude_m (1,1) double = 3000
        options.ias_kmh (1,1) double = 600
        options.gear_down (1,1) logical = false
        options.fuel_kg (1,1) double = NaN
        options.iterations (1,1) double = 6
        options.verbose (1,1) logical = true
    end

    make_cfg = @(alpha_deg) xfm_default_config(altitude_m = options.altitude_m, ias_kmh = options.ias_kmh, ...
        gear_down = options.gear_down, alpha_deg = alpha_deg, fuel_kg = options.fuel_kg);
    residual = @(unknowns) measure_rates(make_cfg(unknowns(1)), ...
        pilot_timeseries.hold(throttle = unknowns(2), gear_cmd = double(options.gear_down)));

    unknowns = [4; 0.4];                   % alpha in deg, throttle lever 0..1
    steps = [0.5; 0.03];                   % finite difference steps
    limits = [0 20; 0.02 0.85];
    for iteration = 1:options.iterations
        base = residual(unknowns);
        if options.verbose
            fprintf("trim %d: alpha %.2f deg, throttle %.3f -> climb %+.3f m/s, acceleration %+.4f m/s2\n", ...
                iteration, unknowns(1), unknowns(2), base(1), base(2));
        end
        if abs(base(1)) < 0.05 && abs(base(2)) < 0.01
            break
        end
        jacobian = zeros(2);
        for k = 1:2
            probe = unknowns;
            probe(k) = probe(k) + steps(k);
            jacobian(:, k) = (residual(probe) - base) / steps(k);
        end
        delta = -jacobian \ base;
        unknowns = min(max(unknowns + delta, limits(:, 1)), limits(:, 2));
    end
    throttle = unknowns(2);
    cfg = make_cfg(unknowns(1));
end

% [climb rate m/s; acceleration along the path m/s2], straight-line fits over seconds 3 to 10
function rates = measure_rates(cfg, pilot)
    result = run_reference(cfg, pilot, 10);
    window = result.t >= 3;
    altitude_fit = polyfit(result.t(window), result.altitude(window), 1);
    speed_fit = polyfit(result.t(window), result.tas(window), 1);
    rates = [altitude_fit(1); speed_fit(1)];
end

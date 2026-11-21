% Pilot inputs as the timeseries the reference model reads (bus Pilot, docs/interfaces.md).
%
%   pilot = pilot_timeseries(times, values)   values: one row per time, 10 columns in this order:
%       stick_pitch, stick_roll, pedals, throttle_left, throttle_right, gear_cmd, airbrake_cmd, tvc_on,
%       limiter_off, refuel_probe
%   pilot = pilot_timeseries.hold(throttle = 0.5)   constant inputs, name = value pairs
%
% The model holds the last row after the last time.
classdef pilot_timeseries
    properties (Constant)
        names = ["stick_pitch", "stick_roll", "pedals", "throttle_left", "throttle_right", "gear_cmd", ...
            "airbrake_cmd", "tvc_on", "limiter_off", "refuel_probe"]
    end
    methods (Static)
        function ts = make(times, values)
            assert(size(values, 2) == numel(pilot_timeseries.names), "pilot_timeseries: need 10 columns");
            ts = timeseries(values, times(:), Name = "pilot");
        end

        % Constant inputs. throttle sets both levers; throttle_left / throttle_right override it.
        function ts = hold(options)
            arguments
                options.stick_pitch (1,1) double = 0
                options.stick_roll (1,1) double = 0
                options.pedals (1,1) double = 0
                options.throttle (1,1) double = 0.5
                options.throttle_left (1,1) double = NaN
                options.throttle_right (1,1) double = NaN
                options.gear_cmd (1,1) double = 0
                options.airbrake_cmd (1,1) double = 0
                options.tvc_on (1,1) double = 0
                options.limiter_off (1,1) double = 0
                options.refuel_probe (1,1) double = 0
            end
            left = options.throttle_left;
            right = options.throttle_right;
            if isnan(left), left = options.throttle; end
            if isnan(right), right = options.throttle; end
            row = [options.stick_pitch, options.stick_roll, options.pedals, left, right, options.gear_cmd, ...
                options.airbrake_cmd, options.tvc_on, options.limiter_off, options.refuel_probe];
            ts = pilot_timeseries.make([0; 1000], [row; row]);
        end

        % Starting from an existing timeseries, replace one channel from time t_change on (step input).
        function ts = step(ts, channel, t_change, new_value)
            index = find(pilot_timeseries.names == channel, 1);
            assert(~isempty(index), "pilot_timeseries.step: unknown channel %s", channel);
            times = ts.Time;
            values = ts.Data;
            before = values(end, :);
            after = before;
            after(index) = new_value;
            keep = times < t_change;
            times = [times(keep); t_change - 1e-6; t_change; times(end)];
            values = [values(keep, :); before; after; after];
            ts = pilot_timeseries.make(times, values);
        end
    end
end

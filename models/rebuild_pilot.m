function ts = rebuild_pilot(time, inputs, throttle)
% PILOT_TIMESERIES  The timeseries the model's Pilot block reads, and puts it in the base workspace as su35_pilot.
%   time       column of sample times, s
%   inputs     struct; any of stick_pitch, stick_roll, pedals, gear_cmd, airbrake_cmd, tvc_on, limiter_off,
%              refuel_probe, each a scalar or a column the length of time. Missing ones are 0.
%   throttle   lever position for both engines (0..1)
% Column order is the Pilot bus's: stick_pitch, stick_roll, pedals, throttle_left, throttle_right, gear_cmd,
% airbrake_cmd, tvc_on, limiter_off, refuel_probe.
time = time(:);
names = {'stick_pitch', 'stick_roll', 'pedals', 'throttle_left', 'throttle_right', 'gear_cmd', 'airbrake_cmd', ...
    'tvc_on', 'limiter_off', 'refuel_probe'};
data = zeros(numel(time), numel(names));
for k = 1:numel(names)
    name = names{k};
    if strcmp(name, 'throttle_left') || strcmp(name, 'throttle_right')
        data(:, k) = throttle;
    elseif isfield(inputs, name)
        data(:, k) = inputs.(name)(:);
    end
end
ts = timeseries(data, time);
assignin('base', 'su35_pilot', ts);
end

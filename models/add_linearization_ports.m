function add_linearization_ports(model)
% ADD_LINEARIZATION_PORTS  Gives the loaded model root ports so Simulink Control Design can perturb the whole model.
%   Three inports add to the stick pitch, stick roll and pedals (the From Workspace pilot stays as it is), and one outport
%   carries V, alpha, beta, p, q, r_yaw, phi, theta, nz. The change is in memory only: do not save the model after it.
% Why: the default block-by-block linearisation of this mixed continuous/discrete loop gives a spurious fast unstable
% lateral pair (see docs/rebuild.md). The numerical-perturbation algorithm perturbs the model itself, like a simulation
% would, but it only accepts root-level ports, not linio points.
load_system(model);
names = {'stick_pitch', 'stick_roll', 'pedals'};
for k = 1:3
    delete_line(model, sprintf('Pilot_split/%d', k), sprintf('Pilot/%d', k));
    inport = [model '/u_' names{k}];
    adder = [model '/Pilot_add_' names{k}];
    add_block('simulink/Sources/In1', inport, 'Port', num2str(k));
    add_block('simulink/Math Operations/Sum', adder, 'Inputs', '++');
    add_line(model, sprintf('Pilot_split/%d', k), ['Pilot_add_' names{k} '/1']);
    add_line(model, ['u_' names{k} '/1'], ['Pilot_add_' names{k} '/2']);
    line = add_line(model, ['Pilot_add_' names{k} '/1'], sprintf('Pilot/%d', k));
    set_param(line, 'Name', names{k});
end
outputs = {'V', 'alpha', 'beta', 'p', 'q', 'r_yaw', 'phi', 'theta', 'nz'};
add_block('simulink/Signal Routing/Bus Selector', [model '/Lin_select'], 'OutputSignals', strjoin(outputs, ','));
add_block('simulink/Signal Routing/Mux', [model '/Lin_mux'], 'Inputs', num2str(numel(outputs)));
add_block('simulink/Sinks/Out1', [model '/y_state'], 'Port', '1');
add_line(model, 'State_builder/1', 'Lin_select/1');
for k = 1:numel(outputs)
    add_line(model, sprintf('Lin_select/%d', k), sprintf('Lin_mux/%d', k));
end
add_line(model, 'Lin_mux/1', 'y_state/1');
end

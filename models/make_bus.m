function make_bus(name, template)
% MAKE_BUS  Define a Simulink.Bus object `name` in the base workspace with one element per field of the struct
% `template` (double scalars or arrays), and a workspace variable `name`_init holding the template as initial value.
elements = Simulink.BusElement.empty;
field_names = fieldnames(template);
for k = 1:numel(field_names)
    element = Simulink.BusElement;
    element.Name = field_names{k};
    element.DataType = 'double';
    element.Dimensions = size(template.(field_names{k}));
    if isequal(element.Dimensions, [1 1])
        element.Dimensions = 1;
    end
    elements(end + 1) = element; %#ok<AGROW>
end
bus = Simulink.Bus;
bus.Elements = elements;
assignin('base', name, bus);
assignin('base', [name '_init'], template);
end

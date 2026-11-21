function ge = plant_ground_effect(P, height_m)
% PLANT_GROUND_EFFECT  Induced-drag factor and lift increment within a span of the ground (cpp:1790-1792).
%   height_m   height of the centre of gravity over the ground
A = P.aero;
h_over_b = min(max(height_m / P.geom.b, A.ge_h_min), A.ge_h_max);
x = A.ge_x * h_over_b;
ge.phi = 1 - A.ground_effect * (1 - x^2 / (1 + x^2));
ge.dcl = A.ground_effect * A.ge_dcl * min(max(1 - h_over_b, 0), 1);
end

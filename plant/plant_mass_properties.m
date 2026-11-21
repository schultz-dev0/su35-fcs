function [mass, Ixx, Iyy, Izz] = plant_mass_properties(P)
% PLANT_MASS_PROPERTIES  Mass and inertia, DCS axes (x roll, y yaw, z pitch). Constant: no fuel burn is modelled.
mass = P.mass.total_kg;
Ixx = P.mass.Ixx;
Iyy = P.mass.Iyy;
Izz = P.mass.Izz;
end

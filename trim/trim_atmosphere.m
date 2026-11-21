function atmo = trim_atmosphere(altitude)
% TRIM_ATMOSPHERE  International standard atmosphere below 11 km, the same as the Aerospace Blockset ISA block.
% atmo = [T a p rho] as the State builder takes it.
T0 = 288.15;
p0 = 101325;
lapse = 0.0065;
R = 287.0531;
gamma_air = 1.4;
T = T0 - lapse * altitude;
p = p0 * (T / T0)^(9.80665 / (lapse * R));
rho = p / (R * T);
a = sqrt(gamma_air * R * T);
atmo = [T; a; p; rho];
end

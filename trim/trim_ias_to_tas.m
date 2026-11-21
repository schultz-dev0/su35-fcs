function tas = trim_ias_to_tas(ias_kmh, altitude, P)
% TRIM_IAS_TO_TAS  True airspeed in m/s for an indicated airspeed in km/h, as the reference harness does it:
% TAS = IAS sqrt(rho0 / rho), no compressibility correction (the mod's IAS is this equivalent airspeed).
atmo = trim_atmosphere(altitude);
tas = ias_kmh / 3.6 * sqrt(P.const.rho0 / atmo(4));
end

function limit = fcs_nz_limit_tvc_off(P, mach, mass_kg)
% FCS_NZ_LIMIT_TVC_OFF  The g limit with TVC off: nz_max up to the full-load mass, then the same load on the airframe
% (nz x mass held), less a dip near M 1 against the transonic pitch-up (cpp:635-643).
F = P.pitch;
limit = F.nz_max;
if F.nz_full_mass_kg > 0 && mass_kg > F.nz_full_mass_kg
    limit = F.nz_max * F.nz_full_mass_kg / mass_kg;
end
dip_in = min(max((mach - F.nz_dip_in_mach) / F.nz_dip_width, 0), 1);
dip_out = min(max((F.nz_dip_out_mach - mach) / F.nz_dip_width, 0), 1);
limit = limit - F.nz_transonic_dip * min(dip_in, dip_out);
end

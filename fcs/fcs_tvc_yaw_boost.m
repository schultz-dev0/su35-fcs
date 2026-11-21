function boost = fcs_tvc_yaw_boost(P, mach, pedal, alpha, bank, pedal_sign)
% FCS_TVC_YAW_BOOST  Extra yaw from the nozzles as a share of the plain thrust-times-arm moment: large when slow and at
% high AoA, fading with bank when the pedal points away from the low wing, soft around centre pedal (cpp:651-663).
L = P.alloc;
m = max(mach, 0.02);
c = L.yb_c5 * m^5 + L.yb_c0 * m^L.yb_cexp;
if m <= L.yb_m1
    yc = L.yb_a1 / c + L.yb_b1;
elseif m <= L.yb_m2
    yc = L.yb_a2 / c + L.yb_b2;
else
    yc = max((L.yb_k * m + L.yb_off)^(-2) - L.yb_sub, 0);
end
if bank * pedal_sign > 0
    bank_deg = abs(bank) * 180 / pi;
    lo = min(max(1 - L.yb_lo_slope * m / L.yb_lo_mach, L.yb_lo_min), 1);
    if bank_deg < L.yb_bank_full
        yc = yc * min(max(1 + (lo - 1) * (bank_deg - L.yb_bank_ref) / L.yb_bank_span, lo), 1);
    else
        yc = yc * min(max(lo + (1 - lo) * (bank_deg - L.yb_bank_back) / L.yb_bank_back, lo), 1);
    end
end
% by the size of the AoA: inverted the signed AoA would cut the boost to its floor
yc = yc * min(max(L.yb_aoa_floor + (1 - L.yb_aoa_floor) * (abs(alpha) * 180 / pi) / L.yb_aoa_ref, L.yb_aoa_floor), 1);
boost = L.yb_gain * yc * abs(pedal)^L.yb_pow;
end

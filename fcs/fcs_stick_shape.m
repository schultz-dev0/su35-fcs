function y = fcs_stick_shape(P, x)
% FCS_STICK_SHAPE  Pitch and roll stick curve: a linear share plus a cubic, so small inputs are gentle and full stick
% still gives full authority (cpp:1110).
lin = min(max(P.pitch.stick_shape_lin, 0), 1);
y = lin * x + (1 - lin) * x^3;
end

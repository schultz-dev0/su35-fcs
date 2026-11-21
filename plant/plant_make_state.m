function st = plant_make_state(P, Vb, eul, pqr, xe, atmo, nz_ny, slow, thrust)
% PLANT_MAKE_STATE  The State bus from the 6-DOF block's outputs (aerospace axes) and the other plant blocks.
% This is the one place where aerospace axes become DCS axes (see docs/rebuild.md):
%   body velocity Vb = [u v w] (x forward, y right, z down), rates pqr = [p q r] about the same axes.
%   alpha is positive nose up and beta positive with the airflow coming from the right, in both frames.
%   r_yaw is DCS's yaw rate, positive nose LEFT, so r_yaw = -r.
%   eul = [phi theta psi], the same angles and signs as DCS (bank positive right wing down).
%   atmo = [T a p rho] from the ISA block. nz_ny = [nz ny_lat] from last frame (DCS delivers them a frame late).
%   slow = [gear flap flap_brake airbrake on_s inhibited]; thrust = [left right], N.
V = sqrt(Vb(1)^2 + Vb(2)^2 + Vb(3)^2);
st.V = V;
st.alpha = atan2(Vb(3), Vb(1));
st.beta = asin(min(max(Vb(2) / max(V, 1e-3), -1), 1));
st.p = pqr(1);
st.q = pqr(2);
st.r_yaw = -pqr(3);
st.phi = eul(1);
st.theta = eul(2);
st.psi = eul(3);
st.nz = nz_ny(1);
st.ny_lat = nz_ny(2);
st.mach = V / atmo(2);
st.rho = atmo(4);
st.qbar = 0.5 * atmo(4) * V^2;
st.altitude = -xe(3);
[mass, Ixx, Iyy, Izz] = plant_mass_properties(P);
st.mass = mass;
st.Ixx = Ixx;
st.Iyy = Iyy;
st.Izz = Izz;
st.gear_pos = slow(1);
st.flap_pos = slow(2);
st.flap_brake_pos = slow(3);
st.airbrake_pos = slow(4);
st.thrust_left = thrust(1);
st.thrust_right = thrust(2);
end

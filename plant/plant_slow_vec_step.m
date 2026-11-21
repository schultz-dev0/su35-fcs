function [slow_now, v_new] = plant_slow_vec_step(P, v, pilot, st_prev, dt)
% PLANT_SLOW_VEC_STEP  Gear, flaps and speed brake as a block with a state vector (wraps plant_slow_step).
%   v = [gear_pos flap_pos flap_brake_pos airbrake_pos airbrake_on_s brake_inhibited]
%   st_prev = [mach alpha nz qbar] of last frame. The positions out are the ones before this step's move, so the
%   block has no direct path from its inputs and the State -> slow -> State loop needs no extra delay.
slow.gear_pos = v(1);
slow.flap_pos = v(2);
slow.flap_brake_pos = v(3);
slow.airbrake_pos = v(4);
slow.airbrake_on_s = v(5);
slow.brake_inhibited = v(6);
cmd.gear = pilot.gear_cmd;
cmd.airbrake = pilot.airbrake_cmd;
st.mach = st_prev(1);
st.alpha = st_prev(2);
st.nz = st_prev(3);
st.qbar = st_prev(4);
slow_now = v;
slow = plant_slow_step(P, slow, cmd, st, dt);
v_new = [slow.gear_pos; slow.flap_pos; slow.flap_brake_pos; slow.airbrake_pos; slow.airbrake_on_s; slow.brake_inhibited];
end

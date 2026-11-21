function [thrust, ab_level, v_new] = plant_engines_step(P, v, pilot, Vb, atmo, dt)
% PLANT_ENGINES_STEP  Both engines, one frame. v = [rpm_left rpm_right ab_left ab_right]. Atmo = [T a p rho].
V = sqrt(Vb(1)^2 + Vb(2)^2 + Vb(3)^2);
air.T = atmo(1);
air.a = atmo(2);
air.p = atmo(3);
air.rho = atmo(4);
[rpm_left, ab_left, thrust_left] = plant_engine_step(P, pilot.throttle_left, v(1), v(3), air, V, dt);
[rpm_right, ab_right, thrust_right] = plant_engine_step(P, pilot.throttle_right, v(2), v(4), air, V, dt);
thrust = [thrust_left; thrust_right];
ab_level = max(ab_left, ab_right);
v_new = [rpm_left; rpm_right; ab_left; ab_right];
end

function x = fcs_state_init()
% FCS_STATE_INIT  Starting values of the control laws' memory. Timers start "long ago" so nothing is held at the start.
x.rf_w = 0;               % Refuelling blend, 0..1
x.tl_aoa_ref = 0;         % Takeoff-Landing AoA reference, rad
x.tl_was = 0;
x.rf_level_pitch = 0;     % attitude of level flight for the Refuelling law, rad
x.rf_pitch_was = 0;
x.rf_bank_target = 0;
x.rf_roll_on = 0;
x.pitch_free_t = 0;       % s with the pitch stick centred
x.roll_free_t = 0;
x.ped_off_t = 99;         % s with the pedal centred
x.ped_free_t = 99;
x.sy_s = 0;               % slewed pedal
x.p_ref = 0;              % roll-rate reference model
x.p_cmd_s = 0;            % roll command after the acceleration limit
x.nz_f = 1;               % filtered g for the head-g limiter
x.r_coord_prev = 0;
x.r_coord_dot = 0;
x.q_int = 0;
x.p_int = 0;
x.r_int = 0;
x.man_w_last = 0;
x.ab_full_t = 0;
end

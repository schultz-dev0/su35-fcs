function x = fcs_state_unpack(v)
% FCS_STATE_UNPACK  Column vector from a Unit Delay back to the state struct. Order matches fcs_state_pack.
x.rf_w = v(1);
x.tl_aoa_ref = v(2);
x.tl_was = v(3);
x.rf_level_pitch = v(4);
x.rf_pitch_was = v(5);
x.rf_bank_target = v(6);
x.rf_roll_on = v(7);
x.pitch_free_t = v(8);
x.roll_free_t = v(9);
x.ped_off_t = v(10);
x.ped_free_t = v(11);
x.sy_s = v(12);
x.p_ref = v(13);
x.p_cmd_s = v(14);
x.nz_f = v(15);
x.r_coord_prev = v(16);
x.r_coord_dot = v(17);
x.q_int = v(18);
x.p_int = v(19);
x.r_int = v(20);
x.man_w_last = v(21);
x.ab_full_t = v(22);
end

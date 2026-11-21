function v = fcs_state_pack(x)
% FCS_STATE_PACK  State struct to the column vector a Unit Delay carries. Order matches fcs_state_unpack.
v = [
    x.rf_w
    x.tl_aoa_ref
    x.tl_was
    x.rf_level_pitch
    x.rf_pitch_was
    x.rf_bank_target
    x.rf_roll_on
    x.pitch_free_t
    x.roll_free_t
    x.ped_off_t
    x.ped_free_t
    x.sy_s
    x.p_ref
    x.p_cmd_s
    x.nz_f
    x.r_coord_prev
    x.r_coord_dot
    x.q_int
    x.p_int
    x.r_int
    x.man_w_last
    x.ab_full_t
];
end

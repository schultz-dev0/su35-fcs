function P = plant_get_params()
% PLANT_GET_PARAMS  The flight model's parameter struct. The numbers live in plant/private/xfm_params.m (git-ignored); this
% wrapper is the one public way to reach them, because MATLAB only lets functions in plant/ call into plant/private/.
P = xfm_params();
end

% Puts the project folders on the MATLAB path. Run once per session: matlab -batch "setup"
function setup()
    root = fileparts(mfilename("fullpath"));
    % plant/private is not listed: MATLAB does not allow a folder named private on the path, and its functions are
    % reached through plant/plant_get_params.m
    folders = ["plant", "fcs", "models", "trim", "analysis", "export"];
    for folder = folders
        path_to_add = fullfile(root, folder);
        if isfolder(path_to_add)
            addpath(path_to_add);
        end
    end
    fprintf("su35-fcs: paths set from %s\n", root);
end

% Puts the project folders on the MATLAB path. Run once per session: matlab -batch "setup"
function setup()
    root = fileparts(mfilename("fullpath"));
    folders = ["plant", "plant/private", "fcs", "trim", "analysis", "export"];
    for folder = folders
        path_to_add = fullfile(root, folder);
        if isfolder(path_to_add)
            addpath(path_to_add);
        end
    end
    fprintf("su35-fcs: paths set from %s\n", root);
end

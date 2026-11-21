% Folder inside the installed mod (the one DCS flies) that holds the flight model files.
%   installed_mod_folder("Bin")  -> folder with SU35XFM.cfg
%   installed_mod_folder("FM")   -> folder with FlankerCFG.lua (mass, centre of gravity, inertia)
% Override the install with the environment variable SU35_MOD_DIR.
function folder = installed_mod_folder(sub_folder)
    mod_dir = getenv("SU35_MOD_DIR");
    if isempty(mod_dir)
        mod_dir = "/mnt/games2/dcs-world/dcs-world/drive_c/users/schultz/Saved Games/DCS/Mods/aircraft/Su-35_EFM_V2.0.51b";
    end
    folder = fullfile(mod_dir, sub_folder);
end

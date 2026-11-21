% Builds the reference harness: the mod's flight model as a shared library (g++) and the S-function MEX.
%
%   build_reference
%   build_reference(mod_repo, api_include, config_dir)
%
% mod_repo     the mod's repo, read only (default ~/Dev/su-35cws)
% api_include  folder that holds FM/wHumanCustomPhysicsAPI.h from the DCS install (read, never copied)
% config_dir   folder with SU35XFM.cfg (default: the installed mod's Bin)
%
% Outputs go to reference/build/ (git-ignored): libxfm_ref.so and the MEX. The mod source is compiled from its own
% path every time; nothing from it or from DCS is copied into this repo.
function build_reference(mod_repo, api_include, config_dir)
    reference_dir = fileparts(mfilename("fullpath"));
    build_dir = fullfile(reference_dir, "build");
    if ~isfolder(build_dir)
        mkdir(build_dir);
    end

    if nargin < 1 || isempty(mod_repo)
        mod_repo = fullfile(getenv("HOME"), "Dev", "su-35cws");
    end
    if nargin < 2 || isempty(api_include)
        api_include = "/mnt/games2/dcs-world/dcs-world/drive_c/Program Files/Eagle Dynamics/DCS World/API/include";
    end
    if nargin < 3 || isempty(config_dir)
        config_dir = installed_mod_folder("Bin");
    end
    mod_repo = string(mod_repo);
    api_include = string(api_include);

    main_source = fullfile(mod_repo, "src", "native", "cpp", "xfm", "su35_xfm.cpp");
    store_source = fullfile(mod_repo, "src", "native", "cpp", "xfm", "store_drag.cpp");
    api_header = fullfile(api_include, "FM", "wHumanCustomPhysicsAPI.h");
    assert(isfile(main_source), "build_reference: %s not found", main_source);
    assert(isfile(api_header), "build_reference: %s not found", api_header);

    build_flight_model_library(reference_dir, build_dir, mod_repo, api_include, main_source, store_source);
    build_sfunction(reference_dir, build_dir, api_include);

    % remembered for xfm_default_config. The two defaults are read from the mod source, not typed here.
    settings.empty_mass_kg_default = read_source_default(main_source, "empty_mass_kg");
    settings.fuel_internal_max_default = read_source_default(main_source, "fuel_internal_max");
    settings.mod_repo = mod_repo;
    settings.api_include = api_include;
    settings.config_dir = string(config_dir);
    save(fullfile(build_dir, "build_settings.mat"), "-struct", "settings");
    fprintf("build_reference: done, outputs in %s\n", build_dir);
end

% The flight model's built-in value of a parameter, read from its Params struct (SU35XFM.cfg may override it).
function value = read_source_default(source_file, name)
    text = fileread(source_file);
    token = regexp(text, "(?<![A-Za-z_])" + name + "\s*=\s*([0-9.]+)", "tokens", "once");
    assert(~isempty(token), "build_reference: default of %s not found in the flight model source", name);
    value = str2double(token{1});
end

function build_flight_model_library(reference_dir, build_dir, mod_repo, api_include, main_source, store_source)
    library = fullfile(build_dir, "libxfm_ref.so");
    % Static libstdc++: MATLAB loads its own, older one, and the library must not depend on the system's.
    % -fno-gnu-unique lets dlclose unload the library, so each run starts from fresh globals.
    % -w: the mod's source is not ours to fix, so its warnings are not shown here.
    command = sprintf(['g++ -std=c++17 -O2 -fPIC -shared -fno-gnu-unique -static-libstdc++ -static-libgcc -w ' ...
        '-I"%s" -I"%s" -I"%s" -DXFM_MAIN_SOURCE="\\"%s\\"" "%s" "%s" -o "%s"'], ...
        fullfile(reference_dir, "shim"), fullfile(mod_repo, "src", "native", "cpp"), api_include, main_source, ...
        fullfile(reference_dir, "xfm_internals.cpp"), store_source, library);
    fprintf("build_reference: compiling the flight model library\n");
    [status, output] = system(command);
    if status ~= 0
        error("build_reference: g++ failed:\n%s", output);
    end
end

function build_sfunction(reference_dir, build_dir, api_include)
    fprintf("build_reference: compiling the S-function\n");
    mex_file = fullfile(build_dir, "xfm_sfun." + mexext);
    try
        mex("-silent", "-outdir", build_dir, "-ldl", "CXXFLAGS=$CXXFLAGS -std=c++17 -fPIC", ...
            "-I" + api_include, fullfile(reference_dir, "xfm_sfun.cpp"));
    catch problem
        % mex tests the file by loading it as a plain MEX function, which an S-function is not: the file is still
        % written. Anything else is a real failure.
        if ~contains(problem.message, "is not a MEX file") || ~isfile(mex_file)
            rethrow(problem);
        end
    end
end

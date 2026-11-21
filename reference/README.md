# Reference harness

Runs the Su-35 mod's **current flight model and FCS (C++, unchanged)** inside Simulink. Simulink does what DCS
does: it integrates the 6-DOF motion from the force and moment the flight model returns and feeds the new state
back. The result is the reference the rebuilt plant (`plant/`) and the control laws (`fcs/`) are compared with.

## Privacy rule

The flight model is built **from the mod's source at its own path** every time. Nothing from the mod or from DCS is
copied into this repo: not its source, not its parameters (mass, inertia and centre of gravity are read from the
installed mod at run time and live only in the workspace), not the DCS API header (included from the DCS install).
`reference/build/` (git-ignored) holds the compiled library, the MEX, logs and plots. `shim/windows.h` is our own.

## Build

```matlab
setup
addpath reference
build_reference                      % defaults: mod ~/Dev/su-35cws, DCS API include, installed mod's Bin folder
build_reference(mod_repo, api_include, config_dir)   % or give the three paths
make_reference_model                 % writes reference/xfm_reference.slx
```

Needs g++ on the path and a MATLAB-supported `mex` setup. `build_reference` does two things:

1. Compiles `xfm_internals.cpp` (which `#include`s the mod's `su35_xfm.cpp`) and the mod's `store_drag.cpp` with
   g++ into `build/libxfm_ref.so`, using `shim/windows.h` in place of `<windows.h>`. Static libstdc++, so MATLAB's
   older one does not matter.
2. Builds the S-function `xfm_sfun.cpp` with `mex`. mex prints "is not a MEX file" for an S-function; the build
   ignores that message when the file was written.

The installed mod is `SU35_MOD_DIR` or the path in `installed_mod_folder.m`. The flight model reads its config from
`<mod>/Bin/SU35XFM.cfg` (there is no `FM/FlankerAA.lua`); mass, centre of gravity and inertia come from
`<mod>/FM/FlankerCFG.lua`; empty mass and fuel capacity from the cfg, else from the flight model's built-in values
(read from its source at build time).

## Run

```matlab
[cfg, throttle] = find_level_trim(altitude_m = 3000, ias_kmh = 600);   % level start: angle of attack + throttle
result = run_reference(cfg, pilot_timeseries.hold(throttle = throttle), 20);
plot(result.t, result.altitude)

check_reference                      % level flight (clean, gear down) and step responses -> build/*.png
replay_csv                           % replay a recorder CSV from DCS Logs (see the file header)
```

`result` has named fields in SI and DCS axes: `vb`, `w` (wx roll, wy yaw + nose left, wz pitch), `alpha`, `beta`,
`nz`, `force`, `moment`, `mass`, `inertia`, the flight model's surfaces (`stab_deg`, `flaperon_deg`, `rudder_deg`,
`le_flap_deg`, `nozzle_*_deg`, `stab_diff_deg`), `thrust_left/right`, `gear_pos`, and `draw` (see below).

## How it is wired

```
Pilot (From Workspace) -> XFM flight model (S-function) -> force, moment (DCS axes)
                                  ^                              |
                                  |                       gain: DCS -> aerospace axes
   state (DCS axes), air  <-------+                              v
   ^                                                  + weight (DCM x m g)
   |                                                             v
   +---- gains: aerospace -> DCS axes <---- 6-DOF (Euler Angles, Aerospace Blockset)
   +---- ISA atmosphere (altitude from the position)
```

* **Axes.** The S-function is all DCS axes (x forward, y up, z right). The 6-DOF block is aerospace (x forward,
  y right, z down). Gain blocks convert at the edges: DCS to aero `[x; z; -y]`, aero to DCS `[x; -z; y]`, for forces,
  moments, velocities, rates and position. A rate about DCS y (+ nose left) is minus the aero yaw rate `r`.
* **Solver.** Fixed step 0.006 s (DCS's flight model step; the mod's recorder shows it), ode4, with the flight
  model's forces held over each step.
* **Delay.** The flight model runs in `mdlUpdate`, on the state at the start of the step; its force acts from the
  next step on. One step (6 ms) of delay, no algebraic loop.
* **Gravity** is added in the model (`DCM x [0; 0; m g]`), 9.80665 m/s2, flat earth. The ISA block gives
  temperature, speed of sound, pressure and density from the altitude. No wind.
* **Mass.** The 6-DOF block uses a fixed mass (the air start value) and the inertia from `FlankerCFG.lua`; the
  flight model itself sees the burning fuel (`ed_fm_change_mass`), which is under 1 % over a run. The config's
  fourth inertia number (product of inertia) is not used.
* **Euler angles** (the 6-DOF block's): no run through pitch +-90 deg.

## What the S-function calls each step

Start (once per run): `xfm_ref_set_folders`, `ed_fm_configure`, `ed_fm_hot_start_in_air`, `xfm_ref_start_gear_down`
(air start with the gear up or down), `ed_fm_set_internal_fuel`, `ed_fm_change_mass` (fuel mass, as DCS does at
spawn). Each step, in this order:

1. `ed_fm_set_current_mass_state(mass, cg, inertia)`
2. `ed_fm_set_atmosphere(altitude, T, a, rho, p, 0, 0, 0)`
3. `ed_fm_set_current_state` (world axes: north / up / east; the quaternion is identity, the mod ignores it)
4. `ed_fm_set_current_state_body_axis` (body acceleration with gravity counted as a force, so level flight reads
   0; body velocity, rates, attitude, alpha, beta)
5. pilot commands through `ed_fm_set_command`, sent only when an input changes (as DCS does)
6. `ed_fm_simulate(0.006)`
7. `ed_fm_add_local_force`, `ed_fm_add_local_moment` (+ the `_component` forms until they return false); the
   force is at the centre of gravity in x and y, so the moment about the CG is the moment returned
8. `ed_fm_change_mass` (fuel burn), `ed_fm_set_draw_args_v2`

| Pilot input | Command |
|---|---|
| stick pitch / roll / pedals | axis 2001 / 2002 / 2003, value as in `docs/interfaces.md` |
| throttle left / right | axis 2005 / 2006, value `1 - 2 x lever` |
| gear | 430 up / 431 down |
| airbrake | 147 out / 148 in |
| limiter off | 121 (toggle, sent when the input changes) |
| TVC on, probe out | not commands in the mod (cockpit arguments): set directly by `xfm_ref_set_switches` |

The flight model's own state (actuator positions, thrust, nozzles) is read back through `xfm_ref_get_internals`
(`xfm_internals.cpp`), because `ed_fm_get_param` does not expose them. The draw arguments (`draw` output) are
the visual ones: stabilator args mix the stick with the actuator, so use the internals for surface comparisons.

## Limits and what is not modelled

* **Fresh state per run.** The flight model keeps its state in globals (one instance per process), so a plain MEX
  would need `clear mex` between runs. Here each run copies `libxfm_ref.so` to `/tmp/xfm_ref_<pid>_<n>.so`, `dlopen`s
  it and `dlclose`s it at the end, so the globals start clean and `clear mex` is not needed. Two models running at
  the same time in one session each get their own copy.
* **Airborne only.** No ground, gear contact, nose wheel or brakes. `ed_fm_suspension_feedback` is never called.
  A run below the ground just keeps falling. The air start with gear down skips the first-frame logic that would
  retract it.
* **No cockpit.** The mod's cockpit parameter functions are null in the harness (`GetModuleHandleA` returns null),
  and every use is guarded, so the flight model skips them. Not available: autopilot modes and AUTO route steering,
  the autothrottle, GCAS terrain input, refuelling mode (its switch is a cockpit argument), store loadout and drag
  table (stores are only mass-based drag), engine start logic and the systems model (electric power, hydraulics),
  the engine mode switch (combat mode), the wheel-brake and parking-brake levers. TVC and the probe are set by
  switch calls (see above).
* **No damage, wind, turbulence, clouds, ground height** (`ed_fm_set_surface` is not called; ground height is 0).
* **Air start engines.** `ed_fm_hot_start_in_air` leaves both engines at 90 % rpm with the afterburner off
  (about 40 kN). The S-function's parameter 6, `cfg.engines = [use, rpm_l, rpm_r, ab_l, ab_r, thrust_l, thrust_r]`,
  overrides that after the start call. `replay_csv` sets it from the recorded first row (`match_engines`, default
  on). Without it a replay that starts in full afterburner loses about 5 s of thrust while it lights.
* **Atmosphere.** The model's air is ISA plus `cfg.temperature_offset_k` and times `cfg.pressure_ratio`
  (0 and 1 = standard day). The recordings have no temperature column, so `replay_csv` derives them from the first
  row: T from TAS / Mach, density from qbar, pressure from the gas law (`match_atmosphere`, default on). The two
  logged flights were ISA +18 K and +12 K, 0 to 4 % above ISA pressure. Held constant over the replay.
* **Residual: less g per stick.** With engines and air matched, the sim pulls about 12 to 15 % less g than the
  recording for the same stick (021412: nz RMS 0.49 g, 022907: 0.27 g; the 022907 pull is 2.4 g sim against 2.75 g
  recorded). That is the leftover altitude loss, -112 m at 10 s. It is not the start state (starting 10 to 20 s
  earlier gives the same), the step size, TVC, limiter or autopilot flags, or a cockpit read. It looks like the
  installed DLL or `FlankerCFG.lua` data differing from this source and config, or DCS data the harness cannot
  see. Not resolved.
* **Files the flight model touches.** `SU35XFM.cfg` is read from the config folder. Its log goes to `build/`
  (`SU35XFM.log`). The flight recorder (`xfm-*.csv`) is refused by the shim, so nothing is written into the DCS
  install.
* The mod source is compiled with `-w` (its warnings are not ours); the shim redefines `fopen` to turn the
  backslash paths into slashes.

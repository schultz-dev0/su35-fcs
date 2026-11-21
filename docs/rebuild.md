# Rebuild of the Su-35 FCS and airframe

Baseline model of the mod's flight control system and airframe, rebuilt from `su35_xfm.cpp` (su-35cws,
`src/native/cpp/xfm/`) as MATLAB functions and a Simulink model. It is the starting point for the redesign.

- Build: `models/make_rebuild_model.m` writes `models/su35_rebuild.slx` (fixed step 0.006 s, ode4 for the 6DoF).
- Start: `trim/trim_level_flight.m` gives the air start for a speed, altitude and configuration.
- Checks: `analysis/check_rebuild.m` (PNGs in `analysis/out/`). Linear models: `trim/linearize_rebuild.m`.
- Mod numbers live only in `plant/private/xfm_params.m` (git-ignored, one struct, each value with its source line).
  Config is the installed `Bin/SU35XFM.cfg`; mass and inertia are from `FlankerCFG.lua:106`
  (Ix 53044, Iy 388509 yaw, Iz 431692 pitch, Ixy ignored). Mass is 24897 kg (empty 19147 + half the internal fuel), the
  value the reference harness starts with, so the two models weigh the same.

Line numbers below are `su35_xfm.cpp`.

## Scope

Airborne only, in the Flight, Takeoff-Landing (gear down), Refuelling and TVC Maneuver SDU modes. Fixed ISA, flat
ground at sea level (height over ground = altitude).

## Interfaces and conventions

| Item | Choice |
|---|---|
| Axes | DCS body axes in the FCS. The 6DoF block is aerospace; forces F = (Fx, Fz, -Fy), moments M = (Mx, Mz, -My), rates p = wx, q = wz, r = -wy, inertia diag(Ixx, Izz, Iyy). Alpha and beta keep their sign. Weight is added in the plant: m g [-sin(theta), cos(theta) sin(phi), cos(theta) cos(phi)]. |
| `Surfaces.stab` | Trailing edge down positive, so it is minus the mod's stab (the mod's + is a nose-up moment). |
| Rudder, `flaperon_diff` | Same sign as the mod. `stab_diff` = 0. `flaperon_sym` = flap_pos x 15 deg. `le_flap` is the target, no lag (visual only). |
| Nozzles | Left/right pitch = noz_pitch -/+ noz_roll; both yaw nozzles = noz_yaw. |
| Frame delays | `nz`, `ny_lat` and the previous nozzle pitch/yaw go through a Unit Delay (DCS gives the FCS last frame's values). `gamma` is derived in the FCS from alpha, beta, theta, phi. |
| Sample times | Every MATLAB Function block and Unit Delay runs at `dt_fcs` = 0.006 s. FCS state is a 22-element packed vector. |

## Channel by channel

| Channel | Source | Where | Fidelity |
|---|---|---|---|
| Stick shaping, dead zone | 2186-2260 | `fcs_stick_shape` | Exact. |
| Pitch: nz demand to pitch-rate demand `q_cmd = (nz_cmd - cos th cos phi) G / V` (V floored at 30), slow-speed stick floor, pedal-slip attenuation, fast-roll g hold, AoA limiter with lead | 2186-3100 | `fcs_commands` | Exact for the reachable branches. TVC-on limits are in `fcs_tvc_*` (rate, nz, AoA, share, Mach gate). |
| Pitch limits with TVC off | 2121-2199 | `fcs_nz_limit_tvc_off` | Exact. |
| Roll: team roll-rate reference model, head-g limiter, vertical no-roll rule, refuelling bank hold, roll acceleration and rate limit from rudder authority | 690-723, 3104-3201 | `fcs_team_roll_step`, `fcs_commands` | Exact. |
| Yaw: sideslip feedback, pedal rate with slip limiter (TVC), gravity-turn term, roll coordination `p tan(alpha)` and its rate feed-forward | 3202-3285 | `fcs_commands` | Exact for `tvc_ped_direct = 0` (see below). |
| Roll into the pedal (`ped_roll`): plain rudder rolls the jet toward the yaw | 3322-3394 | `fcs_commands` | Ported after the harness comparison (fix 2 below). Not ported: the knife-edge reversal latch `rev_on`, and the `rot_v` latch (`rot_v_in`). |
| Integrators (q, p, r) and loop gains | 3499-3530 | `fcs_integrators`, Simulink Sum/Gain blocks | Exact. Loop is bandwidth x error + integrator gain x integrator; yaw error = r_cmd + r_yaw. |
| Allocation to stab, flaperons, rudders, nozzles, airbrake | 3640-3880 | `fcs_allocation` | Exact, including the Maneuver mode weight (`man_w` = clamp((600 - IAS)/100); the integrator reset runs at the start of the next frame, same result). |
| Surfaces and actuators | 3888-3916 | `fcs_surfaces`, `plant_actuators_step` | Exact (rate and position limits). |
| Aero forces and moments | 3924-4034 | `plant_forces_moments`, `plant_aero_bare`, `plant_cl_of`, `plant_cm_extra`, `plant_eff*`, `plant_table`, `plant_ground_effect` | Exact for the airborne path. |
| Engines | 777-931 | `plant_engine_step`, `plant_engines_step` | Exact. Afterburner light level is in the params. No fuel burn, so mass stays constant. |
| Slow systems (gear, flaps, flap brake, airbrake) | in simulate | `plant_slow_step`, `plant_slow_vec_step` | Exact timing. No probe travel: Refuelling `want` uses `Pilot.refuel_probe` directly. |
| Mass properties | `FlankerCFG.lua:106` | `plant_mass_properties` | Diagonal inertia only. |

## Mode flags

- `tvc_ok` = `Pilot.tvc_on` and thrust above 1000 N.
- Refuelling `want` = `refuel_probe` and IAS < 1000 km/h and M <= 0.85 and gear_pos < 0.5.
- Maneuver `man_w` is non-zero only with `tvc_ok`, gear up and not refuelling.
- `limiter_off` only switches off the AoA anticipation limiter. The post-stall blend is not rebuilt.
- `tvc_ped_direct` is 0 in the installed cfg (the C++ default is 1). The rate-demand pedal law with the slip limiter runs
  and the `ped_dir` terms vanish. `tvc_alloc` = `tvc_ok`. With TVC off, `g_law` = 1 and the nozzle feed-forwards are 0.

## Left out

- Display and airshow laws: pancake and pirouette `att_w`, vertical spin `vert_*`, knife-edge reversal `rev_*`,
  flips, kulbit and cobra (`direct`, `tvc_flip`), post-stall pedal rotation (`rot_w`, `pw`, `rot_v` latch).
- GCAS and recovery, autopilot and autoland, bank hold, damage, FCS BIT, ground handling, smoothness logging.
- `roll_schedule` and `roll_curve` (the non-team roll path), `g_override`, level-2 airbrake.
- Buffet, wing flex, non-ISA correction, fuel burn (the harness burns fuel; the mass change is 0.06 % in 20 s).

## Interpreted or simplified

- Speeds are indicated (IAS), `TAS = IAS sqrt(rho0 / rho)` as the harness does it (`trim/trim_ias_to_tas.m`). At 3000 m,
  600 km/h IAS is 696 km/h TAS (193.5 m/s), and 320 km/h IAS is 371 km/h TAS (103.2 m/s).
- Trim solves alpha and throttle from the force balance along body x and z, with the surfaces the FCS allocates. The
  pitch moment balance is left to the dynamic inversion in the FCS.
- Level flight has a small positive `q_cmd`, because `q_cmd` uses `cos(theta) cos(phi)` in the C++. It is the mod's own
  behaviour, not a bug in the port.
- Flat ground at sea level, so ground effect uses the altitude.
- The harness's `stab_deg` is the flight model's `stab`, which is + nose-up moment (a trailing edge up). The harness
  README says "+ trailing edge down". The rebuild's `Surfaces.stab` is + trailing edge down, so the comparison flips the
  harness sign. The forces and responses match, so the physics is the same; only the label in `reference/` is off.

## Comparison with the reference harness

`analysis/compare/compare_with_reference.m` runs both models from the same state at 3000 m, 600 km/h IAS clean and
320 km/h IAS gear down, with the same mass, inertia and engines at the start. The harness is trimmed by its own
`find_level_trim` (alpha, throttle); the rebuild is then started at that alpha, speed and throttle. Scenarios: 20 s hands
off; pitch doublet +-0.3 for 1 s each; roll step 0.5 and 1.0 for 2 s; pedal step 0.3 and 1.0 for 3 s. PNGs and the full
tables (max and RMS difference of 16 signals) are in `analysis/compare/out/` (git-ignored).

Trim at the same IAS, each model's own:

| Case | Harness alpha / throttle | Rebuild alpha / throttle / stab |
|---|---|---|
| clean 600 | 2.51 deg / 0.306 | 2.57 deg / 0.307 / -0.18 deg |
| gear down 320 | 8.17 deg / 0.370 | 7.58 deg / 0.324 / -0.49 deg |

The clean trims agree. The gear-down harness "trim" is not an equilibrium (see below), so its alpha and throttle are
higher than the rebuild's force balance. The comparison uses the harness's alpha and throttle for both.

### Fixes and what was not a fix

1. **Mass.** The rebuild used 25300 kg (the cfg's normal takeoff mass). The harness starts at 24897 kg. Set to the
   harness value.
2. **Roll into the pedal was missing.** Pedal steps at 600 IAS rolled the harness 7 deg (0.3 pedal) and 22 deg (full), the
   rebuild 0.4 and 1 deg. Cause: `ped_roll` law (cpp:3322-3394, p_cmd = ped_roll x yaw rate toward the pedal, faded by
   speed, level-flight window, bank room) was left out. Ported into `fcs_commands` (reachable branch; not the knife-edge
   latch). Pedal-case bank difference went from 9.6 / 29.2 deg to 0.03 / 0.11 deg.
3. **Not a rebuild fix: the harness's gear-down air start leaves the landing flaps up.** `xfm_ref_start_gear_down` sets
   only the gear; the flaps extend over 4 s (flap_pos 0 to 1, leading edge 0.5 to 23 deg) while the jet flies. That
   gives 0.8 g and a 12 deg/s pitch transient at the start. The comparison starts the rebuild's flaps up as well. Before
   that, gear-down hands-off differed by 32 m in altitude and 2.3 deg in pitch; after, 0.5 m and 0.04 deg.
4. **Not a rebuild fix: the first harness sample** is a copy of its second (no model output yet); the tables skip it.

### Differences left, per scenario

Max / RMS difference after the fixes (angles deg, rates deg/s, V m/s, altitude m):

| Scenario | alpha | beta | p | q | r | nz | phi | theta | V | altitude | stab | rudder |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clean hands off | .017/.003 | 0 | 0 | 0 | 0 | .006/.001 | 0 | 0 | .001/.001 | .06/.06 | .001/.000 | 0 |
| clean pitch doublet | .03/.008 | 0 | 0 | .11/.014 | 0 | .010/.003 | 0 | .017/.004 | .004/.002 | .09/.05 | .36/.03 | 0 |
| clean roll 0.5 | .017/.007 | .03/.011 | .95/.14 | .016/.003 | .14/.03 | .006/.002 | .21/.06 | .05/.02 | .015/.006 | .38/.17 | .05/.005 | .45/.05 |
| clean roll full | .07/.03 | .09/.04 | 1.8/.52 | .27/.09 | .43/.10 | .023/.010 | .69/.22 | .09/.05 | .05/.02 | .49/.16 | .27/.03 | 1.0/.19 |
| clean pedal 0.3 | .017/.004 | .025/.007 | .10/.025 | .001/.000 | .06/.018 | .006/.001 | .03/.016 | .002/.001 | .003/.002 | .06/.05 | .007/.000 | .49/.04 |
| clean pedal full | .02/.008 | .07/.026 | .30/.07 | .011/.005 | .16/.06 | .006/.003 | .11/.08 | .03/.012 | .03/.017 | .12/.05 | .011/.002 | .49/.08 |
| gear hands off | .020/.002 | 0 | 0 | .067/.011 | 0 | .002/.001 | 0 | .036/.021 | .037/.027 | .49/.37 | .83/.02 | 0 |
| gear pitch doublet | .020/.007 | 0 | 0 | .067/.025 | 0 | .002/.001 | 0 | .057/.027 | .036/.023 | .46/.29 | .83/.04 | 0 |
| gear roll 0.5 | .020/.004 | .028/.013 | .29/.05 | .067/.017 | .125/.027 | .002/.001 | .08/.04 | .05/.03 | .034/.019 | .46/.26 | .83/.03 | .91/.16 |
| gear roll full | .022/.008 | .36/.10 | .97/.26 | .18/.06 | .67/.23 | .002/.001 | 359*/17* | .06/.03 | .03/.015 | .41/.26 | .83/.07 | 3.3/.62 |
| gear pedal 0.3 | .020/.003 | .038/.010 | 0 | .067/.014 | .10/.024 | .002/.001 | .004/.002 | .04/.03 | .034/.022 | .46/.29 | .83/.03 | .55/.12 |
| gear pedal full | .020/.003 | .14/.043 | 0 | .067/.015 | .24/.08 | .002/.001 | .012/.006 | .036/.025 | .029/.018 | .44/.28 | .83/.03 | .95/.26 |

\* The 359 deg is the bank angle wrapping through 180 deg: the two models are within 1 deg of each other.

What remains is timing. The stab difference of 0.83 deg is in the first frames (the harness's FCS settles from zero
integrators while the rebuild starts from its trimmed integrators). The roll and rudder differences peak where the
roll stops, at the rate-limit edge: p differs by 1.8 deg/s of 203 (0.9 %) where the roll rate falls by 540 deg/s^2 (one
step is 3.3 deg/s), and the rudder leaves its 25 deg stop one to two frames apart. The harness runs the flight model on
the state at the start of each step and applies the force one step later (6 ms); the rebuild's Unit Delays do not match
that exactly. Not changed: this is the harness's way of stepping, and DCS has its own.

### The three questions

- **Gear-down hands-off drift.** The harness drifts +39.4 m in 20 s from its start; the rebuild started the same way
  drifts +39.7 m (V 100.59 / 100.58 m/s, pitch 12.37 / 12.34 deg). The cause is the flap transient above plus the
  harness trim being fitted over seconds 3 to 10 while the flaps were still moving: it is a start-up artefact, not
  an equilibrium. (The rebuild's own earlier "drift of -65 m" was my check script never sending the gear command, so
  the gear retracted; fixed in `check_rebuild`.) Started from its own force-balance trim with the flaps down, the
  rebuild drifts -20 m in 20 s with V +0.6 m/s.
- **Gear-down Dutch roll.** The harness does not ring. After the 0.3 pedal is released at 5 s, beta goes from -3.4 deg
  through zero at about 7.3 s, overshoots to +0.15 deg and is back through zero at about 8.5 s; the rebuild is within
  0.04 deg of it throughout. The
  zeta 0.05 earlier was the Simulink Control Design artefact below, not the model: the final closed-loop Dutch roll is
  zeta 0.55 with the gear down.
- **Roll rate.** Full roll stick at 600 IAS: harness 203.4 deg/s, rebuild 203.4 deg/s. Half stick: 46.3 / 46.2. Gear
  down at 320 IAS full: 107.9 / 107.8; half: 18.2 / 18.2.

## Linearisation

`trim/linearize_rebuild.m` takes both loops from one operating point, a snapshot 10 s into the hands-off run.

- The cause of the earlier unstable lateral pair (+32 and +6 1/s). It is not in the nonlinear model: a pedal pulse at
  10 s (beta 0.3 deg, r 0.9 deg/s) decays, and a finite-difference Jacobian of the model's own two-step map at that
  point (`one step = 0.006 s`, 58 states, central differences) has no eigenvalue outside |z| = 1.00003. It is in
  Simulink Control Design's default block-by-block algorithm. Replacing, row group by row group, the block-by-block
  Jacobian with the finite-difference one, the instability goes away only when the 6DoF's p, q, r rows are replaced:
  the block-by-block `p <- p` entry is 1.26 per step against 0.965 in the model. Changing the perturbation size or
  the rate conversion method, or zeroing the roll coordination, the coordination-rate clip, the beta feedback or the
  integrator gains, left the pair in place. A finite-difference check of `fcs_commands` alone (steps 1e-9 to 1e-3) is
  smooth, so the FCS blocks are not the cause. I did not find what the block-by-block algorithm does wrong in those
  rows. The model itself is not at fault.
- The fix is a linearisation option: `LinearizationAlgorithm = 'numericalpert'`, which perturbs the whole model as a
  simulation would. It needs root ports, which `models/add_linearization_ports.m` adds in memory (three stick inports,
  one state outport) and which are never saved. Pitch and lateral modes now come out of one model at one operating point.
- The stick dead zone makes the stick-to-state gains zero for the perturbation, so modes are sorted by the states
  that carry them (longitudinal: u, w, theta, q; lateral: v, phi, p, r), not by what a stick reaches.
- The open loop (control laws cut at the `Surfaces_builder` output) has no control laws in it and uses the default
  algorithm.
- The model has no heading state in the reduced linear model, so there is no spiral mode to report.

### Poles (rad/s, continuous equivalent)

| Mode | Clean 600 km/h IAS | Gear down 320 km/h IAS |
|---|---|---|
| Short period, open loop | -0.84 +- 0.82i (wn 1.17, zeta 0.72) | -0.46 +- 0.45i (wn 0.64, zeta 0.71) |
| Phugoid, open loop | -0.003 +- 0.060i | one real pair, +0.071 and -0.094 |
| Pitch, closed loop | real: -0.27, -0.79 (x2), -0.95, -3.7 | -0.25, -0.63 (x2); and -2.15 +- 2.71i (wn 3.5, zeta 0.62) |
| Phugoid, closed loop | -0.001, -0.011 | -0.013 +- 0.135i (zeta 0.09) |
| Dutch roll, closed loop | -1.44 +- 2.14i (wn 2.58, zeta 0.56) | -1.40 +- 2.10i (wn 2.53, zeta 0.55) |
| Dutch roll, open loop | -0.35 +- 2.36i (wn 2.39, zeta 0.15) | -0.28 +- 1.41i (wn 1.43, zeta 0.20) |
| Roll, closed loop | -5.45 (also -0.33, -0.55, -25.4, -27.1) | -5.45 (also -0.33, -0.55, -17.6, -27.1) |
| Roll subsidence, open loop | -4.3 and -0.66 | -2.1 and -0.62 |

The closed-loop pitch response is a rate command, so there is no short-period oscillation to read: the short
period is the open-loop pair, and the closed loop replaces it by real poles. The gear-down open-loop phugoid is
real and unstable (+0.071); the closed loop stabilises it. Some closed-loop poles (-0.33, -0.55, -5.45, the
0.002 pair) are integrator, coordination and loop-bandwidth modes I have not named one by one.

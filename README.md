# su35-fcs

Flight control law design for a Su-35-class fly-by-wire fighter, in MATLAB and Simulink. The laws are designed and
tuned here against a 6-DOF model of the airframe, then hand-ported to C++ in the flight model of a DCS World mod.

Status: setting up. Nothing designed yet.

## Why

Tuning control laws by flying them in a simulator costs a flight per change. Here a change is a linearisation, a step
response and a handling-qualities check, in seconds. The simulator flights are then a check of the result, not the
design loop.

## Plan

1. **Plant.** A 6-DOF model of the airframe: aerodynamics, inertia, engines, thrust vectoring, actuators.
2. **Plant check.** Recorded flights from the simulator (pilot inputs and the aircraft's response at 50 Hz) are
   replayed through the plant. It is tuned until the two agree.
3. **Control laws.** The structure follows the published description of the Su-30SM FBW (SDU-10MK): pitch channel with
   AoA and g limiting, roll damper, yaw damper, directional stability augmentation and the roll-yaw interconnect, and
   the Flight / Takeoff-Landing / Refuelling / Maneuver modes.
4. **Design and tuning.** Linearised at trim points across the envelope; checked against MIL-STD-1797A / MIL-F-8785C
   handling qualities.
5. **Hand-off.** Gains, schedules and reference step responses are exported for the C++ port and its tests.

## Layout

| Path | What |
|---|---|
| `plant/` | The airframe model and its parameters |
| `plant/private/` | Parameters taken from the mod's flight model; generated locally, never committed |
| `fcs/` | The control laws |
| `trim/` | Trim and linearisation |
| `analysis/` | Handling-qualities checks |
| `data/` | Flight recordings for the plant check (large files are not committed) |
| `export/` | What the C++ port consumes: gains, schedules, reference responses |
| `docs/` | Design notes and the source list |

## Running

MATLAB R2025b with Simulink, Control System Toolbox, Simulink Control Design and Aerospace Blockset.

```sh
matlab -batch "setup"
```

## Sources

See [`docs/sources.md`](docs/sources.md). Manuals and course material are cited, not included.

## Licence

All rights reserved. See [`LICENSE`](LICENSE).

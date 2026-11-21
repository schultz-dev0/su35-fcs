# Interfaces

The signals shared by the reference harness (`reference/`), the plant (`plant/`) and the control laws (`fcs/`). All
models use these names and units, so any block can be swapped for its counterpart and compared.

## Axes and units

- Body axes as in DCS: **x forward, y up, z right**. Rates `p` (roll, about x), `r_yaw` (about y), `q` (pitch, about
  z) in rad/s, the same signs as the mod's flight model. Where a block uses aerospace axes (x forward, y right, z
  down), it converts at its edges and says so.
- SI inside the models: m, m/s, rad, N, N·m, kg, Pa. km/h and degrees only in plots and logs.

## Pilot inputs (bus `Pilot`)

| Signal | Range | Meaning |
|---|---|---|
| `stick_pitch` | -1..1 | + aft |
| `stick_roll` | -1..1 | + right |
| `pedals` | -1..1 | + right |
| `throttle_left`, `throttle_right` | 0..1 | lever position (the afterburner zone is at the top, as in the mod) |
| `gear_cmd` | 0/1 | 1 = down |
| `airbrake_cmd` | 0/1 | 1 = out |
| `tvc_on` | 0/1 | thrust vectoring enabled |
| `limiter_off` | 0/1 | AoA/g limiter override |
| `refuel_probe` | 0/1 | probe out (Refuelling mode) |

## Aircraft state (bus `State`)

| Signal | Unit | Meaning |
|---|---|---|
| `V` | m/s | true airspeed |
| `alpha`, `beta` | rad | angle of attack, sideslip (beta + = nose left of the airflow, as in the flight model) |
| `p`, `q`, `r_yaw` | rad/s | body rates, DCS axes |
| `phi`, `theta`, `psi` | rad | bank, pitch, heading |
| `nz` | g | normal load factor (+ up) |
| `ny_lat` | g | lateral load factor (+ right) |
| `mach`, `qbar` | -, Pa | Mach number, dynamic pressure |
| `rho`, `altitude` | kg/m³, m | air density, altitude above sea level |
| `mass`, `Ixx`, `Iyy`, `Izz` | kg, kg·m² | mass and inertia (DCS axes: Ixx roll, Iyy yaw, Izz pitch) |
| `gear_pos`, `flap_pos`, `airbrake_pos` | 0..1 | positions |
| `thrust_left`, `thrust_right` | N | engine thrust |

## FCS outputs (bus `Surfaces`)

| Signal | Unit | Meaning |
|---|---|---|
| `stab` | rad | symmetric stabilator, + trailing edge up (nose up), as the flight model's `stab_deg` |
| `stab_diff` | rad | differential stabilator |
| `flaperon_diff` | rad | differential flaperon (roll) |
| `flaperon_sym` | rad | symmetric flaperon (flap droop, DLC) |
| `rudder` | rad | + nose right |
| `le_flap` | rad | leading-edge slats |
| `nozzle_pitch_left`, `nozzle_pitch_right`, `nozzle_yaw_left`, `nozzle_yaw_right` | rad | thrust vectoring |

Each surface's sign and how the flight model turns it into a moment is documented at the block that uses it.

## Plant outputs

Forces and moments about the centre of gravity in body axes (N, N·m), fed to a 6-DOF equations-of-motion block. The
reference harness gets them from the mod's flight model through its `ed_fm_*` API; the rebuilt plant computes them
in MATLAB.

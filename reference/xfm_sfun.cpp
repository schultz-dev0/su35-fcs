// Level-2 C MEX S-function that runs the Su-35 mod's flight model + FCS (C++, unchanged) the way DCS does.
// Simulink integrates the 6-DOF motion; this block feeds the flight model the state and the pilot inputs, calls
// ed_fm_simulate and hands back the forces and moments it returns.
//
// The flight model lives in libxfm_ref.so (built by build_reference.m from the mod's source). Each run loads a
// fresh copy of that library, so every run starts from the flight model's initial state.
//
// Everything crossing this block is in DCS body axes: x forward, y up, z right.
//
// Parameters (all from the workspace struct xfm_cfg, see xfm_default_config.m):
//   1 library path    2 config folder (SU35XFM.cfg)   3 run folder (log)
//   4 settings vector [fuel_kg, empty_mass_kg, Ixx, Iyy, Izz, cg_x, cg_y, cg_z, gear_down_at_start]
//   5 step time dt    (sample time and the dt given to ed_fm_simulate)
//   6 engines at the air start [use, rpm_l, rpm_r, ab_l, ab_r, thrust_l, thrust_r]  (use = 0: the start routine's)
//
// Inputs
//   1 pilot[10]   stick_pitch, stick_roll, pedals, throttle_left, throttle_right, gear_cmd, airbrake_cmd,
//                 tvc_on, limiter_off, refuel_probe          (docs/interfaces.md, bus Pilot)
//   2 state[15]   body velocity (3), body rates wx wy wz (3), roll pitch yaw (3), world position north up east (3),
//                 world velocity north up east (3)
//   3 air[4]      temperature K, speed of sound m/s, pressure Pa, density kg/m3
// Outputs
//   1 force[3]    N at the centre of gravity
//   2 moment[3]   N m about the centre of gravity
//   3 mass[4]     mass kg, Ixx, Iyy, Izz kg m2 (what DCS was told)
//   4 internals[28]  the flight model's own state, in the order of xfm_ref_get_internals (xfm_internals.cpp)
//   5 draw[16]    the draw arguments the flight model sets (kDrawArgs)
//   6 derived[8]  speed m/s, alpha rad, beta rad, mach, qbar Pa, nz g, ny g, altitude m
#define S_FUNCTION_NAME xfm_sfun
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#include <dlfcn.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#include "FM/wHumanCustomPhysicsAPI.h"

namespace {

const double kGravity = 9.80665;
const int kPilotCount = 10;
const int kStateCount = 15;
const int kAirCount = 4;
const int kInternalCount = 28;
const int kDrawArgCount = 16;
const int kDrawArgBufferSize = 5000;

// Draw arguments passed back: stabilators 15/16, leading edge 13/14, flaps 9/10, flaperons 11/12, rudders 17/18,
// afterburners 28/29, nozzle pitch 2015/2016 and yaw 2017/2018.
const int kDrawArgs[kDrawArgCount] = {9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 28, 29, 2015, 2016, 2017, 2018};

enum Param {
    kParamLibrary = 0,
    kParamConfigDir,
    kParamRunDir,
    kParamSettings,
    kParamStep,
    kParamEngines,
    kParamCount
};

enum Setting {
    kFuel = 0,
    kEmptyMass,
    kIxx,
    kIyy,
    kIzz,
    kCgX,
    kCgY,
    kCgZ,
    kGearDownAtStart,
    kSettingCount
};

enum StateIndex {
    kVx = 0, kVy, kVz,
    kWx, kWy, kWz,
    kRoll, kPitch, kYaw,
    kPosNorth, kPosUp, kPosEast,
    kVelNorth, kVelUp, kVelEast
};

typedef int (*InternalCountFn)();
typedef void (*InternalsFn)(double *);
typedef void (*FoldersFn)(const char *, const char *);
typedef void (*GearDownFn)(int);
typedef void (*SwitchesFn)(int, int);
typedef void (*EnginesFn)(double, double, double, double, double, double);

// One loaded copy of the flight model and what the harness remembers between steps.
struct Instance {
    void *library = nullptr;
    std::string library_copy;

    PFN_SIMULATE simulate = nullptr;
    PFN_SET_ATMOSPHERE set_atmosphere = nullptr;
    PFN_CURRENT_MASS_STATE set_mass_state = nullptr;
    PFN_CURRENT_STATE set_state = nullptr;
    PFN_CURRENT_STATE_BODY_AXIS set_state_body = nullptr;
    PFN_SET_COMMAND set_command = nullptr;
    PFN_FORCE_SOURCE add_local_force = nullptr;
    PFN_FORCE_SOURCE add_global_force = nullptr;
    PFN_MOMENT_SOURCE add_local_moment = nullptr;
    PFN_MOMENT_SOURCE add_global_moment = nullptr;
    PFN_FORCE_COMPONENT_SOURCE add_local_force_component = nullptr;
    PFN_MOMENT_COMPONENT_SOURCE add_local_moment_component = nullptr;
    PFN_CHANGE_MASS change_mass = nullptr;
    PFN_SET_DRAW_ARGS_V2 set_draw_args = nullptr;
    void (*configure)(const char *) = nullptr;
    PFN_HOT_START_IN_AIR hot_start_in_air = nullptr;
    void (*release)() = nullptr;
    void (*set_internal_fuel)(double) = nullptr;
    FoldersFn set_folders = nullptr;
    GearDownFn start_gear_down = nullptr;
    SwitchesFn set_switches = nullptr;
    EnginesFn set_engines = nullptr;
    InternalsFn get_internals = nullptr;

    double mass = 0;
    double cg[3] = {0, 0, 0};
    double inertia[3] = {0, 0, 0};

    // last step's results: the flight model gets the acceleration the jet had during the previous step
    double force[3] = {0, 0, 0};
    double moment[3] = {0, 0, 0};
    double internals[kInternalCount] = {0};
    double draw[kDrawArgCount] = {0};
    double derived[8] = {0};
    double previous_rates[3] = {0, 0, 0};
    double previous_velocity[3] = {0, 0, 0};
    bool have_previous = false;

    // inputs already sent, so a command goes out only when the pilot input changes (DCS does the same)
    double sent_axis[5] = {1e9, 1e9, 1e9, 1e9, 1e9};
    double sent_gear = -1;
    double sent_airbrake = -1;
    bool limiter_off = false;
};

void *must_find(void *library, const char *name)
{
    void *symbol = dlsym(library, name);
    if (!symbol) throw std::string("symbol not found in the flight model library: ") + name;
    return symbol;
}

std::string text_param(SimStruct *S, int index)
{
    const mxArray *array = ssGetSFcnParam(S, index);
    char buffer[1024];
    if (mxGetString(array, buffer, sizeof buffer) != 0) throw std::string("parameter must be a short string");
    return buffer;
}

// A copy under a unique name: dlopen returns the already-loaded library when the path is the same, which would
// keep the flight model's globals from the previous run.
std::string copy_library(const std::string &path)
{
    static int counter = 0;
    char name[256];
    snprintf(name, sizeof name, "/tmp/xfm_ref_%d_%d.so", (int)getpid(), counter++);
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::string("cannot open the flight model library ") + path + " (run build_reference first)";
    std::ofstream out(name, std::ios::binary);
    out << in.rdbuf();
    return name;
}

void load_library(Instance &fm, const std::string &path)
{
    fm.library_copy = copy_library(path);
    fm.library = dlopen(fm.library_copy.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!fm.library) throw std::string("dlopen failed: ") + dlerror();

    void *lib = fm.library;
    fm.simulate = (PFN_SIMULATE)must_find(lib, "ed_fm_simulate");
    fm.set_atmosphere = (PFN_SET_ATMOSPHERE)must_find(lib, "ed_fm_set_atmosphere");
    fm.set_mass_state = (PFN_CURRENT_MASS_STATE)must_find(lib, "ed_fm_set_current_mass_state");
    fm.set_state = (PFN_CURRENT_STATE)must_find(lib, "ed_fm_set_current_state");
    fm.set_state_body = (PFN_CURRENT_STATE_BODY_AXIS)must_find(lib, "ed_fm_set_current_state_body_axis");
    fm.set_command = (PFN_SET_COMMAND)must_find(lib, "ed_fm_set_command");
    fm.add_local_force = (PFN_FORCE_SOURCE)must_find(lib, "ed_fm_add_local_force");
    fm.add_global_force = (PFN_FORCE_SOURCE)must_find(lib, "ed_fm_add_global_force");
    fm.add_local_moment = (PFN_MOMENT_SOURCE)must_find(lib, "ed_fm_add_local_moment");
    fm.add_global_moment = (PFN_MOMENT_SOURCE)must_find(lib, "ed_fm_add_global_moment");
    fm.add_local_force_component = (PFN_FORCE_COMPONENT_SOURCE)must_find(lib, "ed_fm_add_local_force_component");
    fm.add_local_moment_component = (PFN_MOMENT_COMPONENT_SOURCE)must_find(lib, "ed_fm_add_local_moment_component");
    fm.change_mass = (PFN_CHANGE_MASS)must_find(lib, "ed_fm_change_mass");
    fm.set_draw_args = (PFN_SET_DRAW_ARGS_V2)must_find(lib, "ed_fm_set_draw_args_v2");
    fm.configure = (void (*)(const char *))must_find(lib, "ed_fm_configure");
    fm.hot_start_in_air = (PFN_HOT_START_IN_AIR)must_find(lib, "ed_fm_hot_start_in_air");
    fm.release = (void (*)())must_find(lib, "ed_fm_release");
    fm.set_internal_fuel = (void (*)(double))must_find(lib, "ed_fm_set_internal_fuel");
    fm.set_folders = (FoldersFn)must_find(lib, "xfm_ref_set_folders");
    fm.start_gear_down = (GearDownFn)must_find(lib, "xfm_ref_start_gear_down");
    fm.set_switches = (SwitchesFn)must_find(lib, "xfm_ref_set_switches");
    fm.get_internals = (InternalsFn)must_find(lib, "xfm_ref_get_internals");
    fm.set_engines = (EnginesFn)must_find(lib, "xfm_ref_set_engines");

    const int count = ((InternalCountFn)must_find(lib, "xfm_ref_internal_count"))();
    if (count != kInternalCount) throw std::string("library and S-function disagree on the internals count");
}

void unload_library(Instance &fm)
{
    if (fm.release) fm.release();
    if (fm.library) dlclose(fm.library);
    if (!fm.library_copy.empty()) remove(fm.library_copy.c_str());
}

// DCS start sequence for an air start: configure, start routine, fuel, then the mass state with the fuel added
// through ed_fm_change_mass.
void start_flight_model(Instance &fm, const std::string &config_dir, const std::string &run_dir, const double *s,
                        const double *engines)
{
    fm.set_folders(config_dir.c_str(), run_dir.c_str());
    fm.configure("");
    fm.hot_start_in_air();
    fm.start_gear_down(s[kGearDownAtStart] > 0.5 ? 1 : 0);
    if (engines[0] > 0.5) fm.set_engines(engines[1], engines[2], engines[3], engines[4], engines[5], engines[6]);

    fm.set_internal_fuel(s[kFuel]);
    fm.mass = s[kEmptyMass];
    fm.inertia[0] = s[kIxx];
    fm.inertia[1] = s[kIyy];
    fm.inertia[2] = s[kIzz];
    fm.cg[0] = s[kCgX];
    fm.cg[1] = s[kCgY];
    fm.cg[2] = s[kCgZ];

    double dm = 0, x = 0, y = 0, z = 0, ix = 0, iy = 0, iz = 0;
    if (fm.change_mass(dm, x, y, z, ix, iy, iz)) fm.mass += dm;
}

void send_axis(Instance &fm, int slot, int command, double value)
{
    if (fabs(value - fm.sent_axis[slot]) < 1e-9) return;
    fm.sent_axis[slot] = value;
    fm.set_command(command, (float)value);
}

// The pilot's inputs as the commands the flight model handles (ed_fm_set_command in su35_xfm.cpp):
// 2001 / 2002 / 2003 pitch, roll and rudder axes, 2005 / 2006 left and right throttle axes (-1 full .. +1 idle),
// 430 / 431 gear up / down, 147 / 148 airbrake out / in, 121 pitch limiter toggle.
// The TVC switch and the probe are cockpit arguments in the mod, so they go through xfm_ref_set_switches.
void send_pilot_commands(Instance &fm, const double *pilot)
{
    send_axis(fm, 0, 2001, pilot[0]);
    send_axis(fm, 1, 2002, pilot[1]);
    send_axis(fm, 2, 2003, pilot[2]);
    send_axis(fm, 3, 2005, 1.0 - 2.0 * pilot[3]);
    send_axis(fm, 4, 2006, 1.0 - 2.0 * pilot[4]);

    const double gear = pilot[5] > 0.5 ? 1 : 0;
    if (gear != fm.sent_gear && fm.sent_gear >= 0) fm.set_command(gear > 0.5 ? 431 : 430, 1.0f);
    if (fm.sent_gear < 0 && gear > 0.5) fm.set_command(431, 1.0f);
    fm.sent_gear = gear;

    const double airbrake = pilot[6] > 0.5 ? 1 : 0;
    if (airbrake != fm.sent_airbrake) fm.set_command(airbrake > 0.5 ? 147 : 148, 1.0f);
    fm.sent_airbrake = airbrake;

    const bool want_limiter_off = pilot[8] > 0.5;
    if (want_limiter_off != fm.limiter_off) {
        fm.set_command(121, 1.0f);
        fm.limiter_off = want_limiter_off;
    }

    fm.set_switches(pilot[7] > 0.5 ? 1 : 0, pilot[9] > 0.5 ? 1 : 0);
}

// Acceleration of the jet in body axes during the previous step, m/s2, gravity included as the force it is (so
// level flight reads 0). This is what DCS passes as the body acceleration; the flight model turns it into nz.
void previous_body_acceleration(const Instance &fm, double pitch, double roll, double *a)
{
    const double gravity_x = -kGravity * sin(pitch);
    const double gravity_y = -kGravity * cos(pitch) * cos(roll);
    const double gravity_z = kGravity * cos(pitch) * sin(roll);
    a[0] = fm.force[0] / fm.mass + gravity_x;
    a[1] = fm.force[1] / fm.mass + gravity_y;
    a[2] = fm.force[2] / fm.mass + gravity_z;
}

void step_flight_model(Instance &fm, const double *pilot, const double *st, const double *air, double dt)
{
    const double vx = st[kVx], vy = st[kVy], vz = st[kVz];
    const double speed = sqrt(vx * vx + vy * vy + vz * vz);
    const double roll = st[kRoll], pitch = st[kPitch], yaw = st[kYaw];
    const double altitude = st[kPosUp];

    // DCS: alpha + nose above the flow (velocity points below the nose), beta + velocity to the right
    const double alpha = atan2(-vy, vx);
    const double beta = speed > 1e-3 ? asin(fmax(-1.0, fmin(1.0, vz / speed))) : 0.0;

    double body_acc[3];
    previous_body_acceleration(fm, pitch, roll, body_acc);
    double rate_dot[3] = {0, 0, 0};
    double world_acc[3] = {0, 0, 0};
    if (fm.have_previous && dt > 0) {
        for (int i = 0; i < 3; ++i) {
            rate_dot[i] = (st[kWx + i] - fm.previous_rates[i]) / dt;
            world_acc[i] = (st[kVelNorth + i] - fm.previous_velocity[i]) / dt;
        }
    }

    fm.set_mass_state(fm.mass, fm.cg[0], fm.cg[1], fm.cg[2], fm.inertia[0], fm.inertia[1], fm.inertia[2]);
    fm.set_atmosphere(altitude, air[0], air[1], air[3], air[2], 0, 0, 0);
    // world axes: x north, y up, z east. The quaternion is not used by the mod and is left as identity.
    fm.set_state(world_acc[0], world_acc[1], world_acc[2], st[kVelNorth], st[kVelUp], st[kVelEast],
                 st[kPosNorth], st[kPosUp], st[kPosEast], rate_dot[0], rate_dot[1], rate_dot[2],
                 st[kWx], st[kWy], st[kWz], 0, 0, 0, 1);
    fm.set_state_body(body_acc[0], body_acc[1], body_acc[2], vx, vy, vz, 0, 0, 0, rate_dot[0], rate_dot[1],
                      rate_dot[2], st[kWx], st[kWy], st[kWz], yaw, pitch, roll, alpha, beta);

    send_pilot_commands(fm, pilot);
    fm.simulate(dt);

    // Collect forces and moments as DCS does. All the mod's force is one local force at (cg.x, cg.y, 0).
    double force[3] = {0, 0, 0};
    double point[3] = {0, 0, 0};
    double moment[3] = {0, 0, 0};
    fm.add_local_force(force[0], force[1], force[2], point[0], point[1], point[2]);
    fm.add_local_moment(moment[0], moment[1], moment[2]);

    // Component variants: DCS calls them until they return false. The mod returns false at once.
    for (int guard = 0; guard < 64; ++guard) {
        double fx = 0, fy = 0, fz = 0, px = 0, py = 0, pz = 0;
        if (!fm.add_local_force_component(fx, fy, fz, px, py, pz)) break;
        force[0] += fx;
        force[1] += fy;
        force[2] += fz;
        moment[0] += (py - fm.cg[1]) * fz - (pz - fm.cg[2]) * fy;
        moment[1] += (pz - fm.cg[2]) * fx - (px - fm.cg[0]) * fz;
        moment[2] += (px - fm.cg[0]) * fy - (py - fm.cg[1]) * fx;
    }
    for (int guard = 0; guard < 64; ++guard) {
        double mx = 0, my = 0, mz = 0;
        if (!fm.add_local_moment_component(mx, my, mz)) break;
        moment[0] += mx;
        moment[1] += my;
        moment[2] += mz;
    }
    // The global force and moment functions are empty in the mod; they are not called here.

    // Force moved to the centre of gravity: moment = M + (point - cg) x F
    const double rx = point[0] - fm.cg[0], ry = point[1] - fm.cg[1], rz = point[2] - fm.cg[2];
    for (int i = 0; i < 3; ++i) fm.force[i] = force[i];
    fm.moment[0] = moment[0] + ry * force[2] - rz * force[1];
    fm.moment[1] = moment[1] + rz * force[0] - rx * force[2];
    fm.moment[2] = moment[2] + rx * force[1] - ry * force[0];

    // Fuel burn: DCS asks for the mass change after simulate
    double dm = 0, mx = 0, my = 0, mz = 0, ix = 0, iy = 0, iz = 0;
    if (fm.change_mass(dm, mx, my, mz, ix, iy, iz)) fm.mass += dm;

    fm.get_internals(fm.internals);

    // Draw arguments, with zeros in (DCS fills in damage visuals the mod reads back; there is no damage here)
    static float args[kDrawArgBufferSize];
    memset(args, 0, sizeof args);
    fm.set_draw_args(args, kDrawArgBufferSize);
    for (int i = 0; i < kDrawArgCount; ++i) fm.draw[i] = args[kDrawArgs[i]];

    for (int i = 0; i < 3; ++i) {
        fm.previous_rates[i] = st[kWx + i];
        fm.previous_velocity[i] = st[kVelNorth + i];
    }
    fm.have_previous = true;

    const double mach = air[1] > 1 ? speed / air[1] : 0;
    fm.derived[0] = speed;
    fm.derived[1] = alpha;
    fm.derived[2] = beta;
    fm.derived[3] = mach;
    fm.derived[4] = 0.5 * air[3] * speed * speed;
    fm.derived[5] = body_acc[1] / kGravity + cos(pitch) * cos(roll);
    fm.derived[6] = body_acc[2] / kGravity - cos(pitch) * sin(roll);
    fm.derived[7] = altitude;
}

Instance *instance_of(SimStruct *S) { return (Instance *)ssGetPWork(S)[0]; }

void fail(SimStruct *S, const std::string &message)
{
    static char text[1100];
    snprintf(text, sizeof text, "xfm_sfun: %s", message.c_str());
    ssSetErrorStatus(S, text);
}

}  // namespace

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, kParamCount);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (int i = 0; i < kParamCount; ++i) ssSetSFcnParamTunable(S, i, SS_PRM_NOT_TUNABLE);

    if (!ssSetNumInputPorts(S, 3)) return;
    const int input_widths[3] = {kPilotCount, kStateCount, kAirCount};
    for (int i = 0; i < 3; ++i) {
        ssSetInputPortWidth(S, i, input_widths[i]);
        ssSetInputPortDirectFeedThrough(S, i, 0);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    if (!ssSetNumOutputPorts(S, 6)) return;
    const int output_widths[6] = {3, 3, 4, kInternalCount, kDrawArgCount, 8};
    for (int i = 0; i < 6; ++i) ssSetOutputPortWidth(S, i, output_widths[i]);

    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_WORKS_WITH_CODE_REUSE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    const double dt = mxGetScalar(ssGetSFcnParam(S, kParamStep));
    ssSetSampleTime(S, 0, dt);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    Instance *fm = new Instance();
    ssGetPWork(S)[0] = fm;
    try {
        const std::string library = text_param(S, kParamLibrary);
        const std::string config_dir = text_param(S, kParamConfigDir);
        const std::string run_dir = text_param(S, kParamRunDir);
        const mxArray *settings = ssGetSFcnParam(S, kParamSettings);
        if (mxGetNumberOfElements(settings) != kSettingCount) throw std::string("settings vector needs 9 elements");
        const mxArray *engines = ssGetSFcnParam(S, kParamEngines);
        if (mxGetNumberOfElements(engines) != 7) throw std::string("engines vector needs 7 elements");
        load_library(*fm, library);
        start_flight_model(*fm, config_dir, run_dir, mxGetPr(settings), mxGetPr(engines));
    } catch (const std::string &message) {
        fail(S, message);
    }
}

static void mdlOutputs(SimStruct *S, int_T)
{
    Instance *fm = instance_of(S);
    if (!fm) return;
    real_T *force = ssGetOutputPortRealSignal(S, 0);
    real_T *moment = ssGetOutputPortRealSignal(S, 1);
    real_T *mass = ssGetOutputPortRealSignal(S, 2);
    real_T *internals = ssGetOutputPortRealSignal(S, 3);
    real_T *draw = ssGetOutputPortRealSignal(S, 4);
    real_T *derived = ssGetOutputPortRealSignal(S, 5);

    for (int i = 0; i < 3; ++i) {
        force[i] = fm->force[i];
        moment[i] = fm->moment[i];
    }
    mass[0] = fm->mass;
    mass[1] = fm->inertia[0];
    mass[2] = fm->inertia[1];
    mass[3] = fm->inertia[2];
    for (int i = 0; i < kInternalCount; ++i) internals[i] = fm->internals[i];
    for (int i = 0; i < kDrawArgCount; ++i) draw[i] = fm->draw[i];
    for (int i = 0; i < 8; ++i) derived[i] = fm->derived[i];
}

// The flight model runs here, once per step, on the state at the start of the step. Its forces are output from the
// next step on: a one-step delay (0.006 s), which also keeps the model free of algebraic loops.
#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T)
{
    Instance *fm = instance_of(S);
    if (!fm) return;
    const double dt = mxGetScalar(ssGetSFcnParam(S, kParamStep));
    // inputs are contiguous (ssSetInputPortRequiredContiguous), so they are read as plain arrays
    const real_T *pilot = (const real_T *)ssGetInputPortSignal(S, 0);
    const real_T *state = (const real_T *)ssGetInputPortSignal(S, 1);
    const real_T *air = (const real_T *)ssGetInputPortSignal(S, 2);
    step_flight_model(*fm, pilot, state, air, dt);
}

static void mdlTerminate(SimStruct *S)
{
    Instance *fm = instance_of(S);
    if (!fm) return;
    unload_library(*fm);
    delete fm;
    ssGetPWork(S)[0] = nullptr;
}

#ifdef MATLAB_MEX_FILE
extern "C" {
#include "simulink.c"
}
#else
#include "cg_sfun.h"
#endif

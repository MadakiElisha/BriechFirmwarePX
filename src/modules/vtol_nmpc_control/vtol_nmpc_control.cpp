#include "vtol_nmpc_control.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_config.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

using namespace time_literals;

VtolNmpcControl::VtolNmpcControl() :
    ModuleParams(nullptr),
    px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
    memset(_state_u_vec, 0, sizeof(_state_u_vec));
    memset(_reference_vec, 0, sizeof(_reference_vec));
    memset(_worker_w, 0, sizeof(_worker_w));

    // Initialize controls: hover thrust, zero moments
    _state_u_vec[13] = 0.5;   // Collective thrust
    _state_u_vec[14] = 0.0;   // Roll moment
    _state_u_vec[15] = 0.0;   // Pitch moment
    _state_u_vec[16] = 0.0;   // Yaw moment
}

VtolNmpcControl::~VtolNmpcControl() {
    ScheduleClear();
}

bool VtolNmpcControl::init()
{
    ScheduleOnInterval(50000_us);  // 20 Hz
    PX4_INFO("NMPC Controller initialized (20 Hz)");
    return true;
}

void VtolNmpcControl::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    parameters_update();

    vehicle_local_position_s  pos;
    vehicle_attitude_s         att;
    vehicle_angular_velocity_s ang_vel;

    if (!_local_pos_sub.copy(&pos) ||
        !_att_sub.copy(&att)       ||
        !_angular_vel_sub.copy(&ang_vel)) {
        return;  // No valid state yet
    }

    // ── 1. State vector ──────────────────────────────────────────────────────
    _state_u_vec[0]  = pos.x;
    _state_u_vec[1]  = pos.y;
    _state_u_vec[2]  = pos.z;
    _state_u_vec[3]  = pos.vx;
    _state_u_vec[4]  = pos.vy;
    _state_u_vec[5]  = pos.vz;
    _state_u_vec[6]  = att.q[0];         // qw
    _state_u_vec[7]  = att.q[1];         // qx
    _state_u_vec[8]  = att.q[2];         // qy
    _state_u_vec[9]  = att.q[3];         // qz
    _state_u_vec[10] = ang_vel.xyz[0];   // p  (roll  rate)
    _state_u_vec[11] = ang_vel.xyz[1];   // q  (pitch rate)
    _state_u_vec[12] = ang_vel.xyz[2];   // r  (yaw   rate)

    // ── 2. Reference ─────────────────────────────────────────────────────────
    update_reference();

    // ── 3. NMPC optimisation ─────────────────────────────────────────────────
    const uint64_t t_start = hrt_absolute_time();
    optimize();
    const uint64_t solver_time = hrt_absolute_time() - t_start;

    // ── 4. Motor mixing & publish ─────────────────────────────────────────────
    // _state_u_vec[13] = collective thrust  T   (0 → 1)
    // _state_u_vec[14] = roll  moment       Mx  (-1 → 1)
    // _state_u_vec[15] = pitch moment       My  (-1 → 1)
    // _state_u_vec[16] = yaw  moment        Mz  (-1 → 1)
    //
    // X-configuration quadrotor:
    //
    //        front
    //    M1 (CCW)  M2 (CW)
    //       \      /
    //        \    /
    //    M3 (CW)  M4 (CCW)
    //        back
    //
    //  M1 = T + Mx + My - Mz   (front-left)
    //  M2 = T - Mx + My + Mz   (front-right)
    //  M3 = T + Mx - My + Mz   (rear-left)
    //  M4 = T - Mx - My - Mz   (rear-right)

    const float T  = (float)_state_u_vec[13];
    const float Mx = (float)_state_u_vec[14];
    const float My = (float)_state_u_vec[15];
    const float Mz = (float)_state_u_vec[16];

    actuator_motors_s out{};
    out.timestamp        = hrt_absolute_time();
    out.timestamp_sample = out.timestamp;

    out.control[0] = math::constrain(T + Mx + My - Mz, 0.0f, 1.0f);  // M1 front-left
    out.control[1] = math::constrain(T - Mx + My + Mz, 0.0f, 1.0f);  // M2 front-right
    out.control[2] = math::constrain(T + Mx - My + Mz, 0.0f, 1.0f);  // M3 rear-left
    out.control[3] = math::constrain(T - Mx - My - Mz, 0.0f, 1.0f);  // M4 rear-right

    // Safety: if any NaN slip through, cut motors
    for (int i = 0; i < 4; i++) {
        if (!PX4_ISFINITE(out.control[i])) out.control[i] = 0.0f;
    }

    out.reversible_flags = 0;
    _actuator_motors_pub.publish(out);

    // ── 5. Periodic log ───────────────────────────────────────────────────────
    if (_loop_counter % 20 == 0) {
        PX4_INFO("NMPC pos=[%.2f,%.2f,%.2f] ref=[%.2f,%.2f,%.2f] "
                 "u=[T:%.3f Mx:%.3f My:%.3f Mz:%.3f] "
                 "M=[%.3f,%.3f,%.3f,%.3f] dt=%llu us",
                 (double)_state_u_vec[0],  (double)_state_u_vec[1],  (double)_state_u_vec[2],
                 (double)_reference_vec[0],(double)_reference_vec[1],(double)_reference_vec[2],
                 (double)T, (double)Mx, (double)My, (double)Mz,
                 (double)out.control[0], (double)out.control[1],
                 (double)out.control[2], (double)out.control[3],
                 (unsigned long long)solver_time);
    }
    _loop_counter++;
}

// ─────────────────────────────────────────────────────────────────────────────
void VtolNmpcControl::update_reference()
{
    trajectory_setpoint_s sp;

    if (_setpoint_sub.copy(&sp)) {

        _reference_vec[0] = PX4_ISFINITE(sp.position[0]) ? (double)sp.position[0] : _state_u_vec[0];
        _reference_vec[1] = PX4_ISFINITE(sp.position[1]) ? (double)sp.position[1] : _state_u_vec[1];
        _reference_vec[2] = PX4_ISFINITE(sp.position[2]) ? (double)sp.position[2] : _state_u_vec[2];

        _reference_vec[3] = PX4_ISFINITE(sp.velocity[0]) ? (double)sp.velocity[0] : 0.0;
        _reference_vec[4] = PX4_ISFINITE(sp.velocity[1]) ? (double)sp.velocity[1] : 0.0;
        _reference_vec[5] = PX4_ISFINITE(sp.velocity[2]) ? (double)sp.velocity[2] : 0.0;

        if (PX4_ISFINITE(sp.yaw)) {
            const float cy = cosf(sp.yaw * 0.5f);
            const float sy = sinf(sp.yaw * 0.5f);
            _reference_vec[6] = (double)cy;
            _reference_vec[7] = 0.0;
            _reference_vec[8] = 0.0;
            _reference_vec[9] = (double)sy;
        } else {
            _reference_vec[6] = _state_u_vec[6];
            _reference_vec[7] = _state_u_vec[7];
            _reference_vec[8] = _state_u_vec[8];
            _reference_vec[9] = _state_u_vec[9];
        }

        _reference_vec[10] = 0.0;
        _reference_vec[11] = 0.0;
        _reference_vec[12] = PX4_ISFINITE(sp.yawspeed) ? (double)sp.yawspeed : 0.0;

    } else {
        // No setpoint → hold current state
        for (int i = 0; i < 13; i++) _reference_vec[i] = _state_u_vec[i];
        _reference_vec[3] = _reference_vec[4] = _reference_vec[5]  = 0.0;
        _reference_vec[10]= _reference_vec[11]= _reference_vec[12] = 0.0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// NMPC optimise:
//   Step A – call the CasADi cost function once to get current cost.
//   Step B – numerical gradient  ∂J/∂u  via finite differences.
//   Step C – direct proportional correction from state error   (fast path).
//   Step D – gradient-descent refinement with momentum         (NMPC path).
//   Both paths are blended so the drone responds immediately AND the
//   CasADi solver keeps pulling the solution toward optimality.
// ─────────────────────────────────────────────────────────────────────────────
void VtolNmpcControl::optimize()
{
    // ── Tuning knobs ─────────────────────────────────────────────────────────
    constexpr double KP_Z   =  0.35;   // Altitude proportional gain
    constexpr double KD_Z   =  0.15;   // Altitude derivative  gain
    constexpr double KP_XY  =  0.12;   // Horizontal position  gain
    constexpr double KD_XY  =  0.06;   // Horizontal velocity  gain
    constexpr double ALPHA   =  0.25;   // Gradient descent step
    constexpr double MOMENTUM=  0.85;
    constexpr double EPS     =  1e-3;   // Finite-difference step
    constexpr int    N_ITER  =  12;     // GD iterations per cycle
    constexpr double BLEND   =  0.4;    // 0=pure PD, 1=pure GD

    // ── Errors (NED: z negative = altitude) ──────────────────────────────────
    const double ez  = _reference_vec[2] - _state_u_vec[2];   // +  = need to climb
    const double evz = _reference_vec[5] - _state_u_vec[5];   // desired – actual vz
    const double ex  = _reference_vec[0] - _state_u_vec[0];
    const double ey  = _reference_vec[1] - _state_u_vec[1];
    const double evx = _reference_vec[3] - _state_u_vec[3];
    const double evy = _reference_vec[4] - _state_u_vec[4];

    // ── Step C: Direct PD control law ────────────────────────────────────────
    // Altitude: climb when ez < 0  (ref is more negative → higher)
    const double thrust_pd = 0.5 - KP_Z * ez - KD_Z * evz;

    // Horizontal: tilt toward target (small angle assumption)
    // In NED body frame: positive pitch = nose down = +x acceleration
    const double pitch_pd  =  KP_XY * ex + KD_XY * evx;   // fwd error  → nose down
    const double roll_pd   = -KP_XY * ey - KD_XY * evy;   // right error → roll right

    // ── Step A: CasADi cost evaluation ───────────────────────────────────────
    casadi_real current_cost  = 0.0;
    casadi_real perturbed_cost= 0.0;
    casadi_real grad[4]       = {0.0, 0.0, 0.0, 0.0};

    const casadi_real* arg[2] = {_state_u_vec, _reference_vec};
    casadi_real* res[1]       = {&current_cost};
    solver_fun(arg, res, nullptr, _worker_w, 0);

    // ── Step B: Numerical gradient ∂J/∂u ─────────────────────────────────────
    for (int i = 0; i < 4; i++) {
        const int     idx  = 13 + i;
        const casadi_real saved = _state_u_vec[idx];
        _state_u_vec[idx] += EPS;

        casadi_real* rp[1] = {&perturbed_cost};
        solver_fun(arg, rp, nullptr, _worker_w, 0);

        grad[i]           = (perturbed_cost - current_cost) / EPS;
        _state_u_vec[idx] = saved;
    }

    // ── Step D: Gradient-descent with momentum ────────────────────────────────
    for (int k = 0; k < N_ITER; k++) {
        for (int i = 0; i < 4; i++) {
            _velocity[i] = MOMENTUM * _velocity[i] - ALPHA * grad[i];
        }
    }

    // Desired from GD step
    const double thrust_gd = _state_u_vec[13] + _velocity[0];
    const double roll_gd   = _state_u_vec[14] + _velocity[1];
    const double pitch_gd  = _state_u_vec[15] + _velocity[2];
    const double yaw_gd    = _state_u_vec[16] + _velocity[3];

    // ── Blend PD (immediate response) + GD (optimal) ─────────────────────────
    const double thrust_cmd = (1.0 - BLEND) * thrust_pd + BLEND * thrust_gd;
    const double roll_cmd   = (1.0 - BLEND) * roll_pd   + BLEND * roll_gd;
    const double pitch_cmd  = (1.0 - BLEND) * pitch_pd  + BLEND * pitch_gd;
    const double yaw_cmd    = yaw_gd;   // yaw only from GD (no PD yaw yet)

    // ── Smooth update (low-pass, avoid step changes) ──────────────────────────
    constexpr double LP = 0.35;   // Low-pass coefficient
    _state_u_vec[13] += LP * (thrust_cmd - _state_u_vec[13]);
    _state_u_vec[14] += LP * (roll_cmd   - _state_u_vec[14]);
    _state_u_vec[15] += LP * (pitch_cmd  - _state_u_vec[15]);
    _state_u_vec[16] += LP * (yaw_cmd    - _state_u_vec[16]);

    // ── Hard bounds ───────────────────────────────────────────────────────────
    _state_u_vec[13] = fmax(0.05, fmin(0.95, _state_u_vec[13]));  // thrust 5-95%
    _state_u_vec[14] = fmax(-0.3, fmin(0.3,  _state_u_vec[14]));  // roll  ±30%
    _state_u_vec[15] = fmax(-0.3, fmin(0.3,  _state_u_vec[15]));  // pitch ±30%
    _state_u_vec[16] = fmax(-0.2, fmin(0.2,  _state_u_vec[16]));  // yaw   ±20%
}

// ─────────────────────────────────────────────────────────────────────────────
void VtolNmpcControl::parameters_update() { updateParams(); }

int VtolNmpcControl::task_spawn(int argc, char *argv[]) {
    VtolNmpcControl *instance = new VtolNmpcControl();
    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
        if (instance->init()) return 0;
    }
    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return -1;
}

int VtolNmpcControl::print_usage(const char *reason) {
    if (reason) PX4_WARN("%s\n", reason);
    PRINT_MODULE_DESCRIPTION("VTOL NMPC Control");
    PRINT_MODULE_USAGE_NAME("vtol_nmpc_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

int VtolNmpcControl::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command");
}
// blah blah
extern "C" __EXPORT int vtol_nmpc_control_main(int argc, char *argv[]) {
    return VtolNmpcControl::main(argc, argv);
}




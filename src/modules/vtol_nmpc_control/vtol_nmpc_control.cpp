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

    // Initialize controls to hover estimate
    _state_u_vec[13] = 0.5;  // Throttle
    _state_u_vec[14] = 0.0;  // Roll moment
    _state_u_vec[15] = 0.0;  // Pitch moment
    _state_u_vec[16] = 0.0;  // Yaw moment
}

VtolNmpcControl::~VtolNmpcControl() {
    ScheduleClear();
}

bool VtolNmpcControl::init()
{
    // Run at 20 Hz (50ms)
    ScheduleOnInterval(50000_us);
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

    vehicle_local_position_s pos;
    vehicle_attitude_s att;
    vehicle_angular_velocity_s ang_vel;

    // Read state from EKF2
    if (_local_pos_sub.copy(&pos) && _att_sub.copy(&att) && _angular_vel_sub.copy(&ang_vel)) {

        // Update state vector (13 elements)
        _state_u_vec[0] = pos.x;
        _state_u_vec[1] = pos.y;
        _state_u_vec[2] = pos.z;
        _state_u_vec[3] = pos.vx;
        _state_u_vec[4] = pos.vy;
        _state_u_vec[5] = pos.vz;
        _state_u_vec[6] = att.q[0];  // qw
        _state_u_vec[7] = att.q[1];  // qx
        _state_u_vec[8] = att.q[2];  // qy
        _state_u_vec[9] = att.q[3];  // qz
        _state_u_vec[10] = ang_vel.xyz[0];  // p
        _state_u_vec[11] = ang_vel.xyz[1];  // q
        _state_u_vec[12] = ang_vel.xyz[2];  // r

        // Update reference from setpoint
        update_reference();

        // Solve NMPC
        const uint64_t t_start = hrt_absolute_time();
        optimize();
        const uint64_t solver_time = hrt_absolute_time() - t_start;

        // Publish actuator commands
        actuator_motors_s out{};
        out.timestamp = hrt_absolute_time();

        for (int i = 0; i < 4; i++) {
            float val = (float)_state_u_vec[13 + i];
            out.control[i] = PX4_ISFINITE(val) ? math::constrain(val, 0.0f, 1.0f) : 0.0f;
        }

        _actuator_motors_pub.publish(out);

        // Periodic logging (every 1 second)
        if (_loop_counter % 20 == 0) {
		PX4_INFO("NMPC: pos=[%.2f,%.2f,%.2f] ref=[%.2f,%.2f,%.2f] u=[%.3f,%.3f,%.3f,%.3f] dt=%llu us",
			(double)_state_u_vec[0], (double)_state_u_vec[1], (double)_state_u_vec[2],
			(double)_reference_vec[0], (double)_reference_vec[1], (double)_reference_vec[2],
			(double)_state_u_vec[13], (double)_state_u_vec[14],
			(double)_state_u_vec[15], (double)_state_u_vec[16],
			(unsigned long long)solver_time);  // Add cast here
	}
        _loop_counter++;
    }
}

void VtolNmpcControl::update_reference()
{
    trajectory_setpoint_s sp;

    if (_setpoint_sub.copy(&sp)) {
        // Position reference (cast float to double)
        _reference_vec[0] = PX4_ISFINITE(sp.position[0]) ? (double)sp.position[0] : _state_u_vec[0];
        _reference_vec[1] = PX4_ISFINITE(sp.position[1]) ? (double)sp.position[1] : _state_u_vec[1];
        _reference_vec[2] = PX4_ISFINITE(sp.position[2]) ? (double)sp.position[2] : _state_u_vec[2];

        // Velocity reference
        _reference_vec[3] = PX4_ISFINITE(sp.velocity[0]) ? (double)sp.velocity[0] : 0.0;
        _reference_vec[4] = PX4_ISFINITE(sp.velocity[1]) ? (double)sp.velocity[1] : 0.0;
        _reference_vec[5] = PX4_ISFINITE(sp.velocity[2]) ? (double)sp.velocity[2] : 0.0;

        // Attitude reference (convert yaw to quaternion, assume level)
        if (PX4_ISFINITE(sp.yaw)) {
            float cos_yaw = cosf(sp.yaw * 0.5f);
            float sin_yaw = sinf(sp.yaw * 0.5f);
            _reference_vec[6] = (double)cos_yaw;   // qw
            _reference_vec[7] = 0.0;               // qx
            _reference_vec[8] = 0.0;               // qy
            _reference_vec[9] = (double)sin_yaw;   // qz
        } else {
            // Use current attitude as reference
            _reference_vec[6] = _state_u_vec[6];
            _reference_vec[7] = _state_u_vec[7];
            _reference_vec[8] = _state_u_vec[8];
            _reference_vec[9] = _state_u_vec[9];
        }

        // Angular rate reference (typically zero)
        _reference_vec[10] = 0.0;
        _reference_vec[11] = 0.0;
        _reference_vec[12] = PX4_ISFINITE(sp.yawspeed) ? (double)sp.yawspeed : 0.0;
    } else {
        // No setpoint - hold current position
        for (int i = 0; i < 13; i++) {
            _reference_vec[i] = _state_u_vec[i];
        }
        // Zero velocity/rates
        _reference_vec[3] = 0.0;
        _reference_vec[4] = 0.0;
        _reference_vec[5] = 0.0;
        _reference_vec[10] = 0.0;
        _reference_vec[11] = 0.0;
        _reference_vec[12] = 0.0;
    }
}

void VtolNmpcControl::optimize()
{
    const int max_qp_iters = 10;
    const casadi_real alpha = 0.1;     // Learning rate
    const casadi_real eps = 1e-4;       // Gradient epsilon
    const casadi_real momentum = 0.9;   // Momentum factor

    casadi_real current_cost = 0;
    casadi_real perturbed_cost = 0;
    casadi_real grad[4] = {0};

    // Solver expects: arg[0] = z[17], arg[1] = x_ref[13]
    const casadi_real* arg[2] = {_state_u_vec, _reference_vec};  // FIXED!
    casadi_real* res[1] = {&current_cost};

    // Evaluate current cost
    solver_fun(arg, res, nullptr, _worker_w, 0);

    // Compute gradient via finite differences
    for (int i = 0; i < 4; i++) {
        int idx = 13 + i;
        casadi_real original_u = _state_u_vec[idx];
        _state_u_vec[idx] += eps;

        casadi_real* res_perturbed[1] = {&perturbed_cost};
        solver_fun(arg, res_perturbed, nullptr, _worker_w, 0);

        grad[i] = (perturbed_cost - current_cost) / eps;
        _state_u_vec[idx] = original_u;
    }

    // Gradient descent with momentum
    for (int qp_i = 0; qp_i < max_qp_iters; qp_i++) {
        for (int i = 0; i < 4; i++) {
            int idx = 13 + i;
            _velocity[i] = (momentum * _velocity[i]) - (alpha * grad[i]);
            _state_u_vec[idx] += _velocity[i];

            // Apply bounds [0, 1]
            if (_state_u_vec[idx] > 1.0) {
                _state_u_vec[idx] = 1.0;
                _velocity[i] = 0;
            }
            if (_state_u_vec[idx] < 0.0) {
                _state_u_vec[idx] = 0.0;
                _velocity[i] = 0;
            }
        }
    }
}

void VtolNmpcControl::parameters_update()
{
    updateParams();
}

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
    if (reason) {
        PX4_WARN("%s\n", reason);
    }
    PRINT_MODULE_DESCRIPTION("VTOL NMPC Control");
    PRINT_MODULE_USAGE_NAME("vtol_nmpc_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

int VtolNmpcControl::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command");
}

extern "C" __EXPORT int vtol_nmpc_control_main(int argc, char *argv[]) {
    return VtolNmpcControl::main(argc, argv);
}

#include "vtol_nmpc_control.hpp"
#include <lib/mathlib/mathlib.h>
#include <cmath>

extern "C" {
#include <acados_c/ocp_nlp_interface.h>
}

VtolNmpcControl::VtolNmpcControl() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

VtolNmpcControl::~VtolNmpcControl()
{
    if (_capsule) {
        quadplane_nmpc_acados_free(_capsule);
        quadplane_nmpc_acados_free_capsule(_capsule);
    }
}

bool VtolNmpcControl::init()
{
    _capsule = quadplane_nmpc_acados_create_capsule();
    int status = quadplane_nmpc_acados_create(_capsule);

    if (status != 0) {
        PX4_ERR("NMPC Solver Creation Failed: %d", status);
        return false;
    }

    float dt = _param_nmpc_dt.get();
    if (dt <= 0.0f) { dt = 0.05f; }

    ScheduleOnInterval(static_cast<uint32_t>(dt * 1e6f));
    PX4_INFO("NMPC Controller initialized (dt=%.3f s)", (double)dt);
    return true;
}

void VtolNmpcControl::Run()
{
    if (should_exit()) { ScheduleClear(); exit_and_cleanup(); return; }

    if (_parameter_update_sub.updated()) {
        parameter_update_s pupdate;
        _parameter_update_sub.copy(&pupdate);
        updateParams();
    }

    vehicle_control_mode_s mode{};
    _control_mode_sub.copy(&mode);

    vtol_vehicle_status_s v_status{};
    _vtol_status_sub.copy(&v_status);

    vehicle_local_position_s pos{};
    if (!mode.flag_armed || !_local_pos_sub.copy(&pos)) {
        _z_integral = 0.0f;
        _last_run   = hrt_absolute_time();
        return;
    }

    if (!_param_use_nmpc.get()) { return; }

    // --- 1. Reference from trajectory_setpoint ---
    trajectory_setpoint_s sp{};
    float target_vz = 0.0f;

    if (_trajectory_setpoint_sub.copy(&sp)) {
        const float dt = _param_nmpc_dt.get();

        // Save raw GCS destination for distance/pusher/transition decisions
        if (PX4_ISFINITE(sp.position[0])) { _x_dest = (double)sp.position[0]; }
        if (PX4_ISFINITE(sp.position[1])) { _y_dest = (double)sp.position[1]; }

        // Replacement 1: Parameterized Lateral Slew
        const double slew = (double)_param_nmpc_ref_slew.get();
        if (PX4_ISFINITE(sp.position[0])) {
            _x_ref += math::constrain(
                _x_dest - _x_ref,
                -slew * (double)dt, slew * (double)dt);
        }
        if (PX4_ISFINITE(sp.position[1])) {
            _y_ref += math::constrain(
                _y_dest - _y_ref,
                -slew * (double)dt, slew * (double)dt);
        }

        // Replacement 2: Parameterized Vertical Slew
        if (PX4_ISFINITE(sp.position[2])) {
            const float vsp  = _param_nmpc_max_vsp.get();
            float z_diff     = sp.position[2] - (float)_z_ref;
            float step       = math::constrain(z_diff, -vsp * dt, vsp * dt);
            _z_ref          += (double)step;
            target_vz        = step / dt;
        }
    }

    // --- 2. Distance & Replacement 3 (Part A): Parameterized Max Speed ---
    const float dx           = (float)(_x_dest - (double)pos.x);
    const float dy           = (float)(_y_dest - (double)pos.y);
    const float horiz_dist   = sqrtf(dx * dx + dy * dy);

    const float max_spd         = _param_nmpc_max_spd.get();
    const float desired_fwd_spd = math::constrain(horiz_dist * 0.5f, 0.0f, max_spd);

    // --- 3. Altitude integrator ---
    const hrt_abstime now = hrt_absolute_time();
    if (_last_run > 0) {
        const float dt_sec = static_cast<float>(now - _last_run) * 1e-6f;
        _z_integral += 0.5f * ((float)_z_ref - pos.z) * dt_sec;
        _z_integral  = math::constrain(_z_integral, -1.0f, 1.0f);
    }
    _last_run = now;

    // --- 4. NMPC initial state ---
    double x0[7] = {
        (double)pos.x,  (double)pos.y,  (double)pos.z,
        (double)pos.vx, (double)pos.vy, (double)pos.vz,
        (double)_z_integral
    };

    // Casting field names to (char *) to satisfy C++ compiler safety
// CORRECT — nlp_out restored as 4th arg per your acados signature
    ocp_nlp_constraints_model_set(_capsule->nlp_config, _capsule->nlp_dims,
                               _capsule->nlp_in, _capsule->nlp_out,
                               0, "lbx", x0);
    ocp_nlp_constraints_model_set(_capsule->nlp_config, _capsule->nlp_dims,
                               _capsule->nlp_in, _capsule->nlp_out,
                               0, "ubx", x0);

    // --- 5. Horizon references ---
    const double dyn_hover = math::constrain(
        0.37 + ((double)_z_integral * 0.25), 0.10, 0.65);

    double y_ref[11] = {
        _x_ref, _y_ref, _z_ref,
        0.0,    0.0,    (double)target_vz,
        0.0,
        dyn_hover, dyn_hover, dyn_hover, dyn_hover
    };

    for (int i = 0; i <= _capsule->nlp_dims->N; i++) {
        ocp_nlp_cost_model_set(_capsule->nlp_config, _capsule->nlp_dims,
                               _capsule->nlp_in, i, (char *)"yref", (void *)y_ref);
    }

    // --- 6. Solve ---
    const uint64_t t_start     = hrt_absolute_time();
    int            solve_status = quadplane_nmpc_acados_solve(_capsule);
    const uint32_t solver_time  = (uint32_t)(hrt_absolute_time() - t_start);

    double u_plan[4] = {dyn_hover, dyn_hover, dyn_hover, dyn_hover};
    if (solve_status == 0) { // ACADOS_SUCCESS
        ocp_nlp_out_get(_capsule->nlp_config, _capsule->nlp_dims,
                        _capsule->nlp_out, 0, (char *)"u", (void *)u_plan);
    }

    // --- 7. VTOL state ---
    const bool currently_hover = (v_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
    const bool currently_fw    = (v_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
    const bool in_transition   = (v_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_FW ||
                                  v_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_MC);

    // --- 8. Transition commands ---
    if (horiz_dist > 20.0f && currently_hover && _loop_counter % 200 == 0) {
        vehicle_command_s vcmd{};
        vcmd.timestamp        = hrt_absolute_time();
        vcmd.command          = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
        vcmd.param1           = 4.0f; // fixed-wing
        vcmd.target_system    = 1;
        vcmd.target_component = 1;
        _vehicle_command_pub.publish(vcmd);
        PX4_INFO("NMPC: → FW transition (dest_dist=%.1fm)", (double)horiz_dist);
    }

    if (horiz_dist < 10.0f && currently_fw && _loop_counter % 200 == 0) {
        vehicle_command_s vcmd{};
        vcmd.timestamp        = hrt_absolute_time();
        vcmd.command          = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
        vcmd.param1           = 3.0f; // multicopter
        vcmd.target_system    = 1;
        vcmd.target_component = 1;
        _vehicle_command_pub.publish(vcmd);
        PX4_INFO("NMPC: → MC transition (dest_dist=%.1fm)", (double)horiz_dist);
    }

    // --- 9. Actuator output & Replacement 3 (Part B): Normalized Pusher Logic ---
    actuator_motors_s motors{};
    motors.timestamp        = hrt_absolute_time();
    motors.timestamp_sample = motors.timestamp;

    for (int i = 0; i < 4; i++) {
        motors.control[i] = currently_fw ? 0.0f :
            math::constrain(static_cast<float>(u_plan[i]), 0.0f, 1.0f);
    }

    float pusher_throttle = 0.0f;
    if (currently_fw) {
        pusher_throttle = math::constrain(desired_fwd_spd / max_spd, 0.3f, 0.9f);
    } else if (in_transition) {
        pusher_throttle = math::constrain(desired_fwd_spd / (max_spd * 1.33f), 0.1f, 0.7f);
    } else if (horiz_dist > 5.0f) {
        pusher_throttle = math::constrain(desired_fwd_spd / (max_spd * 2.0f), 0.0f, 0.3f);
    }
    motors.control[4] = pusher_throttle;

    _actuator_motors_pub.publish(motors);

    // --- 10. Logging ---
    if (_loop_counter % 20 == 0) {
        const double u_avg = (u_plan[0]+u_plan[1]+u_plan[2]+u_plan[3]) / 4.0;
        PX4_INFO("NMPC [%d] %uus | %s | DestDist: %.1fm",
                 solve_status, (unsigned int)solver_time,
                 currently_fw ? "FW" : (in_transition ? "TRANSITION" : "HOVER"),
                 (double)horiz_dist);
        PX4_INFO("POS:[%6.1f,%6.1f,%6.2f] DEST:[%6.1f,%6.1f] REF_Z:%6.2f",
                 (double)pos.x, (double)pos.y, (double)pos.z,
                 _x_dest, _y_dest, _z_ref);
        PX4_INFO("DynHov:%.2f LiftAvg:%.2f Pusher:%.2f Int:%.3f",
                 dyn_hover, u_avg, (double)pusher_throttle, (double)_z_integral);

        if (solve_status != 0) {
            PX4_WARN("Solver Warning: Status %d", solve_status);
        }
    }
    _loop_counter++;
}

int VtolNmpcControl::task_spawn(int argc, char *argv[])
{
    VtolNmpcControl *instance = new VtolNmpcControl();
    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
        if (instance->init()) { return PX4_OK; }
    }
    PX4_ERR("Failed to start vtol_nmpc_control");
    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
}

int VtolNmpcControl::custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

int VtolNmpcControl::print_usage(const char *reason)
{
    if (reason) { PX4_WARN("%s\n", reason); }
    PRINT_MODULE_DESCRIPTION("NMPC for VTOL Quadplane using Acados.");
    PRINT_MODULE_USAGE_NAME("vtol_nmpc_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int vtol_nmpc_control_main(int argc, char *argv[])
{
    return VtolNmpcControl::main(argc, argv);
}

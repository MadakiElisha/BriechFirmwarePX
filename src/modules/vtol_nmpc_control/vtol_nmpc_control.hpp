#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>

extern "C" {
#ifdef UNUSED
#undef UNUSED
#endif
#include "acados_solver_quadplane_nmpc.h"
}

class VtolNmpcControl : public ModuleBase<VtolNmpcControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    VtolNmpcControl();
    ~VtolNmpcControl() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;

    quadplane_nmpc_solver_capsule *_capsule{nullptr};

    uORB::Subscription         _local_pos_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription         _attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription         _vtol_status_sub{ORB_ID(vtol_vehicle_status)};
    uORB::Subscription         _control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 100000};
    uORB::Subscription         _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};

    uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
    uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

    float       _z_integral{0.0f};
    hrt_abstime _last_run{0};
    uint32_t    _loop_counter{0};

    // Rate-limited NMPC solver reference (follows drone smoothly)
    double      _x_ref{0.0};
    double      _y_ref{0.0};
    double      _z_ref{-10.0};

    // Raw GCS destination — used for horiz_dist, pusher, and transition logic
    double      _x_dest{0.0};
    double      _y_dest{0.0};

    DEFINE_PARAMETERS(
        (ParamBool<px4::params::USE_NMPC>)      _param_use_nmpc,
        (ParamInt<px4::params::NMPC_N_STEPS>)   _param_nmpc_n_steps,
        (ParamFloat<px4::params::NMPC_DT>)      _param_nmpc_dt,
        (ParamFloat<px4::params::NMPC_MAX_SPD>) _param_nmpc_max_spd,
        (ParamFloat<px4::params::NMPC_REF_SLEW>)_param_nmpc_ref_slew,
        (ParamFloat<px4::params::NMPC_MAX_VSP>) _param_nmpc_max_vsp
    )
};

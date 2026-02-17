#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

// uORB Topics
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/actuator_motors.h>

#include "nmpc_solver.h"

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
	void optimize();
	void parameters_update();
	void update_reference();  // NEW: Get reference from setpoint

	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_vel_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _setpoint_sub{ORB_ID(trajectory_setpoint)};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	casadi_real _state_u_vec[17];     // [states(13); controls(4)]
	casadi_real _reference_vec[13];   // NEW: Reference trajectory
	casadi_real _worker_w[66];        // Updated size: 64->66

	casadi_real _velocity[4]{0.0, 0.0, 0.0, 0.0};

	uint32_t _loop_counter{0};  // For periodic logging

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p
	)
};

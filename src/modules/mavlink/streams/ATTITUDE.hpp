/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef ATTITUDE_HPP
#define ATTITUDE_HPP

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

class MavlinkStreamAttitude : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAttitude(mavlink); }

	static constexpr const char *get_name_static() { return "ATTITUDE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ATTITUDE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _att_sub.advertised() ? MAVLINK_MSG_ID_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamAttitude(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	bool send() override
	{
		if (_mavlink->_crypt->state() != crypt_state::ESTABLISHED || _mavlink == nullptr || _mavlink->_crypt == nullptr) {
			return false;
		}

		vehicle_attitude_s att;

		if (_att_sub.update(&att)) {
			// 1. Prepare inner message
			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			mavlink_attitude_t inner_msg{};

			const matrix::Eulerf euler = matrix::Quatf(att.q);
			inner_msg.time_boot_ms = att.timestamp / 1000;
			inner_msg.roll = euler.phi();
			inner_msg.pitch = euler.theta();
			inner_msg.yaw = euler.psi();

			inner_msg.rollspeed = angular_velocity.xyz[0];
			inner_msg.pitchspeed = angular_velocity.xyz[1];
			inner_msg.yawspeed = angular_velocity.xyz[2];

			// mavlink_msg_attitude_send_struct(_mavlink->get_channel(), &inner_msg);


			// 2. Place inner message into a buffer
			uint8_t payload_buffer[sizeof(mavlink_attitude_t)];
			memcpy(payload_buffer, &inner_msg, sizeof(inner_msg));

			// 3. Prepare the wrapper
			mavlink_obfuscated_data_t wrapper_msg{};
			wrapper_msg.len = sizeof(payload_buffer);

			// 4. Encryption
			int crypt_ret = _mavlink->_crypt->encrypt_msg(
				payload_buffer,
				sizeof(payload_buffer),
				wrapper_msg.nonce,
				wrapper_msg.tag,
				wrapper_msg.data
			);

			if (crypt_ret == 0) {
				mavlink_msg_obfuscated_data_send_struct(_mavlink->get_channel(), &wrapper_msg);
				return true;
			} else {
				PX4_ERR("Encryption failed, packet dropped.");
				return false;
			}
		}


		return false;
	}
};

#endif // ATTITUDE_HPP

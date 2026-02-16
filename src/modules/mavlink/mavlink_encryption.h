#pragma once

class MavlinkCrypt
{
	public:
		MavlinkCrypt();

		int encrypt_msg();
		int decrypt_msg();
};

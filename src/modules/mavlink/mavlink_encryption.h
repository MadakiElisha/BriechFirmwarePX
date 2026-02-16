#pragma once

// Including the mbedTLS crypto lib
#include <mbedtls/gcm.h>
#include <px4_platform_common/log.h>

class MavlinkCrypt
{
	public:
		MavlinkCrypt();

		int encrypt_msg();
		int decrypt_msg();
};

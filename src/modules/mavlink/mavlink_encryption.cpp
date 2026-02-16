#include "mavlink_encryption.h"
#include <stdlib.h>
#include <time.h>

MavlinkCrypt::MavlinkCrypt() :
	_encryption_count(0),
	_decrypted_count(0),
	_auth_failures(0)
{
	memset(&_aes_ctx, 0, sizeof(_aes_ctx));
	memset(_encryption_key, 0, sizeof(_encryption_key));

	PX4_INFO("MavLinkCrypt constructed");
}

MavlinkCrypt::~MavlinkCrypt()
{
	mbedtls_gcm_free(&_aes_ctx);
	memset(_encryption_key, 0, AES_KEY_SIZE);
}

int MavlinkCrypt::encrypt_msg(
	const uint8_t *plaintext,
	size_t plaintext_len,
	encrypted_message_s *output)
{
	return 1;
}

int MavlinkCrypt::decrypt_msg(
	const encrypted_message_s *encrypted,
	uint8_t *plaintext,
	size_t *plaintext_len)
{
	return 1;
}

#include "mavlink_main.h"
#include "mavlink_encryption.h"

MavlinkCrypt::MavlinkCrypt(Mavlink *parent) :
	_state(crypt_state::UNINITIALIZED),
	_mavlink(parent)
{

	_state = crypt_state::INITIALIZING;
	// PX4_INFO("MavLinkCrypt constructed");
}

MavlinkCrypt::~MavlinkCrypt()
{
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

void MavlinkCrypt::initiate_handshake()
{
	if(_generate_key_pair()) PX4_INFO("[DBugger] Generated keypair no prob");
	_send_public_key();
	return;
}

bool MavlinkCrypt::_generate_key_pair()
{
    size_t olen = 0;
    mbedtls_ecdh_init(&_ctx);
    mbedtls_entropy_init(&_entropy);
    mbedtls_ctr_drbg_init(&_ctr_drbg);


    int ret = 0;

    // 1. Seed the RNG
    hrt_abstime now = hrt_absolute_time();
    mbedtls_ctr_drbg_update(&_ctr_drbg, (const unsigned char *)&now, sizeof(now));

    // 2. Select curve
    ret = mbedtls_ecdh_setup(&_ctx, MBEDTLS_ECP_DP_CURVE25519);
    if (ret != 0) {
        PX4_ERR("ECDH Setup failed: -0x%04x", -ret);
        return false;
    }

    // 3. Generate key pair
    ret = mbedtls_ecdh_make_public(&_ctx, &olen, _public_key, sizeof(_public_key),
                                   mbedtls_ctr_drbg_random, &_ctr_drbg);

    if (ret != 0) {
        PX4_ERR("Make Public failed: -0x%04x", -ret);
        return false;
    }

    PX4_INFO("Key generated successfully, length: %zu", olen);
    return true;
}

bool MavlinkCrypt::_send_public_key(){
	if (_mavlink) {
		mavlink_msg_key_exchange_data_send(
			_mavlink->get_channel(),
			255,
			190,
			_public_key
		);

		PX4_INFO("[DBugger] Should have sent public key");
		print_key(_public_key, 32);
    	}
	return 1;
}

void MavlinkCrypt::print_key(uint8_t* key, size_t len) {
    printf("Key: ");
    for (size_t i = 0; i < len; i++) {
        // %02x prints hex with 2 digits and a leading zero if needed
        printf("%02x", key[i]);
    }
    printf("\n");
}

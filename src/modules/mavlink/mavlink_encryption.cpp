#include "mavlink_main.h"
#include "mavlink_encryption.h"

MavlinkCrypt::MavlinkCrypt(Mavlink *parent) :
	_state(crypt_state::UNINITIALIZED),
	_mavlink(parent)
{
	PX4_INFO("[Debug] MavLinkCrypt Initializing");
    PX4_INFO("[Debug] Changing state to INITIALIZED");
    if(_generate_key_pair()) _state = crypt_state::INITIALIZED;
    else PX4_INFO("[Debug] Failed to initialize");
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
    PX4_INFO("[Debug] Changing state to HANDSHAKING");
    _state = crypt_state::HANDSHAKING;
    _send_public_key();
	return;
}

bool MavlinkCrypt::_generate_key_pair()
{
    psa_status_t status;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    // psa_key_id_t key_id;


    status = psa_crypto_init();
    if (status != PSA_SUCCESS) {
        PX4_INFO("Failed to initialize PSA Crypto: %d\n", status);
        return false;
    }

    // 2. Set Key Attributes
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_DERIVE);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECDH);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_MONTGOMERY));
    psa_set_key_bits(&attributes, 256);

    // 3. Generate the Key
    PX4_WARN("[Debug] Using temporary STATIC KEY!!");

    // PX4_INFO("Generating key pair...\n");
    // status = psa_generate_key(&attributes, &key_id);
    // if (status != PSA_SUCCESS) {
    //     PX4_INFO("Key generation failed: %d\n", status);
    //     return false;
    // }
    print_key(_public_key, 32);


    // 4. Export the Public Key
    // uint8_t public_key[32];
    // size_t public_key_len;

    // status = psa_export_public_key(key_id, public_key, sizeof(public_key), &public_key_len);

    // if (status == PSA_SUCCESS) {
        // print_key(_public_key, 32);
    // } else {
    //     PX4_INFO("Failed to export public key: %d\n", status);
    // }




    // 5. Clean up
    // psa_destroy_key(key_id);
    // psa_reset_key_attributes(&attributes);

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

		PX4_INFO("[Debug] Should have sent public key");
		print_key(_public_key, 32);
    	}
    else{
        PX4_ERR("[Debug] MAVLink not initialized");
    }
	return 1;
}

void MavlinkCrypt::print_key(uint8_t* key, size_t len) {
    printf("Key: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x", key[i]);
    }
    printf("\n");
    return;
}

void MavlinkCrypt::recv_public_key(){
    PX4_INFO("[Debug] Received public key from GCS");

    PX4_WARN("[Debug] Using static key!!!");
    print_key(_shared_secret, 256);
}


void MavlinkCrypt::finalize_handshake(){
    PX4_INFO("[Debug] Just passing the check for testing");

    PX4_INFO("[Debug] Changing state to READY");
    _state = crypt_state::READY;
}

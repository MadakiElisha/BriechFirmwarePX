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

void MavlinkCrypt::decrypt_payload(
			uint8_t *payload,
			uint16_t len
		)
{
    crypto_chacha20_ctr(payload, payload, len, _shared_key, _temp_nonce, 0);
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
    PX4_INFO("[Debug] Generating keypair!!");

    _random_num_gen(_secret_key, 32);

    PX4_INFO("[Debug] Secret Key");
    print_key(_secret_key, 32);

    // Compute public key
    crypto_x25519_public_key(_public_key, _secret_key);
    PX4_INFO("[Debug] Public Key");
    print_key(_public_key, 32);

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

void MavlinkCrypt::print_key(uint8_t* key,
                             size_t len) {
    printf("Key: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x", key[i]);
    }
    printf("\n");
    return;
}

void MavlinkCrypt::recv_public_key(uint8_t public_key[32]){
    PX4_INFO("[Debug] Received public key from GCS");
    print_key(public_key, 32);

    PX4_INFO("[Debug] Generating shared key");
    crypto_x25519(_shared_key, _secret_key, public_key);

    print_key(_shared_key, 32);

    finalize_handshake();

}

void MavlinkCrypt::finalize_handshake(){
    PX4_INFO("[Debug] Just passing the check for testing");

    PX4_INFO("[Debug] Changing state to READY");
    _state = crypt_state::READY;
}

bool MavlinkCrypt::_random_num_gen(uint8_t* buffer, uint8_t size){
    int fd = open("/dev/urandom", O_RDONLY);

    if(fd>=0){
        ssize_t bytes_read = read(fd, buffer, size);

        close(fd);

        return true;

        if(bytes_read != size){
            PX4_ERR("[Debug] Error in random number generator");
            return false;
        }
    }else {
        PX4_ERR("[Debug] /dev/urandom is not available");
        return false;
    }

    return false;
}

#include "mavlink_main.h"
#include "mavlink_encryption.h"

MavlinkCrypt::MavlinkCrypt(Mavlink *parent) :
	_state(crypt_state::IDLE),
	_mavlink(parent)
{
	PX4_INFO("[Debug] MavLinkCrypt Initializing");

    if(!_generate_key_pair()) PX4_ERR("[Debug] Failed to initialize");
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
    // crypto_chacha20_ctr(payload, payload, len, _shared_key, _temp_nonce, 0);
}

void MavlinkCrypt::initiate_handshake(uint8_t public_key[32], uint8_t nonce[32])
{
    memcpy(_recvd_public_key, public_key, 32);
    memcpy(_recvd_nonce, nonce, 32);

    PX4_INFO("Received public key");
    print_key(_recvd_public_key, 32);

    PX4_INFO("Received Nonce");
    print_key(_recvd_nonce, 32);

    _send_public_key();
    _state = crypt_state::WAIT_FINAL;
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
		// mavlink_msg_key_exchange_data_send(
		// 	_mavlink->get_channel(),
		// 	255,
		// 	190,
		// 	_public_key
		// );
        mavlink_msg_secure_handshake_send(
            _mavlink->get_channel(),
            _mavlink->get_system_id(),
            _recvd_nonce,
            1,
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

// void MavlinkCrypt::recv_public_key(uint8_t public_key[32]){
//     PX4_INFO("[Debug] Received public key from GCS");
//     print_key(public_key, 32);

//     PX4_INFO("[Debug] Generating shared key");
//     crypto_x25519(_shared_key, _secret_key, public_key);

//     print_key(_shared_key, 32);

//     finalize_handshake();
// }


bool MavlinkCrypt::_random_num_gen(uint8_t* buffer, uint8_t size){
    int fd = open("/dev/urandom", O_RDONLY);

    if(fd<0){
        PX4_ERR("Failed to open /dev/urandom");
        return false;
    }

    // Read until buffer is full or there is an error
    size_t total_read = 0;
    while (total_read < size) {
        ssize_t n = read(fd, buffer + total_read, size - total_read);
        if (n <= 0) break;
        total_read += n;
    }

    close(fd);

    if (total_read != size) {
        PX4_ERR("Insufficient entropy: read %zu of %u", total_read, size);
        return false;
    }

    return true;
}

void MavlinkCrypt::print_key(uint8_t* key,
                             size_t len)
{
    printf("Key: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x", key[i]);
    }
    printf("\n");
    return;
}

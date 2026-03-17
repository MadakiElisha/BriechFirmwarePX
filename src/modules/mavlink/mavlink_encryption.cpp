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


void MavlinkCrypt::initiate_handshake(uint8_t public_key[32], uint8_t nonce[24])
{
    memcpy(_recvd_public_key, public_key, 32);
    memcpy(_recvd_nonce, nonce, 24);

    PX4_INFO("Received public key");
    print_key(_recvd_public_key, 32);

    PX4_INFO("Received Nonce");
    print_key(_recvd_nonce, 24);

    // Obtain shared key and create session key
    crypto_x25519(_shared_key, _secret_key, _recvd_public_key);

    PX4_INFO("Generated Shared Key");
    print_key(_shared_key, 32);

    // Create session key
    crypto_blake2b_ctx ctx;
    crypto_blake2b_init(&ctx);

    crypto_blake2b_update(&ctx, _shared_key, 32);
    crypto_blake2b_update(&ctx, _recvd_nonce, 24);
    crypto_blake2b_update(&ctx, (const uint8_t*)_mission_password, strlen(_mission_password));

    uint8_t full_hash[64];
    crypto_blake2b_final(&ctx, full_hash);
    memcpy(_session_key, full_hash, 32);

    PX4_INFO("Generated Session Key");
    print_key(_session_key, 32);

    crypto_wipe(_shared_key, 32);
    crypto_wipe(_secret_key, 32);


    // This has to wait until the above is done to ensure that the sequence follows a defined order and prevents issues
    _send_public_key();
    _state = crypt_state::WAIT_FINAL;
}


void MavlinkCrypt::verify_handshake(uint8_t pass_key[32], uint8_t nonce[24], uint8_t mac[16])
{

    uint8_t plain_key[32] = {};

    // Decrypt the pass key

    // PX4_INFO("[Debug] Encrypted Pass Key");
    // print_key(pass_key, 32);
    // PX4_INFO("[Debug] Decryption Nonce");
    // print_key(nonce, 24);
    if (crypto_unlock_aead(plain_key, _session_key, nonce, mac, NULL, 0, pass_key, 32) == 0) {
        PX4_INFO("[Crypto] Decrypted verification key");
    } else {
        // Integrity check FAILED.
        // The data was corrupted or the key/nonce is incorrect.
        // DO NOT trust the 'plain' buffer at this point; wipe it.
        PX4_ERR("[Crypto] Decrypting key failed");
        return;
    }

    // Re-encrypt the pass key
    uint8_t encrypted_pass_key[32] = {0};
    uint8_t new_mac[16] = {0};
    uint8_t new_nonce[24] = {0};
    _random_num_gen(new_nonce, 24);

    crypto_lock_aead(new_mac, encrypted_pass_key, _session_key, new_nonce, NULL, 0, plain_key, 32);

    // PX4_INFO("[Debug] Re-encrypted Pass Key");
    // print_key(encrypted_pass_key, 32);
    // PX4_INFO("[Debug] Re-encryption Nonce");
    // print_key(new_nonce, 16);


    // Send the reencrypted pass
    mavlink_msg_secure_handshake_send(
        _mavlink->get_channel(),
        255,
        new_nonce,
        2, // State: Verification Response
        encrypted_pass_key,
        new_mac
    );

    crypto_wipe(plain_key, 32);
    crypto_wipe(new_nonce, 32);

    PX4_INFO("Handshake Verification Sent!");
    _state = crypt_state::ESTABLISHED;
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


bool MavlinkCrypt::_send_public_key()
{
	if (_mavlink) {
		// mavlink_msg_key_exchange_data_send(
		// 	_mavlink->get_channel(),
		// 	255,
		// 	190,
		// 	_public_key
		// );
        uint8_t zero_tag[16] = {0};
        uint8_t zero_nonce[24] = {0};

        mavlink_msg_secure_handshake_send(
            _mavlink->get_channel(),
            _mavlink->get_system_id(),
            zero_nonce,
            1,
            _public_key,
            zero_tag
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


bool MavlinkCrypt::_random_num_gen(uint8_t* buffer, uint8_t size)
{
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

void MavlinkCrypt::print_key(uint8_t* key, size_t len)
{
    printf("Key: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02x", key[i]);
    }
    printf("\n");
    return;
}

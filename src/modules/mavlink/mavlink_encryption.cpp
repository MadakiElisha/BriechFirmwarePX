#include "mavlink_main.h"
#include "mavlink_encryption.h"

MavlinkCrypt::MavlinkCrypt(Mavlink *parent) :
	_state(crypt_state::IDLE),
	_mavlink(parent)
{
	PX4_INFO("[Debug] MavLinkCrypt Initialized");

}


MavlinkCrypt::~MavlinkCrypt()
{
    // GET RID OF ALL OF THE ENCRYPTION VARIABLES AND PARAMETERS
    crypto_wipe(_secret_key, 32);
    crypto_wipe(_public_key, 32);
    crypto_wipe(_shared_key, 32);
    crypto_wipe(_session_key, 32);
}


int MavlinkCrypt::encrypt_msg(const uint8_t *plaintext, size_t plaintext_len, uint8_t *nonce, uint8_t *tag, uint8_t *output)
{
	if (_state != crypt_state::ESTABLISHED) {
        // PX4_ERR("Cannot encrypt message: Handshake not established");
        return -1;
    }

    _encr_counter += 2;
    uint8_t full_nonce[24] = {0};
    memcpy(full_nonce, &_encr_counter, 8);
    memcpy(nonce, &_encr_counter, 8);

    crypto_aead_lock(output, tag, _session_key, full_nonce, NULL, 0, plaintext, plaintext_len);

    return 0;
}


int MavlinkCrypt::decrypt_msg(const encrypted_message_s *encrypted, uint8_t *plaintext, size_t *plaintext_len)
{
	return 1;
}


bool MavlinkCrypt::decrypt_payload(uint8_t *payload, uint8_t *cipher, size_t len, uint8_t *nonce, uint8_t *tag)
{
    uint64_t recv_counter;
    memcpy(&recv_counter, nonce, 8);

    // Prevent replay attacks
    if(recv_counter <= _recv_counter){
        // PX4_ERR("Replay attack detected: received counter %lu is not greater than last counter %lu", recv_counter, _recv_counter);
        return false;
    }

    uint8_t full_nonce[24] = {0};
    memcpy(full_nonce, nonce, 8);

    if (crypto_aead_unlock(payload, tag, _session_key, full_nonce, NULL, 0, cipher, len) == 0) {
        _recv_counter = recv_counter;
        return true;
    }

    // Integrity check failed! Someone tampered with the message or the key is wrong.
    return false;
}


void MavlinkCrypt::initiate_handshake(uint8_t public_key[32], uint8_t nonce[24])
{
    _state = crypt_state::IDLE;

    if(!_generate_key_pair()) {
        PX4_ERR("[Debug] Failed to initialize");
    }

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
    crypto_blake2b_init(&ctx, 32);

    crypto_blake2b_update(&ctx, _shared_key, 32);
    crypto_blake2b_update(&ctx, _recvd_nonce, 24);
    crypto_blake2b_update(&ctx, (const uint8_t*)_mission_password, strlen(_mission_password));

    uint8_t full_hash[32];
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

    // Start the counter at the recvd gcs counter value
    memcpy(&_encr_counter, nonce, 8);

    // Decrypt the pass key
    PX4_INFO("\n[Debug] Encrypted Pass Key");
    print_key(pass_key, 32);
    PX4_INFO("\n[Debug] Decryption Nonce");
    // Pad the nonce for ChaCha20
    uint8_t full_nonce[24] = {0};
    memcpy(full_nonce, nonce, 8);
    print_key(nonce, 24);

    if (crypto_aead_unlock(plain_key, mac, _session_key, full_nonce, NULL, 0, pass_key, 32) == 0) {
        PX4_INFO("[Crypto] Decrypted verification key");
    } else {
        // Integrity check FAILED.
        // The data was corrupted or the key/nonce is incorrect.
        PX4_ERR("[Crypto] Decrypting key failed");
        crypto_wipe(plain_key, 32);
        return;
    }

    // Re-encrypt the pass key
    _encr_counter++; // ensure that the counter starts offset from the gcs counter. From now on, only add 2
    uint8_t encrypted_pass_key[32] = {0};
    uint8_t new_mac[16] = {0};
    uint8_t new_nonce[24] = {0};
    uint8_t new_counter[8] = {0};
    memcpy(new_nonce, &_encr_counter, 8);
    memcpy(new_counter, &_encr_counter, 8);

    crypto_aead_lock(encrypted_pass_key, new_mac, _session_key, new_nonce, NULL, 0, plain_key, 32);

    PX4_INFO("\n[Debug] Re-encrypted Pass Key");
    print_key(encrypted_pass_key, 32);
    PX4_INFO("\n[Debug] Re-encryption Nonce");
    print_key(new_nonce, 8);


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

    PX4_INFO("\nHandshake Verification Sent!");
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

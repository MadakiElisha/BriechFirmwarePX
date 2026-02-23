#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

// Including the mbedTLS crypto lib
#include <mbedtls/gcm.h>


// To gain access to PX4_INFO and the like
#include <px4_platform_common/log.h>

// Encryption constants
#define AES_KEY_SIZE 32        // 256 bits
#define AES_IV_SIZE 12         // 96 bits (GCM standard)
#define AES_TAG_SIZE 16        // 128 bits (authentication tag)
#define MAX_MAVLINK_SIZE 280   // MAVLink v2 max size

/**
 * Encrypted message structure
 */
struct encrypted_message_s {
    uint64_t timestamp;
    uint8_t  iv[AES_IV_SIZE];
    uint8_t  ciphertext[MAX_MAVLINK_SIZE];
    uint16_t ciphertext_len;
    uint8_t  tag[AES_TAG_SIZE];
};

enum class crypt_state{
	UNINITIALIZED,
	INITIALIZED,
	HANDHAKING,
	READY
};

class Mavlink; // Creating a class that will be used to create the null pointer which will point to the mavlink class on startup


class MavlinkCrypt
{
	public:
		MavlinkCrypt(Mavlink *parent);
		~MavlinkCrypt();

		int encrypt_msg(
			const uint8_t *plaintext,
			size_t plaintext_len,
			encrypted_message_s *output
		);

		int decrypt_msg(
			const encrypted_message_s *encrypted,
			uint8_t *plaintext,
			size_t *plaintext_len
		);

		void initiate_handshake();

		crypt_state state(){
			return _state;
		}

	private:
		mbedtls_gcm_context _aes_ctx;
		uint8_t _encryption_key[AES_KEY_SIZE];
		uint32_t _encryption_count;
		uint32_t _decrypted_count;
		uint32_t _auth_failures;

		crypt_state _state;
		Mavlink *_mavlink{nullptr};
		// orb_advert_t _handshake_pub = nullptr;

		// void _load_encryption_key();
		// void _generate_random_iv(uint8_t *iv);

};

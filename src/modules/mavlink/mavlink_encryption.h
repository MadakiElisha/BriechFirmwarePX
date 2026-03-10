#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

// Including the monocypher crypto lib
#include <fcntl.h>
#include <unistd.h>
#include <monocypher.h>

// To gain access to PX4_INFO and the like
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <mavlink/encr_dialect/mavlink.h>

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

enum class crypt_state : uint8_t{
	UNINITIALIZED,
	INITIALIZED,
	HANDSHAKING,
	VERIFYING,
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

		void decrypt_payload(
			uint8_t *payload,
			uint16_t len
		);

		void initiate_handshake();
		void recv_public_key(uint8_t public_key[32]);
		void finalize_handshake();

		crypt_state state(){
			return _state;
		}



	private:
		unsigned char _secret_key[32];
		unsigned char _public_key[32];

		unsigned char _shared_key[32];
		unsigned char _temp_nonce[8] = {0};

		crypt_state _state;
		Mavlink *_mavlink{nullptr};


		bool _generate_key_pair();
		bool _send_public_key();
		bool _random_num_gen(uint8_t* buffer, uint8_t size);
		void print_key(uint8_t* key, size_t len);

};

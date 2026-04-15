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
	IDLE,
	WAIT_FINAL,
	ESTABLISHED,
	DISCONNECTED,
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
			uint8_t *nonce,
			uint8_t *len,
			uint8_t *output
		);

		int decrypt_msg(
			const encrypted_message_s *encrypted,
			uint8_t *plaintext,
			size_t *plaintext_len
		);

		bool decrypt_payload(
			uint8_t *payload,
			uint8_t *cipher,
			size_t len,
			uint8_t *nonce,
			uint8_t *tag
		);

		void initiate_handshake(uint8_t public_key[32], uint8_t nonce[24]);

		void verify_handshake(uint8_t pass_key[32], uint8_t nonce[24], uint8_t mac[16]);

		crypt_state state(){
			return _state;
		}



	private:
		unsigned char _secret_key[32];
		unsigned char _public_key[32];

		unsigned char _recvd_public_key[32];
		unsigned char _recvd_nonce[24];

		unsigned char _shared_key[32];
		unsigned char _nonce[8] = {0};

		unsigned char _session_key[32];

		uint64_t message_counter;

		const char* _mission_password = "oscar-oscar-papa-sierra";

		crypt_state _state;
		Mavlink *_mavlink{nullptr};

		bool _generate_key_pair();
		bool _send_public_key();
		bool _random_num_gen(uint8_t* buffer, uint8_t size);
		void print_key(uint8_t* key, size_t len);

};

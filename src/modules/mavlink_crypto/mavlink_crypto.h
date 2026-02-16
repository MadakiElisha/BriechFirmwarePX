/****************************************************************************
 *
 * mavlink_crypto.h
 *
 * AES-256-GCM encryption for MAVLink messages
 * Briech UAS
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>
#include <string.h>

// mbedTLS crypto library
#include <mbedtls/gcm.h>

// PX4 platform includes
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

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

/**
 * MAVLink encryption module
 */
class MavlinkCrypto : public ModuleBase<MavlinkCrypto>
{
public:
    MavlinkCrypto();
    ~MavlinkCrypto() override;

    //added in an attempt to fix the instantiate error
    static MavlinkCrypto *instantiate(int argc, char *argv[]);
    //added in an attempt to fix the instantiate error

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    //Added in an attempt to solve the init problem when compiling the whole thing
    bool init ();
    //Added this in an attempt to solve the init problem when compiling the whole thing

    void run() override;

    int encrypt_message(const uint8_t *plaintext,
                       size_t plaintext_len,
                       encrypted_message_s *output);

    int decrypt_message(const encrypted_message_s *encrypted,
                       uint8_t *plaintext,
                       size_t *plaintext_len);

private:
    mbedtls_gcm_context _aes_ctx;
    uint8_t _encryption_key[AES_KEY_SIZE];
    uint32_t _encrypted_count;
    uint32_t _decrypted_count;
    uint32_t _auth_failures;

    void load_encryption_key();
    void generate_random_iv(uint8_t *iv);
};

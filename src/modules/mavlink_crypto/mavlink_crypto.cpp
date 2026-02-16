/****************************************************************************
 *
 * mavlink_crypto.cpp
 *
 * AES-256-GCM encryption implementation
 * Briech UAS
 *
 ****************************************************************************/

#include "mavlink_crypto.h"
#include <stdlib.h>
#include <time.h>

MavlinkCrypto::MavlinkCrypto() :
    ModuleBase(),
    _encrypted_count(0),
    _decrypted_count(0),
    _auth_failures(0)
{
    memset(&_aes_ctx, 0, sizeof(_aes_ctx));
    memset(_encryption_key, 0, sizeof(_encryption_key));
}

MavlinkCrypto::~MavlinkCrypto()
{
    mbedtls_gcm_free(&_aes_ctx);
    memset(_encryption_key, 0, AES_KEY_SIZE);
}

bool MavlinkCrypto::init()
{
    mbedtls_gcm_init(&_aes_ctx);
    load_encryption_key();

    int ret = mbedtls_gcm_setkey(&_aes_ctx, MBEDTLS_CIPHER_ID_AES,
                                 _encryption_key, AES_KEY_SIZE * 8);

    if (ret != 0) {
        PX4_ERR("AES key setup failed: %d", ret);
        return false;
    }

    PX4_INFO("AES-256-GCM initialized");
    srand(time(NULL) ^ hrt_absolute_time());

    return true;
}

MavlinkCrypto *MavlinkCrypto::instantiate(int argc, char *argv[])
{
    MavlinkCrypto *instance = new MavlinkCrypto();

    if (instance) {
        if (!instance->init()) {
            PX4_ERR("Failed to initialize MavlinkCrypto");
            delete instance;
            instance = nullptr;
        }
    }

    return instance;
}

void MavlinkCrypto::load_encryption_key()
{
    const uint8_t key[AES_KEY_SIZE] = {
        0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe,
        0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
        0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7,
        0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4
    };
    memcpy(_encryption_key, key, AES_KEY_SIZE);
    PX4_WARN("Using TEST key - replace in production");
}

void MavlinkCrypto::generate_random_iv(uint8_t *iv)
{
    for (int i = 0; i < AES_IV_SIZE; i++) {
        iv[i] = rand() & 0xFF;
    }
}

int MavlinkCrypto::encrypt_message(const uint8_t *plaintext,
                                   size_t plaintext_len,
                                   encrypted_message_s *output)
{
    if (!plaintext || !output || plaintext_len == 0 ||
        plaintext_len > MAX_MAVLINK_SIZE) {
        return -1;
    }

    generate_random_iv(output->iv);
    int ret = mbedtls_gcm_crypt_and_tag(&_aes_ctx, MBEDTLS_GCM_ENCRYPT,
                                        plaintext_len, output->iv, AES_IV_SIZE,
                                        NULL, 0, plaintext, output->ciphertext,
                                        AES_TAG_SIZE, output->tag);

    if (ret != 0) {
        PX4_ERR("Encryption failed: %d", ret);
        return -1;
    }

    output->timestamp = hrt_absolute_time();
    output->ciphertext_len = plaintext_len;
    _encrypted_count++;
    return 0;
}

int MavlinkCrypto::decrypt_message(const encrypted_message_s *encrypted,
                                   uint8_t *plaintext, size_t *plaintext_len)
{
    if (!encrypted || !plaintext || !plaintext_len) {
        return -1;
    }

    if (encrypted->ciphertext_len > MAX_MAVLINK_SIZE) {
        return -1;
    }

    int ret = mbedtls_gcm_auth_decrypt(&_aes_ctx, encrypted->ciphertext_len,
                                       encrypted->iv, AES_IV_SIZE, NULL, 0,
                                       encrypted->tag, AES_TAG_SIZE,
                                       encrypted->ciphertext, plaintext);

    if (ret != 0) {
        PX4_ERR("Auth failed: %d", ret);
        _auth_failures++;
        return -1;
    }

    *plaintext_len = encrypted->ciphertext_len;
    _decrypted_count++;
    return 0;
}

void MavlinkCrypto::run()
{
    while (!should_exit()) {
        px4_usleep(100000);
    }
}

int MavlinkCrypto::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("mavlink_crypto",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2000,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return PX4_ERROR;
    }

    return PX4_OK;
}

int MavlinkCrypto::custom_command(int argc, char *argv[])
{
    if (argc >= 1 && !strcmp(argv[0], "test")) {
        MavlinkCrypto *instance = get_instance();

        if (!instance) {
            PX4_ERR("Module not running. Start it with 'mavlink_crypto start'");
            return -1;
        }

        PX4_INFO("========================================");
        PX4_INFO("  Encryption Test");
        PX4_INFO("========================================");

        const char *msg = "CLASSIFIED: 34.05N 118.24W";
        encrypted_message_s enc;

        if (instance->encrypt_message((uint8_t *)msg, strlen(msg), &enc) != 0) {
            PX4_ERR("Encryption FAILED");
            return -1;
        }
        PX4_INFO("Encryption OK");

        uint8_t dec[MAX_MAVLINK_SIZE];
        size_t dec_len;
        if (instance->decrypt_message(&enc, dec, &dec_len) != 0) {
            PX4_ERR("Decryption FAILED");
            return -1;
        }
        PX4_INFO("Decryption OK: %.*s", (int)dec_len, dec);

        enc.ciphertext[0] ^= 0xFF; // Tamper
        if (instance->decrypt_message(&enc, dec, &dec_len) != 0) {
            PX4_INFO("Tampering detected OK");
        } else {
            PX4_ERR("Tampering NOT detected");
            return -1;
        }

        PX4_INFO("========================================");
        PX4_INFO("  ALL TESTS PASSED");
        PX4_INFO("========================================");
        return 0;
    }

    return print_usage("unknown command");
}

int MavlinkCrypto::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### MAVLink Encryption
AES-256-GCM encryption module

### Usage
Start: mavlink_crypto start
Test:  mavlink_crypto test
Stop:  mavlink_crypto stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("mavlink_crypto", "communication");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Run encryption test");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int mavlink_crypto_main(int argc, char *argv[])
{
    return MavlinkCrypto::main(argc, argv);
}

/****************************************************************************
 *
 * crypto_test.cpp - Test AES-256-GCM encryption
 *
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

// Include our encryption module
#include <modules/mavlink_crypto/mavlink_crypto.h>

/**
 * Print data in hexadecimal
 */
static void print_hex(const char *label, const uint8_t *data, size_t len)
{
    printf("%s: ", label);
    for (size_t i = 0; i < len; i++) {
        printf("%02x", data[i]);
        if ((i + 1) % 16 == 0 && i < len - 1) {
            printf("\n    ");
        }
    }
    printf("\n");
}

/**
 * Main test function
 */
extern "C" __EXPORT int crypto_test_main(int argc, char *argv[])
{
    PX4_INFO("========================================");
    PX4_INFO("  AES-256-GCM Encryption Test");
    PX4_INFO("  Defense Firmware - Briech Systems");
    PX4_INFO("========================================\n");

    // Create encryption instance
    MavlinkCrypto crypto;

    // Test message (simulated classified data)
    const char *test_message = "CLASSIFIED: Target coordinates 34.0522N 118.2437W";
    size_t msg_len = strlen(test_message);

    PX4_INFO("Test 1: Basic Encryption/Decryption");
    PX4_INFO("------------------------------------");
    PX4_INFO("Original message: '%s'", test_message);
    PX4_INFO("Message length: %zu bytes\n", msg_len);

    // Encrypted message structure
    encrypted_message_s encrypted;

    // ENCRYPT
    PX4_INFO("Encrypting...");
    int ret = crypto.encrypt_message(
        (const uint8_t *)test_message,
        msg_len,
        &encrypted
    );

    if (ret != 0) {
        PX4_ERR("❌ Encryption FAILED!");
        return -1;
    }

    PX4_INFO("✅ Encryption successful!\n");
    
    // Display encrypted data
    print_hex("IV (12 bytes)", encrypted.iv, AES_IV_SIZE);
    print_hex("Ciphertext", encrypted.ciphertext, encrypted.ciphertext_len);
    print_hex("Auth Tag (16 bytes)", encrypted.tag, AES_TAG_SIZE);
    printf("\n");

    // DECRYPT
    PX4_INFO("Decrypting...");
    uint8_t decrypted[MAX_MAVLINK_SIZE];
    size_t decrypted_len;

    ret = crypto.decrypt_message(&encrypted, decrypted, &decrypted_len);

    if (ret != 0) {
        PX4_ERR("❌ Decryption FAILED!");
        return -1;
    }

    PX4_INFO("✅ Decryption successful!\n");
    PX4_INFO("Decrypted message: '%.*s'", (int)decrypted_len, decrypted);
    PX4_INFO("Decrypted length: %zu bytes\n", decrypted_len);

    // VERIFY
    if (decrypted_len == msg_len && 
        memcmp(decrypted, test_message, msg_len) == 0) {
        PX4_INFO("✅ PASS: Decrypted message matches original!\n");
    } else {
        PX4_ERR("❌ FAIL: Decrypted message does NOT match!");
        return -1;
    }

    // Test 2: Tampering detection
    PX4_INFO("\nTest 2: Tampering Detection");
    PX4_INFO("------------------------------------");
    PX4_INFO("Simulating data tampering...");

    encrypted_message_s tampered = encrypted;
    tampered.ciphertext[0] ^= 0xFF;  // Corrupt one byte

    PX4_INFO("Attempting to decrypt tampered message...");
    ret = crypto.decrypt_message(&tampered, decrypted, &decrypted_len);

    if (ret != 0) {
        PX4_INFO("✅ PASS: Tampering correctly detected!\n");
    } else {
        PX4_ERR("❌ FAIL: Tampering NOT detected! SECURITY BREACH!");
        return -1;
    }

    // Test 3: IV uniqueness
    PX4_INFO("\nTest 3: IV Uniqueness");
    PX4_INFO("------------------------------------");
    PX4_INFO("Encrypting same message twice...");

    encrypted_message_s encrypted2;
    ret = crypto.encrypt_message(
        (const uint8_t *)test_message,
        msg_len,
        &encrypted2
    );

    if (ret != 0) {
        PX4_ERR("❌ Second encryption failed!");
        return -1;
    }

    // Check IVs are different
    if (memcmp(encrypted.iv, encrypted2.iv, AES_IV_SIZE) != 0) {
        PX4_INFO("✅ PASS: Different IVs generated\n");
        print_hex("First IV ", encrypted.iv, AES_IV_SIZE);
        print_hex("Second IV", encrypted2.iv, AES_IV_SIZE);
        printf("\n");
    } else {
        PX4_ERR("❌ FAIL: Same IV reused! SECURITY RISK!");
        return -1;
    }

    // Check ciphertexts are different
    if (memcmp(encrypted.ciphertext, encrypted2.ciphertext, msg_len) != 0) {
        PX4_INFO("✅ PASS: Different ciphertexts produced\n");
    } else {
        PX4_ERR("❌ FAIL: Same ciphertext! IV not working!");
        return -1;
    }

    // Test 4: Performance test
    PX4_INFO("\nTest 4: Performance Test");
    PX4_INFO("------------------------------------");
    
    const int iterations = 100;
    uint64_t start_time = hrt_absolute_time();
    
    for (int i = 0; i < iterations; i++) {
        crypto.encrypt_message((const uint8_t *)test_message, msg_len, &encrypted);
    }
    
    uint64_t end_time = hrt_absolute_time();
    uint64_t total_us = end_time - start_time;
    float avg_us = (float)total_us / iterations;
    
    PX4_INFO("Encrypted %d messages in %llu microseconds", iterations, total_us);
    PX4_INFO("Average encryption time: %.2f microseconds (%.2f ms)", avg_us, avg_us / 1000.0f);
    PX4_INFO("Throughput: %.0f encryptions/second\n", 1000000.0f / avg_us);

    // Final summary
    PX4_INFO("\n========================================");
    PX4_INFO("  ALL TESTS PASSED! ✅");
    PX4_INFO("========================================");
    PX4_INFO("AES-256-GCM encryption is operational.");
    PX4_INFO("System ready for secure communications.");
    PX4_INFO("========================================\n");

    return 0;
}

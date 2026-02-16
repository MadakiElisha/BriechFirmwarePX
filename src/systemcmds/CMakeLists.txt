############################################################################
#
# Crypto Test
#
############################################################################

px4_add_module(
    MODULE systemcmds__crypto_test
    MAIN crypto_test
    SRCS
        crypto_test.cpp
    DEPENDS
        modules__mavlink_crypto
)

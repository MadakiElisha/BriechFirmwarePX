############################################################################
#
# MAVLink Crypto Module
#
############################################################################

px4_add_module(
    MODULE modules__mavlink_crypto
    MAIN mavlink_crypto
    SRCS
        mavlink_crypto.cpp
    DEPENDS
        mbedtls
)

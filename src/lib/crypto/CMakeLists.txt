# px4_add_git_submodule(TARGET git_mbedtls PATH "mbedtls")

# add_definitions(-D_GNU_SOURCE)

# add_subdirectory(mbedtls EXCLUDE_FROM_ALL)

# set(MBEDTLS_RELAXED_FLAGS
#     -Wno-bad-function-cast
#     -Wno-cast-align
#     -Wno-error=bad-function-cast
#     -Wno-error=implicit-function-declaration
# )

# set(MBED_TARGETS mbedtls mbedx509 mbedcrypto builtin)
# foreach(target ${MBED_TARGETS})
#     if(TARGET ${target})
#         target_compile_options(${target} PRIVATE ${MBEDTLS_RELAXED_FLAGS})
#     endif()
# endforeach()

# add_library(mbedtls_interface INTERFACE)

# target_link_libraries(mbedtls_interface
#     INTERFACE
#         mbedtls
#         mbedx509
#         mbedcrypto
#         tfpsacrypto
# )

# target_include_directories(mbedtls_interface
#     INTERFACE
#         ${CMAKE_CURRENT_SOURCE_DIR}/mbedtls/include
#         ${CMAKE_CURRENT_SOURCE_DIR}/mbedtls/tf-psa-crypto/include
# )


# 1. Register submodule
px4_add_git_submodule(TARGET git_monocypher PATH "monocypher")

# 2. Define the library
add_library(monocypher STATIC
    monocypher/src/monocypher.c
)
add_dependencies(monocypher git_monocypher)

# 3. Configure the lib
target_include_directories(monocypher PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/monocypher/src
)

# Avoid the warnings that usually trigger in PX4 builds
target_compile_options(monocypher PRIVATE
    -Wno-shadow
    -Wno-unused-parameter
    -Wno-missing-field-initializers
)

# 4. Create Monocypher interface
add_library(monocypher_interface INTERFACE)
target_link_libraries(monocypher_interface INTERFACE monocypher)
target_include_directories(monocypher_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/monocypher/src
)

# adds this directory to make linking to it easy
set(PX4_CRYPTO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/monocypher/src PARENT_SCOPE)

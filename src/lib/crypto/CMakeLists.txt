px4_add_git_submodule(TARGET git_mbedtls PATH "mbedtls")

add_definitions(-D_GNU_SOURCE)

add_subdirectory(mbedtls EXCLUDE_FROM_ALL)

set(MBEDTLS_RELAXED_FLAGS
    -Wno-bad-function-cast
    -Wno-cast-align
    -Wno-error=bad-function-cast
    -Wno-error=implicit-function-declaration
)

set(MBED_TARGETS mbedtls mbedx509 mbedcrypto builtin)
foreach(target ${MBED_TARGETS})
    if(TARGET ${target})
        target_compile_options(${target} PRIVATE ${MBEDTLS_RELAXED_FLAGS})
    endif()
endforeach()

add_library(mbedtls_interface INTERFACE)

target_link_libraries(mbedtls_interface
    INTERFACE
        mbedtls
        mbedx509
        mbedcrypto
        tfpsacrypto
)

target_include_directories(mbedtls_interface
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/mbedtls/include
        ${CMAKE_CURRENT_SOURCE_DIR}/mbedtls/tf-psa-crypto/include
)

# 1. Detect Architecture using PX4_PLATFORM (CMake variable, not a C preprocessor define)
#    PX4_PLATFORM = "posix" for SITL, "nuttx" for Pixhawk hardware
if("${PX4_PLATFORM}" STREQUAL "posix")
    set(ACADOS_INSTALL_DIR "/home/fnsbuas/acados/install")
    # SITL x86 build has shared libs only
    set(ACADOS_LIB_ACADOS  ${ACADOS_INSTALL_DIR}/lib/libacados.so)
    set(ACADOS_LIB_HPIPM   ${ACADOS_INSTALL_DIR}/lib/libhpipm.so)
    set(ACADOS_LIB_BLASFEO ${ACADOS_INSTALL_DIR}/lib/libblasfeo.so)
    message(STATUS "NMPC: Building for SITL (x86/posix) using ${ACADOS_INSTALL_DIR} [shared libs]")
else()
    set(ACADOS_INSTALL_DIR "/home/fnsbuas/acados_arm_install")
    # Pixhawk ARM build uses static libs for a self-contained binary
    set(ACADOS_LIB_ACADOS  ${ACADOS_INSTALL_DIR}/lib/libacados.a)
    set(ACADOS_LIB_HPIPM   ${ACADOS_INSTALL_DIR}/lib/libhpipm.a)
    set(ACADOS_LIB_BLASFEO ${ACADOS_INSTALL_DIR}/lib/libblasfeo.a)
    message(STATUS "NMPC: Building for Pixhawk (ARM/nuttx) using ${ACADOS_INSTALL_DIR} [static libs]")
endif()

# 2. Define the PX4 Module
px4_add_module(
    MODULE modules__vtol_nmpc_control
    MAIN vtol_nmpc_control
    STACK_MAIN 5000
    SRCS
        vtol_nmpc_control.cpp
        acados_solver_quadplane_nmpc.c
        quadplane_nmpc_model/quadplane_nmpc_expl_ode_fun.c
        quadplane_nmpc_model/quadplane_nmpc_expl_vde_forw.c
        quadplane_nmpc_model/quadplane_nmpc_expl_vde_adj.c
    MODULE_CONFIG
        module.yaml
    DEPENDS
        px4_work_queue
)

# 3. Add Include Paths
target_include_directories(modules__vtol_nmpc_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/quadplane_nmpc_model
    ${ACADOS_INSTALL_DIR}/include
    ${ACADOS_INSTALL_DIR}/include/acados
    ${ACADOS_INSTALL_DIR}/include/blasfeo/include
    ${ACADOS_INSTALL_DIR}/include/hpipm/include
)

# 4. Link Libraries (shared for SITL, static for ARM — set above)
target_link_libraries(modules__vtol_nmpc_control PRIVATE
    ${ACADOS_LIB_ACADOS}
    ${ACADOS_LIB_HPIPM}
    ${ACADOS_LIB_BLASFEO}
)

# 5. For SITL shared libs: bake the .so path into the binary rpath
#    so it runs without needing LD_LIBRARY_PATH to be set manually
if("${PX4_PLATFORM}" STREQUAL "posix")
    set_target_properties(modules__vtol_nmpc_control PROPERTIES
        BUILD_RPATH  "${ACADOS_INSTALL_DIR}/lib"
        INSTALL_RPATH "${ACADOS_INSTALL_DIR}/lib"
    )
endif()

# 6. Compiler Flags
target_compile_options(modules__vtol_nmpc_control PRIVATE
    -Wno-error
    -Wno-strict-prototypes
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-unused-but-set-variable
)

target_compile_definitions(modules__vtol_nmpc_control PRIVATE
    ACADOS_WITH_NO_UNUSED_MACRO
)

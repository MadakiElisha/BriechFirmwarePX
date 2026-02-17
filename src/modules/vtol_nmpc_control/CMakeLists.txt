px4_add_module(
        MODULE modules__vtol_nmpc_control
        MAIN vtol_nmpc_control
        STACK_MAIN 5000
        SRCS
                vtol_nmpc_control.cpp
                nmpc_solver.c
        MODULE_CONFIG
                module.yaml
        DEPENDS
                px4_work_queue
        )

px4_add_module(
    MODULE drivers__winch_payload
    MAIN winch_payload
    STACK_MAIN 2048
    SRCS
        winch_payload.cpp
    # Remove winch_params.c from here
    DEPENDS
        px4_work_queue
    MODULE_CONFIG
        module.yaml  # Add this line
)

file(GLOB CAN_SRCS *.cpp)
add_library(motors_protocol
        STATIC
        ${CAN_SRCS}
)
target_include_directories(motors_protocol PUBLIC ./can)
target_link_libraries(motors_protocol PUBLIC ${PUBLIC_DEPENDENCIES})

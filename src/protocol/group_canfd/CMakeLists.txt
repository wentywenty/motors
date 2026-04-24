file(GLOB DATA_SRCS *.cpp)
add_library(motors_group_canfd STATIC ${DATA_SRCS})
target_include_directories(motors_group_canfd PUBLIC 
    ./ 
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/protocol/canfd
)
target_link_libraries(motors_group_canfd PUBLIC ${PUBLIC_DEPENDENCIES} motors_canfd)
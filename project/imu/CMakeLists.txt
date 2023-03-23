add_executable(imu imu.hpp imu.cpp)
target_link_libraries(imu ${Boost_LIBRARIES} Utility NodeHandler mip "${SERIAL_LIB}")
target_compile_definitions(imu PUBLIC "MIP_USE_SERIAL")
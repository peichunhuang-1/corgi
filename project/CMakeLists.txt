include_directories(mathematic)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/odometry)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/calibration)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/legs)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/imu)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/cpg)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/force)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/foothold)


add_executable(calibration calibration.cpp)
target_link_libraries(calibration Eigen3::Eigen ${NLOPT_LIBRARIES})


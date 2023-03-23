add_executable(odometry odometry.hpp odometry.cpp)
target_link_libraries(odometry Eigen3::Eigen ${Boost_LIBRARIES} Utility NodeHandler)


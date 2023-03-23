add_executable(force force_optimizer.hpp force_optimizer.cpp)
target_link_libraries(force ${Boost_LIBRARIES} Utility NodeHandler Eigen3::Eigen ${NLOPT_LIBRARIES})
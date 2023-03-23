add_executable(foothold foothold.hpp foothold.cpp)
target_link_libraries(foothold Eigen3::Eigen ${Boost_LIBRARIES} Utility NodeHandler)


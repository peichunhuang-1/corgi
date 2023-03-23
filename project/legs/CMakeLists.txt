add_executable(legs legs.cpp)
target_link_libraries(legs Eigen3::Eigen ${Boost_LIBRARIES} Utility NodeHandler)


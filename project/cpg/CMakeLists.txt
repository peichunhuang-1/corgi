add_executable(cpg cpg.hpp cpg.cpp)
target_link_libraries(cpg ${Boost_LIBRARIES} Utility NodeHandler Eigen3::Eigen)
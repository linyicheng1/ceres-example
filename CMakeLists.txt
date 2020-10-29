PROJECT(ceres_example)
cmake_minimum_required(VERSION 2.8)

#在测试运行时间时打开
#set(BUILD_TYPE Release)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 添加c++ 11标准支持
set(CMAKE_CXX_STANDARD 11)

# 寻找Ceres库
find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
  
add_executable(ceres_example_1
    1_hello_world.cc
)
target_link_libraries(
    ceres_example_1
    ${CERES_LIBRARIES}
)

add_executable(ceres_example_2
    2_helloworld_numeric_diff.cc
)
target_link_libraries(
    ceres_example_2
    ${CERES_LIBRARIES}
)

add_executable(ceres_example_3
    3_helloworld_analytic_diff.cc
)
target_link_libraries(
    ceres_example_3
    ${CERES_LIBRARIES}
)

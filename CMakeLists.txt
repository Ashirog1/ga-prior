cmake_minimum_required(VERSION 3.0.0)
project(ga-priority VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

add_executable(ga-priority main.cpp src/ga.cpp src/def.cpp)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)

#cmake最低版本
cmake_minimum_required(VERSION 2.8)

#声明工程
project(Matrix_Sophus)

set(CMAKE_BUILD_TYPE “Debug”) 

#添加头文件
include_directories ("/usr/include/eigen3")
#include_directories ("/usr/local/include/sophus")

#为了使用sophus，需要使用find_package命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
set(Sophus_LIBRARIES libSophus.so))


#添加可执行文件
add_executable (useSophus useSophus.cpp)
target_link_libraries( useSophus ${Sophus_LIBRARIES})
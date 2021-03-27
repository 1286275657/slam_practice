#cmake最低版本
cmake_minimum_required(VERSION 2.8)

#声明工程
project(Matrix)

set(CMAKE_BUILD_TYPE “Debug”) 

#添加头文件
include_directories ("/usr/include/eigen3")

#添加可执行文件
add_executable (eigenMatrix eigenMatrix.cpp)

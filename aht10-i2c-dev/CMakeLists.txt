cmake_minimum_required(VERSION 3.17)
project(aht10 C)

set(CMAKE_C_COMPILER arm-linux-gcc)

set(CMAKE_C_STANDARD 11)

add_executable(aht10
        main.c)

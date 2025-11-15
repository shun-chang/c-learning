# 最低CMake版本要求
cmake_minimum_required(VERSION 3.10)

# 项目名称（可自定义）
project(BigintCalculator)

# 设置C++标准（至少C++11，根据需要调整）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找所有源文件（这里明确列出文件，也可使用通配符）
set(SOURCES
    bigint.cpp
    main.cpp
)

# 查找所有头文件（可选，用于IDE索引）
set(HEADERS
    bigint.hpp
)

# 添加可执行文件（指定输出的程序名称为bigint_calc）
add_executable(bigint_calc ${SOURCES} ${HEADERS})
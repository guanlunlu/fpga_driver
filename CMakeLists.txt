cmake_minimum_required(VERSION 3.10)

project(FPGA_SERVER C CXX ASM)
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "-std=c99")
set(INC_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

set(BOOST_COMPONENTS ALL)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread program_options)

include_directories(${INC_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})
include_directories(/usr/local/include/yaml-cpp)
include_directories(/usr/include/boost)
include_directories(${CURSES_INCLUDE_DIR})
include_directories(/usr/local/include/ncurses)
link_directories(/usr/local/lib)


add_executable(fpga_srv_main ${SRC_DIR}/fpga_server.cpp ${SRC_DIR}/leg_module.cpp ${SRC_DIR}/fpga_handler.cpp ${SRC_DIR}/color.cpp ${SRC_DIR}/fsm.cpp ${SRC_DIR}/console.cpp ${SRC_DIR}/NiFpga.c ${SRC_DIR}/NiFpga_FPGA_CANBus_IMU_4module_IRQ.c)
target_link_libraries(fpga_srv_main yaml-cpp ${CURSES_LIBRARY} -lncurses ${Boost_LIBRARIES} ${CMAKE_DL_LIBS} Utility NodeHandler -lpthread)

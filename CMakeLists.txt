# define the project name
project(LFAST_PADDLE_CONTROL C CXX)
cmake_minimum_required(VERSION 3.10)
set(CXX_STANDARD 17)

# CXX_STANDARD_REQUIRED
include(GNUInstallDirs)
find_package(PkgConfig)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --libs libmodbus")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )

add_compile_definitions(LFAST_TERMINAL)



# set(Boost_USE_MULTITHREAD ON)
# set(Boost_USE_STATIC_LIBS ON)
# set(Boost_USE_STATIC_RUNTIME OFF)
# set(Boost_DEBUG ON)
# find_package(Boost 1.60.0 REQUIRED COMPONENTS system)
# IF (Boost_FOUND)
#     message(STATUS "Boost version: ${Boost_VERSION}")
#     INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
#     LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
# endif()




list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/02_CMake_Modules/")
find_package(Threads REQUIRED)
find_package(Modbus REQUIRED)
find_package(CppLinuxSerial REQUIRED)

include_directories(${MODBUS_INCLUDE_DIRS} )

set(PROJ_LIBS "")
# add_library(bash_wrapper SHARED bash_wrapper.cc)
add_library(terminal_iface SHARED TerminalInterface.cc)
list(APPEND PROJ_LIBS terminal_iface)

add_library(kinco_iface_lib SHARED KincoDriver.cc)
target_link_libraries(kinco_iface_lib terminal_iface ${MODBUS_LIBRARIES} ${MODBUS_LIBRARY})
list(APPEND PROJ_LIBS kinco_iface_lib CppLinuxSerial::CppLinuxSerial)

add_executable(
    main
    main.cc
    PaddleController.cc
)



target_link_libraries(main ${PROJ_LIBS})

# add_executable(
#     main
#     main.cc
#     kincoDriver.cc
#     TerminalInterface.cc
# )
# target_link_libraries(main ${MODBUS_LIBRARIES} ${MODBUS_LIBRARY})
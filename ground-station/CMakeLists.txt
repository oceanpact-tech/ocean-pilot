cmake_minimum_required(VERSION 3.20)
project(ocean_pilot CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set output directory for binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

# ----- Boost setup -----
set(Boost_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost")
set(Boost_NO_SYSTEM_PATHS ON)

# Set RPATH for runtime library search path
set(CMAKE_BUILD_RPATH "${Boost_ROOT}/stage/lib")

# ----- MAVSDK setup -----
set(MAVSDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MAVSDK/cpp/")
set(MAVSDK_NO_SYSTEM_PATHS ON)

# cmake config 
list(PREPEND CMAKE_PREFIX_PATH 
    "${Boost_ROOT}/stage/lib/cmake"
    "${Boost_ROOT}"
    "${MAVSDK_ROOT}/build/install"
)

# ---------------------- 
# Dependencies
# ---------------------- 

find_package(Boost REQUIRED CONFIG 
    COMPONENTS system json) # Make sure to use its own cmake config files
find_package(OpenSSL REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

# ---------------------- 
# Libraries
# ---------------------- 

# ----- core_settings -----

add_library(core_settings INTERFACE)

target_include_directories(core_settings INTERFACE 
    include
    ${OPENSSL_INCLUDE_DIR}
    "${Boost_ROOT}"
)

target_link_libraries(core_settings INTERFACE 
    Boost::system
    Boost::json
    OpenSSL::SSL
    OpenSSL::Crypto
    pthread
)

# ----- dji_interface -----
add_library(dji_interface STATIC
    src/dji_interface/dji_interface.cpp
    src/dji_interface/telemetry_handler.cpp
    src/dji_interface/transport/http_handler.cpp
    src/dji_interface/transport/tcp_handler.cpp    
)
target_include_directories(dji_interface PUBLIC include/dji_interface)
target_link_libraries(dji_interface PUBLIC core_settings)

# ----- Mavlink -----
add_library(mavlink_core INTERFACE)
target_link_libraries(mavlink_core INTERFACE
    dji_interface
    MAVSDK::mavsdk
    Threads::Threads
)

# ---------------------- 
# Executables
# ---------------------- 

add_executable(${PROJECT_NAME} src/dji_interface/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE dji_interface)

add_executable(${PROJECT_NAME}_test_telemetry test/dji_interface/test_telemetry.cpp)
target_link_libraries(${PROJECT_NAME}_test_telemetry PRIVATE dji_interface)

add_executable(mavlink_bridge_test_telemetry test/mavlink_bridge/test_mavlink.cpp)
target_link_libraries(mavlink_bridge_test_telemetry PRIVATE mavlink_core)
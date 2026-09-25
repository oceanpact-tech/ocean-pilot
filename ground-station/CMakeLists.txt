cmake_minimum_required(VERSION 3.20)
project(ocean_pilot CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set output directory for binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

# Boost setup
set(Boost_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost")
set(Boost_NO_SYSTEM_PATHS ON)

# Set RPATH for runtime library search path
set(CMAKE_BUILD_RPATH "${Boost_ROOT}/stage/lib")

# Boost cmake config 
list(PREPEND CMAKE_PREFIX_PATH 
    "${Boost_ROOT}/stage/lib/cmake"
    "${Boost_ROOT}"
)

# ---------------------- 
# Dependencies
# ---------------------- 

find_package(Boost REQUIRED CONFIG COMPONENTS system json) # Make sure to use its own cmake config files
find_package(OpenSSL REQUIRED)

# ---------------------- 
# Libraries
# ---------------------- 

add_library(core_settings INTERFACE)

# Includes
target_include_directories(core_settings INTERFACE
    include
    include/transport
    include/custom_types
    ${OPENSSL_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(core_settings INTERFACE
    Boost::system
    Boost::json
    OpenSSL::SSL
    OpenSSL::Crypto
    pthread
)

# ---------------------- 
# Executables
# ---------------------- 

# Main executable
set(MAIN_SOURCE_FILES
    src/main.cpp
    src/http_handler.cpp
    src/dji_interface.cpp
    src/telemetry_handler.cpp
    src/transport/tcp_handler.cpp
)
add_executable(${PROJECT_NAME} ${MAIN_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE core_settings)

# Tests
set(TEST_SOURCE_FILES
    test/test_telemetry.cpp
    src/http_handler.cpp
    src/dji_interface.cpp
    src/telemetry_handler.cpp
    src/transport/tcp_handler.cpp
)
add_executable(${PROJECT_NAME}_test_telemetry ${TEST_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_test_telemetry PRIVATE core_settings)

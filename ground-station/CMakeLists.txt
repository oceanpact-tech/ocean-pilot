cmake_minimum_required(VERSION 3.20)
project(mavlink_bridge CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define a pasta onde os executáveis gerados serão salvos
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

# 1. Definir o caminho raiz usando a nomenclatura que o CMake moderno espera
set(Boost_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost")
set(Boost_NO_SYSTEM_PATHS ON)

# 2. Apontar onde estão os arquivos *.cmake gerados no build do Boost
# O Boost coloca os arquivos de configuração do CMake no próprio stage/lib/cmake
list(PREPEND CMAKE_PREFIX_PATH 
    "${Boost_ROOT}/stage/lib/cmake"
    "${Boost_ROOT}"
)

# 3. Chamar o find_package em modo CONFIG (usa os configs do próprio Boost, não o FindBoost do CMake)
find_package(Boost REQUIRED CONFIG COMPONENTS system)
find_package(OpenSSL REQUIRED)

# 4. Executável
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/http_handler.cpp
    src/dji_interface.cpp 
)

# 5. Includes
target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${OPENSSL_INCLUDE_DIR}
    # third_party/mavlink/c_library_v2
)

# 6. Linkagem direta com os targets modernos do Boost
target_link_libraries(${PROJECT_NAME} PRIVATE
    Boost::system
    OpenSSL::SSL
    OpenSSL::Crypto
    pthread
)

# 7. Garantir que o binário encontre os .so em runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${Boost_ROOT}/stage/lib"
)
cmake_minimum_required(VERSION 3.20)
project(projet_bras_robotique VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json pour clangd / clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Type de build" FORCE)
endif()

option(ENABLE_WARNINGS "Compile avec -Wall -Wextra -Wpedantic -Werror" ON)
option(ENABLE_SANITIZERS "Compile avec AddressSanitizer et UBSan" OFF)

# --- Dépendances ---
find_package(Eigen3 CONFIG REQUIRED)

# --- Bibliothèque de cinématique (logique pure, testable hors matériel) ---
add_library(kinematics
    src/planar_arm.cpp
    src/inverse_kinematics.cpp
)
target_include_directories(kinematics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(kinematics PUBLIC Eigen3::Eigen)
target_compile_features(kinematics PUBLIC cxx_std_20)

if(ENABLE_WARNINGS)
    target_compile_options(kinematics PRIVATE
        -Wall -Wextra -Wpedantic -Werror)
endif()

if(ENABLE_SANITIZERS)
    target_compile_options(kinematics PUBLIC
        -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(kinematics PUBLIC
        -fsanitize=address,undefined)
endif()

# --- Démo (viz Phase 0 : génère un SVG animé) ---
add_executable(arm_demo app/arm_demo.cpp)
target_link_libraries(arm_demo PRIVATE kinematics)
if(ENABLE_WARNINGS)
    target_compile_options(arm_demo PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# --- Tests ---
include(CTest)
if(BUILD_TESTING)
    find_package(Catch2 3 CONFIG REQUIRED)
    add_executable(kinematics_tests
        tests/test_planar_arm.cpp
        tests/test_inverse_kinematics.cpp
    )
    target_link_libraries(kinematics_tests PRIVATE kinematics Catch2::Catch2WithMain)

    if(ENABLE_WARNINGS)
        target_compile_options(kinematics_tests PRIVATE
            -Wall -Wextra -Wpedantic -Werror)
    endif()

    include(Catch)
    catch_discover_tests(kinematics_tests)
endif()

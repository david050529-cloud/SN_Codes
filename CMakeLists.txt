cmake_minimum_required(VERSION 3.14)
project(SN_Codes VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Eigen3 header-only linear algebra library
# Using an interface library (no compilation needed)
set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0")
add_library(Eigen3::Eigen INTERFACE IMPORTED)
set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
)

# SP++3.0 signal processing library (header-only)
set(SPPP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SP++3.0_new/include")

# Suppress warnings about Eigen used in headers
if(MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
    add_compile_options(/wd4244 /wd4267 /bigobj)
else()
    add_compile_options(-Wall -Wextra -Wa,-mbig-obj -O2)
endif()

# Static library: public utility functions for DOA
add_subdirectory(publicFunctionDoa)

# Static library: core DOA arithmetic algorithms
add_subdirectory(Arithmetic)

# Shared library (DLL): spoofing DOA detection
add_subdirectory(SpoofingDoaDll)

# Shared library (DLL): spoofing DOA for GN560 device
add_subdirectory(SpoofingDoaGN560Dll)

# Shared library (DLL): suppression DOA
add_subdirectory(SuppressDoaDll)

# Shared library (DLL): suppression DOA for GN560 device
add_subdirectory(SuppressDoaGN560Dll)

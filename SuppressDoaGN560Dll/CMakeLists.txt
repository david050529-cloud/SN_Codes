# Shared library (DLL): suppression DOA for GN560 device
# Uses SuppressDoa base class, configured for GN560 single-channel + directional antenna
# Base class sources are compiled directly to avoid Windows DLL symbol export issues

add_library(SuppressDoaGN560Dll SHARED
    interface_suppress_GN560.cpp
    dllmain.cpp
    pch.cpp
    # Base class sources from SuppressDoaDll
    ${CMAKE_SOURCE_DIR}/SuppressDoaDll/SuppressDoa.cpp
    ${CMAKE_SOURCE_DIR}/SuppressDoaDll/interf.cpp
    ${CMAKE_SOURCE_DIR}/SuppressDoaDll/jointdiag.cpp
    ${CMAKE_SOURCE_DIR}/SuppressDoaDll/sigleChannel.cpp
)

target_include_directories(SuppressDoaGN560Dll PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/SuppressDoaDll
    ${CMAKE_SOURCE_DIR}/publicFunctionDoa
    ${CMAKE_SOURCE_DIR}/Arithmetic
    ${SPPP_INCLUDE_DIR}
)

target_link_libraries(SuppressDoaGN560Dll PRIVATE
    Arithmetic
    publicFunctionDoa
    Eigen3::Eigen
)

set_target_properties(SuppressDoaGN560Dll PROPERTIES
    PREFIX ""
    OUTPUT_NAME "SuppressDoaGN560Dll"
)

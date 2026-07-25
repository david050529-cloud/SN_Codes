# Shared library (DLL): suppression/interference DOA estimation
# Supports single-channel (4-phase shifting), multi-channel,
# correlative interferometer, joint diagonalization for multi-signal DOA

add_library(SuppressDoaDll SHARED
    SuppressDoa.cpp
    interf.cpp
    jointdiag.cpp
    sigleChannel.cpp
    interface.cpp
    dllmain.cpp
    pch.cpp
)

target_include_directories(SuppressDoaDll PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/publicFunctionDoa
    ${CMAKE_SOURCE_DIR}/Arithmetic
    ${SPPP_INCLUDE_DIR}
)

target_link_libraries(SuppressDoaDll PRIVATE
    Arithmetic
    publicFunctionDoa
    Eigen3::Eigen
)

set_target_properties(SuppressDoaDll PROPERTIES
    PREFIX ""
    OUTPUT_NAME "SuppressDoaDll"
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)

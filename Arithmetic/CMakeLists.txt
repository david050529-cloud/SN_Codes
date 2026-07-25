# Static library: core DOA arithmetic algorithms
# Implements correlative interferometer, amplitude-phase DOA, virtual array expansion

add_library(Arithmetic STATIC
    arithmetic.cpp
)

target_include_directories(Arithmetic PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/publicFunctionDoa
)

target_link_libraries(Arithmetic PUBLIC
    publicFunctionDoa
)

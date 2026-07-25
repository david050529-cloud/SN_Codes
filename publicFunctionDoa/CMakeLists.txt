# Static library: public utility functions shared across DOA modules
# Provides logging, file I/O, string utilities, math helpers, Eigen wrappers

add_library(publicFunctionDoa STATIC
    publicFunctionDoa.cpp
)

target_include_directories(publicFunctionDoa PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(publicFunctionDoa PUBLIC
    Eigen3::Eigen
)

# Platform-specific: Windows needs Shlwapi for PathRemoveFileSpec
if(WIN32)
    target_link_libraries(publicFunctionDoa PUBLIC shlwapi)
endif()

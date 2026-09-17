cmake_minimum_required(VERSION 3.16)

project(DOA VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 静态库也要 PIC，方便被其他动态库/可执行文件链接
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDOA_DEBUG -D_DEBUG)
else()
    add_definitions(-DNDEBUG)
endif()

add_definitions(-DUNICODE -D_UNICODE)

if(WIN32)
    add_definitions(-D_WIN32 -D_WINDOWS)
elseif(UNIX AND NOT APPLE)
    add_definitions(-D__linux__)
elseif(APPLE)
    add_definitions(-D__APPLE__ -D__MACH__)
endif()

set(EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/eigen-3.4.0")
set(SPLAB_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/include")
set(JSON_INCLUDE_DIR  "${PROJECT_SOURCE_DIR}/lib/json")

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SPLAB_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
)

# ---- 库源文件（不含 main.cpp）----
set(SOURCES
    interface.cpp
    WW.cpp
    BaseDoa.cpp
    DireDoa.cpp
    OmniDoa.cpp
    MicroDoa.cpp
    SuppressingDoa.cpp
    MulPath.cpp
    GN902.cpp
)

set(HEADERS
    interface.h
    WW.h
    BaseDoa.h
    DireDoa.h
    OmniDoa.h
    MicroDoa.h
    SuppressingDoa.h
    MulPath.h
    GN902.h
    gn902_dat_parser.h
)

# ---- 编译选项（两种库共用）----
if(MSVC)
    set(DOA_COMPILE_OPTIONS /W4 /EHsc /wd4244 /wd4389 /wd4101 /wd4554 /wd4390)
else()
    set(DOA_COMPILE_OPTIONS
        -Wall -Wextra
        -Wno-unused-but-set-parameter
        -Wno-unused-but-set-variable
        -Wno-sign-compare
        -Wno-unused-parameter
        -Wno-unused-variable
        -Wno-unused-value
        -Wno-parentheses
        -Wno-vla
        -Wno-misleading-indentation
        -Wno-maybe-uninitialized
    )
endif()

find_package(Threads REQUIRED)

# ---- 动态库 ----
add_library(doa_shared SHARED ${SOURCES} ${HEADERS})
target_compile_options(doa_shared PRIVATE ${DOA_COMPILE_OPTIONS})
target_link_libraries(doa_shared PUBLIC Threads::Threads)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
    target_compile_definitions(doa_shared PRIVATE -DARM_ARCH)
endif()

# ---- 静态库 ----
add_library(doa_static STATIC ${SOURCES} ${HEADERS})
target_compile_options(doa_static PRIVATE ${DOA_COMPILE_OPTIONS})
target_link_libraries(doa_static PUBLIC Threads::Threads)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
    target_compile_definitions(doa_static PRIVATE -DARM_ARCH)
endif()

# ---- 输出名称 ----
# Linux/macOS：libDOA.so + libDOA.a
set_target_properties(doa_shared PROPERTIES OUTPUT_NAME DOA)
set_target_properties(doa_static PROPERTIES OUTPUT_NAME DOA)

if(WIN32)
    # Windows：DOA.dll + DOA.lib(导入库) + DOA_static.lib(静态库)
    set_target_properties(doa_shared PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
    set_target_properties(doa_static PROPERTIES OUTPUT_NAME DOA_static)
endif()

# ---- 可执行文件（可选，用于本地测试；链接动态库）----
add_executable(output main.cpp)
target_link_libraries(output PRIVATE doa_shared)
set_target_properties(output PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
    target_compile_definitions(output PRIVATE -DARM_ARCH)
endif()

# ---- 安装 ----
install(TARGETS doa_shared doa_static output
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES ${HEADERS} DESTINATION include)
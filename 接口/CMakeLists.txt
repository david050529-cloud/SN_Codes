cmake_minimum_required(VERSION 3.10)
project(GN902 LANGUAGES CXX)

# ---- 基本设置 ----
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ---- 源文件与头文件 ----
set(GN902_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/GN902.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gn902_interface.cpp
)

set(GN902_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/GN902.h
    ${CMAKE_CURRENT_SOURCE_DIR}/gn902_interface.h
)

set(GN902_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})

# ---- 输出目录 ----
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---- 动态库 ----
add_library(gn902 SHARED ${GN902_SOURCES} ${GN902_HEADERS})
target_include_directories(gn902 PUBLIC ${GN902_INCLUDE_DIRS})
set_target_properties(gn902 PROPERTIES
    OUTPUT_NAME "gn902"
    VERSION 1.0.0
    SOVERSION 1
    POSITION_INDEPENDENT_CODE ON
)
if(WIN32)
    # Windows 下自动导出所有符号（接口函数已用 __declspec(dllexport)，
    # 此选项可额外导出 GN902Container 等全局符号）
    set_target_properties(gn902 PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# ---- 静态库 ----
add_library(gn902_static STATIC ${GN902_SOURCES} ${GN902_HEADERS})
target_include_directories(gn902_static PUBLIC ${GN902_INCLUDE_DIRS})
set_target_properties(gn902_static PROPERTIES
    OUTPUT_NAME "gn902_static"
    POSITION_INDEPENDENT_CODE ON
)

# ---- 链接系统库 ----
if(UNIX AND NOT APPLE)
    # Linux 下数学函数需要显式链接 libm
    target_link_libraries(gn902 PRIVATE m)
    target_link_libraries(gn902_static PRIVATE m)
endif()

# ---- 安装规则（可选） ----
install(TARGETS gn902 gn902_static
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(FILES ${GN902_HEADERS} DESTINATION include)

# 生成 compile_commands.json，方便 IDE 跳转
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.16)
project(gn902 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

# 只包含 GN902 核心 + 精简 C 接口层
set(GN902_SOURCES
    GN902.cpp
    gn902_interface.cpp
)

# 动态库
add_library(gn902_shared SHARED ${GN902_SOURCES})
# 静态库
add_library(gn902_static STATIC ${GN902_SOURCES})

foreach(tgt gn902_shared gn902_static)
    target_include_directories(${tgt}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${tgt} PUBLIC Threads::Threads)
endforeach()

# 输出名：Linux 下 libGN902.so / libGN902.a
set_target_properties(gn902_shared PROPERTIES OUTPUT_NAME GN902)
set_target_properties(gn902_static PROPERTIES OUTPUT_NAME GN902)

if(WIN32)
    set_target_properties(gn902_shared PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
    set_target_properties(gn902_static PROPERTIES OUTPUT_NAME GN902_static)
endif()

install(TARGETS gn902_shared gn902_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES GN902.h gn902_interface.h DESTINATION include)
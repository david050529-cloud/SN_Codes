# Shared library (DLL): spoofing DOA for GN560 device
# Inherits from SpoofingDoa, configured for GN560's directional antenna array
# Base class sources are compiled directly to avoid Windows DLL symbol export issues

add_library(SpoofingDoaGN560Dll SHARED
    SpoofingDoaGN560.cpp
    interface_GN560.cpp
    dllmain.cpp
    pch.cpp
    # Base class sources from SpoofingDoaDll
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/SpoofingDoa.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/Alarm.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/AmpPhase.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/Corrected.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/Directed.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/InitData.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/Interf.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/Omni.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/PreparationData.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/SpectrumDesity.cpp
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll/WriteLog.cpp
)

target_include_directories(SpoofingDoaGN560Dll PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/SpoofingDoaDll
    ${CMAKE_SOURCE_DIR}/publicFunctionDoa
    ${CMAKE_SOURCE_DIR}/Arithmetic
)

target_link_libraries(SpoofingDoaGN560Dll PRIVATE
    Arithmetic
    publicFunctionDoa
)

set_target_properties(SpoofingDoaGN560Dll PROPERTIES
    PREFIX ""
    OUTPUT_NAME "SpoofingDoaGN560Dll"
)

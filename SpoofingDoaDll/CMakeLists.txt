# Shared library (DLL): GNSS spoofing detection and DOA estimation
# Detects spoofing signals across GPS/BDS/GLONASS/Galileo constellations
# using correlative interferometer or amplitude-phase methods

add_library(SpoofingDoaDll SHARED
    Alarm.cpp
    AmpPhase.cpp
    Corrected.cpp
    Directed.cpp
    InitData.cpp
    Interf.cpp
    Omni.cpp
    PreparationData.cpp
    SpectrumDesity.cpp
    SpoofingDoa.cpp
    WriteLog.cpp
    interface.cpp
    dllmain.cpp
    pch.cpp
)

target_include_directories(SpoofingDoaDll PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/publicFunctionDoa
    ${CMAKE_SOURCE_DIR}/Arithmetic
)

target_link_libraries(SpoofingDoaDll PRIVATE
    Arithmetic
    publicFunctionDoa
)

# Set output name (without lib prefix on Linux)
set_target_properties(SpoofingDoaDll PROPERTIES
    PREFIX ""
    OUTPUT_NAME "SpoofingDoaDll"
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)

if(WIN32)
    target_link_libraries(SpoofingDoaDll PRIVATE shlwapi)
endif()

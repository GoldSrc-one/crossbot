cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0091)
    cmake_policy(SET CMP0091 NEW)
endif()

project(crossbot CXX)

# GoldSrc dedicated servers don't ship VC++ redistributable -- link CRT statically.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# GoldSrc is 32-bit-only (Windows and Linux). On MSVC, enforce with Win32 generator (cmake -B build -A Win32); on GCC/Clang, force via compile/link flags.
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR
        "crossbot must be built 32-bit (GoldSrc requirement). "
        "On MSVC, configure with: cmake -B build -A Win32")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(HLSDK_DIR ${CMAKE_SOURCE_DIR}/external/hlsdk)
set(METAMOD_DIR ${CMAKE_SOURCE_DIR}/external/metamod)

file(GLOB_RECURSE CROSSBOT_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

add_library(crossbot_mm SHARED ${CROSSBOT_SOURCES})

target_include_directories(crossbot_mm PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${HLSDK_DIR}/common
    ${HLSDK_DIR}/dlls
    ${HLSDK_DIR}/engine
    ${HLSDK_DIR}/game_shared
    ${HLSDK_DIR}/pm_shared
    ${HLSDK_DIR}/public
    ${METAMOD_DIR}
)

set_target_properties(crossbot_mm PROPERTIES
    PREFIX ""
    OUTPUT_NAME "crossbot_mm"
)

if(MSVC)
    target_compile_options(crossbot_mm PRIVATE /W3 /GR-)
    target_compile_definitions(crossbot_mm PRIVATE _CRT_SECURE_NO_WARNINGS)

    # GiveFnptrsToDll is __stdcall (WINAPI); without .def file MSVC would export decorated name (_GiveFnptrsToDll@8), which engine's GetProcAddress("GiveFnptrsToDll") can't find.
    target_sources(crossbot_mm PRIVATE ${CMAKE_SOURCE_DIR}/src/meta/crossbot.def)
else()
    target_compile_options(crossbot_mm PRIVATE -m32 -Wall -fno-rtti)
    target_link_options(crossbot_mm PRIVATE -m32 -static-libgcc -static-libstdc++)
    target_compile_definitions(crossbot_mm PRIVATE linux __linux__)
endif()

if(WIN32)
    target_compile_definitions(crossbot_mm PRIVATE _WINDOWS WIN32)
endif()

# Offline BSP nav-graph compiler (Milestone 2) -- standalone tool, not linked against Metamod/engine headers. See tools/navgen/CMakeLists.txt.
add_subdirectory(tools/navgen)

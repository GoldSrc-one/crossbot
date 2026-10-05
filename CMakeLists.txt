cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0091)
    cmake_policy(SET CMP0091 NEW)
endif()

project(crossbot CXX)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# GoldSrc only loads a 32-bit plugin, while navgen is an offline tool built 64-bit. Each configure
# builds one of them: 32-bit gives crossbot_mm, 64-bit gives navgen. build.bat does both.
set(CROSSBOT_64BIT OFF)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CROSSBOT_64BIT ON)
    message(STATUS "crossbot: 64-bit configure, building navgen only")
else()
    message(STATUS "crossbot: 32-bit configure, building crossbot_mm only")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(HLSDK_DIR ${CMAKE_SOURCE_DIR}/external/hlsdk)
set(METAMOD_DIR ${CMAKE_SOURCE_DIR}/external/metamod)
set(CROSSPLAY_DIR ${CMAKE_SOURCE_DIR}/external/crossplay)

if(NOT CROSSBOT_64BIT)

file(GLOB CROSSBOT_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/console/*.cpp
    ${CMAKE_SOURCE_DIR}/src/bot/*.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/*.cpp
    ${CMAKE_SOURCE_DIR}/src/game/*.cpp
    ${CMAKE_SOURCE_DIR}/src/game/*/*.cpp
    ${CMAKE_SOURCE_DIR}/src/meta/*.cpp
)

add_library(crossbot_mm SHARED ${CROSSBOT_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/navgen/crc32.cpp
)

target_include_directories(crossbot_mm PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/navgen
    ${CMAKE_SOURCE_DIR}/external/jsmn
    ${CROSSPLAY_DIR}
    ${HLSDK_DIR}/common
    ${HLSDK_DIR}/dlls
    ${HLSDK_DIR}/engine
    ${HLSDK_DIR}/game_shared
    ${HLSDK_DIR}/pm_shared
    ${HLSDK_DIR}/public
    ${METAMOD_DIR}
)

target_link_directories(crossbot_mm PRIVATE ${CROSSPLAY_DIR})

set_target_properties(crossbot_mm PROPERTIES
    PREFIX ""
    OUTPUT_NAME "crossbot_mm"
)

if(MSVC)
    target_compile_options(crossbot_mm PRIVATE /W3 /GR-)
    target_compile_definitions(crossbot_mm PRIVATE _CRT_SECURE_NO_WARNINGS)

    target_sources(crossbot_mm PRIVATE ${CMAKE_SOURCE_DIR}/src/meta/crossbot.def)
else()
    target_compile_options(crossbot_mm PRIVATE -m32 -Wall -fno-rtti)
    target_link_options(crossbot_mm PRIVATE -m32 -static-libgcc -static-libstdc++)
    target_compile_definitions(crossbot_mm PRIVATE linux __linux__)
endif()

if(WIN32)
    target_compile_definitions(crossbot_mm PRIVATE _WINDOWS WIN32)
endif()

option(CROSSBOT_LADDER_DEBUG "Per-frame ladder state trace on the server console" OFF)
if(CROSSBOT_LADDER_DEBUG)
    target_compile_definitions(crossbot_mm PRIVATE CROSSBOT_LADDER_DEBUG)
    message(STATUS "crossbot: ladder state trace ENABLED")
endif()

option(CROSSBOT_VERBOSE "Per-bot narration of movement, planning and item choice" ON)
if(CROSSBOT_VERBOSE)
    target_compile_definitions(crossbot_mm PRIVATE CROSSBOT_VERBOSE)
else()
    message(STATUS "crossbot: verbose bot logging DISABLED")
endif()

option(CROSSBOT_PERF "Zone/trace profiler and its crossbot_perf cvars" OFF)
if(CROSSBOT_PERF)
    target_compile_definitions(crossbot_mm PRIVATE CROSSBOT_PERF)
    message(STATUS "crossbot: zone profiler ENABLED")
endif()

endif() # NOT CROSSBOT_64BIT

if(CROSSBOT_64BIT)

file(GLOB NAVGEN_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/src/navgen/*.cpp
)

add_executable(navgen ${NAVGEN_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/nav/airmove_cache.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/funnel_graph.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/plan_search.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/waypoint_plan.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/path_profile.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/path_follower.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/pm_sim.cpp
)

set_target_properties(navgen PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools/navgen
)

target_include_directories(navgen PRIVATE
    ${CMAKE_SOURCE_DIR}/src/navgen
    ${CMAKE_SOURCE_DIR}/src/nav
    ${HLSDK_DIR}/common
    ${HLSDK_DIR}/engine
    ${HLSDK_DIR}/dlls
)

find_package(Threads REQUIRED)
target_link_libraries(navgen PRIVATE Threads::Threads)

if(MSVC)
    target_compile_options(navgen PRIVATE /W3 /Ob2)
    target_compile_definitions(navgen PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(navgen PRIVATE -Wall)
endif()

endif() # CROSSBOT_64BIT

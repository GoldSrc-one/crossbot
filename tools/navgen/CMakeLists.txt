# Offline BSP nav-graph compiler. Deliberately NOT linked against Metamod or
# any engine headers -- pure BSP30 file parsing and geometry, so it only
# needs the vendored, engine-independent HLSDK struct/constant headers
# (external/hlsdk/engine/bspfile.h, external/hlsdk/common/const.h). Built by
# the same (32-bit-pinned, see root CMakeLists.txt) toolchain as crossbot_mm
# purely to keep one build configuration -- it has no GoldSrc bitness
# requirement of its own.

file(GLOB_RECURSE NAVGEN_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_executable(navgen ${NAVGEN_SOURCES})

target_include_directories(navgen PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${HLSDK_DIR}/common
    ${HLSDK_DIR}/engine
    ${CMAKE_SOURCE_DIR}/src/nav
)

if(MSVC)
    target_compile_options(navgen PRIVATE /W3)
    target_compile_definitions(navgen PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(navgen PRIVATE -Wall)
endif()

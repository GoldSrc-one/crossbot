# Offline BSP nav-graph compiler. Deliberately NOT linked against Metamod or
# any actual engine (no g_engfuncs calls, no running game) -- pure BSP30 file
# parsing and geometry, plus the shared src/nav/astar_graph.cpp pathfinding
# module (engine-independent by design, see its own header comment). Still
# freely uses vendored, struct/math-only HLSDK headers where convenient
# (bspfile.h/const.h for the BSP format, dlls/vector.h for the same Vector
# type the runtime plugin uses -- sharing the real type instead of a
# hand-rolled lookalike is what lets astar_graph.cpp compile unmodified on
# both sides). Built by the same (32-bit-pinned, see root CMakeLists.txt)
# toolchain as crossbot_mm purely to keep one build configuration -- it has
# no GoldSrc bitness requirement of its own.

file(GLOB_RECURSE NAVGEN_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_executable(navgen ${NAVGEN_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/nav/astar_graph.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/funnel_graph.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/waypoint_plan.cpp
    ${CMAKE_SOURCE_DIR}/src/nav/path_follower.cpp
)

target_include_directories(navgen PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${HLSDK_DIR}/common
    ${HLSDK_DIR}/engine
    ${HLSDK_DIR}/dlls
    ${CMAKE_SOURCE_DIR}/src/nav
)

if(MSVC)
    target_compile_options(navgen PRIVATE /W3)
    target_compile_definitions(navgen PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(navgen PRIVATE -Wall)
endif()

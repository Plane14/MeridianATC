cmake_minimum_required(VERSION 3.20)
project(MeridianATC_XP VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(XPlaneSDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/SDK" CACHE PATH "X-Plane SDK root")
option(MERIDIAN_BUILD_PLUGIN "Build the X-Plane plugin" ON)
include(CTest)

if(MERIDIAN_BUILD_PLUGIN)
    set(INCLUDE_FMOD_SOUND 0 CACHE STRING "FMOD disabled: no external sound dependency" FORCE)
    add_subdirectory(lib/XPMP2 EXCLUDE_FROM_ALL)
    target_compile_definitions(XPMP2 PUBLIC XPLM302=1 XPLM410=1 XPLM420=1 XPLM430=1)
    add_library(MeridianATC MODULE src/Plugin.cpp)
    target_include_directories(MeridianATC PRIVATE lib "${XPlaneSDK_DIR}/CHeaders/XPLM")
    target_compile_definitions(MeridianATC PRIVATE MERIDIAN_XPMP2_VERSION="3.7.0" NOMINMAX WIN32_LEAN_AND_MEAN)
    target_link_libraries(MeridianATC PRIVATE XPMP2)
    if(WIN32)
        set(PLATFORM_DIR win_x64)
        target_link_libraries(MeridianATC PRIVATE "${XPlaneSDK_DIR}/Libraries/Win/XPLM_64.lib" opengl32 glu32 ws2_32 wsock32 iphlpapi)
    elseif(APPLE)
        set(PLATFORM_DIR mac_x64)
        target_link_libraries(MeridianATC PRIVATE "${XPlaneSDK_DIR}/Libraries/Mac/XPLM.framework" "-framework OpenGL" "-framework IOKit" "-framework CoreFoundation")
        target_link_options(MeridianATC PRIVATE "-undefined" "dynamic_lookup")
    else()
        set(PLATFORM_DIR lin_x64)
        find_package(OpenGL REQUIRED)
        find_package(Threads REQUIRED)
        target_link_libraries(MeridianATC PRIVATE OpenGL::GL OpenGL::GLU Threads::Threads ${CMAKE_DL_LIBS})
    endif()
    if(MSVC)
        target_compile_options(MeridianATC PRIVATE /W4 /EHsc)
    else()
        target_compile_options(MeridianATC PRIVATE -Wall -Wextra -Wpedantic -fvisibility=hidden)
    endif()
    set(PLUGIN_ROOT "${CMAKE_BINARY_DIR}/MeridianATC")
    set_target_properties(MeridianATC PROPERTIES PREFIX "" SUFFIX ".xpl"
        LIBRARY_OUTPUT_DIRECTORY "${PLUGIN_ROOT}/${PLATFORM_DIR}"
        RUNTIME_OUTPUT_DIRECTORY "${PLUGIN_ROOT}/${PLATFORM_DIR}")
    foreach(CONFIG DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        set_target_properties(MeridianATC PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${PLUGIN_ROOT}/${PLATFORM_DIR}"
            RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${PLUGIN_ROOT}/${PLATFORM_DIR}")
    endforeach()
    add_custom_command(TARGET MeridianATC POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/lib/XPMP2/Resources" "${PLUGIN_ROOT}/Resources"
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/Resources" "${PLUGIN_ROOT}/Resources"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${PLUGIN_ROOT}/Licenses"
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/lib/SDK/license.txt" "${PLUGIN_ROOT}/Licenses/SDK.txt"
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/lib/json-LICENSE.MIT" "${PLUGIN_ROOT}/Licenses/json.txt"
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/lib/XPMP2/LICENSE" "${PLUGIN_ROOT}/Licenses/XPMP2.txt")
endif()

if(BUILD_TESTING)
    add_executable(meridian_protocol_tests tests/TrafficTests.cpp)
    target_include_directories(meridian_protocol_tests PRIVATE src lib)
    if(WIN32)
        target_compile_definitions(meridian_protocol_tests PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
        target_link_libraries(meridian_protocol_tests PRIVATE ws2_32)
    endif()
    add_test(NAME traffic_protocol COMMAND meridian_protocol_tests)
endif()

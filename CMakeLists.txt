cmake_minimum_required(VERSION 3.18)
project(dual_hik_stitch LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

set(LYSTREAM_SOURCE_DIR "" CACHE PATH "Path to the PT233/lystream checkout")
if(NOT LYSTREAM_SOURCE_DIR)
    include(FetchContent)
    FetchContent_Declare(lystream_headers
        GIT_REPOSITORY https://github.com/PT233/lystream.git
        GIT_TAG 7f12d207a7514863ac6d225c504a83409d17882c
        SOURCE_SUBDIR headers_only)
    FetchContent_MakeAvailable(lystream_headers)
    set(LYSTREAM_SOURCE_DIR "${lystream_headers_SOURCE_DIR}")
endif()
if(NOT EXISTS "${LYSTREAM_SOURCE_DIR}/stream_control.h")
    message(FATAL_ERROR "LYSTREAM_SOURCE_DIR must point to PT233/lystream")
endif()

find_package(Threads REQUIRED)
find_package(OpenMP QUIET COMPONENTS CXX)
add_library(stitch_core STATIC src/stitch.cpp src/pipeline.cpp src/raw_output.cpp src/lystream_port.cpp)
target_include_directories(stitch_core PUBLIC src "${LYSTREAM_SOURCE_DIR}")
target_link_libraries(stitch_core PUBLIC Threads::Threads)
if(OpenMP_CXX_FOUND)
    target_link_libraries(stitch_core PRIVATE OpenMP::OpenMP_CXX)
endif()
add_executable(dual-hik-stitch src/main.cpp)
target_link_libraries(dual-hik-stitch PRIVATE stitch_core)
if(MINGW)
    target_link_options(dual-hik-stitch PRIVATE -municode)
endif()

function(enable_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()
enable_warnings(stitch_core)
enable_warnings(dual-hik-stitch)

option(ENABLE_MVS "Build the Hikrobot industrial camera source" OFF)
if(ENABLE_MVS)
    set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK installation")
    find_path(MVS_INCLUDE_DIR MvCameraControl.h HINTS "${MVS_ROOT}/include")
    find_library(MVS_LIBRARY NAMES MvCameraControl
        HINTS "${MVS_ROOT}/lib/aarch64" "${MVS_ROOT}/lib/64" "${MVS_ROOT}/lib")
    if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
        message(FATAL_ERROR "Set MVS_ROOT or pass MVS_INCLUDE_DIR and MVS_LIBRARY explicitly")
    endif()
    target_sources(dual-hik-stitch PRIVATE src/mvs.cpp)
    target_compile_definitions(dual-hik-stitch PRIVATE ENABLE_MVS)
    target_include_directories(dual-hik-stitch SYSTEM PRIVATE "${MVS_INCLUDE_DIR}")
    target_link_libraries(dual-hik-stitch PRIVATE "${MVS_LIBRARY}")
endif()

include(GNUInstallDirs)
install(TARGETS dual-hik-stitch RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(PROGRAMS deploy/run-rtsp.sh tools/board_baseline.py DESTINATION "${CMAKE_INSTALL_LIBEXECDIR}/dual-hik-stitch")

include(CTest)
if(BUILD_TESTING)
    add_executable(stitch-regression tests/regression.cpp)
    target_link_libraries(stitch-regression PRIVATE stitch_core)
    enable_warnings(stitch-regression)
    add_test(NAME recovery_and_output COMMAND stitch-regression)
    add_test(NAME synthetic_cli COMMAND ${CMAKE_COMMAND}
        -DAPP=$<TARGET_FILE:dual-hik-stitch> -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}
        -DWORK=${CMAKE_CURRENT_BINARY_DIR} -DPARALLEL=${OpenMP_CXX_FOUND}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoke.cmake)
    set_tests_properties(recovery_and_output synthetic_cli PROPERTIES TIMEOUT 15)
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME camera_baseline COMMAND ${CMAKE_COMMAND}
            -DAPP=$<TARGET_FILE:dual-hik-stitch> -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}
            -DWORK=${CMAKE_CURRENT_BINARY_DIR} -DPYTHON=${Python3_EXECUTABLE}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/baseline.cmake)
        set_tests_properties(camera_baseline PROPERTIES TIMEOUT 15)
    endif()
    if(UNIX)
        add_test(NAME rtsp_supervisor COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/supervisor.sh
            ${CMAKE_CURRENT_SOURCE_DIR}/deploy/run-rtsp.sh)
        set_tests_properties(rtsp_supervisor PROPERTIES TIMEOUT 15)
    endif()
endif()

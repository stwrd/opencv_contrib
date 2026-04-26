cmake_minimum_required(VERSION 3.16)
project(realtime_stab_sdk LANGUAGES CXX)

include(GNUInstallDirs)

option(RTSDK_BUILD_EXAMPLE "Build standalone example" ON)
option(RTSDK_BUILD_PC_VIDEO_TEST "Build PC video in/out ffmpeg example" ON)

add_library(realtime_stab_sdk
    src/realtime_stab_sdk.cpp
)

target_include_directories(realtime_stab_sdk
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(realtime_stab_sdk PUBLIC cxx_std_14)
set_target_properties(realtime_stab_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(MSVC)
  target_compile_options(realtime_stab_sdk PRIVATE /O2 /GL)
else()
  target_compile_options(realtime_stab_sdk PRIVATE -O3 -ffast-math -fvisibility=hidden)
endif()

if(RTSDK_BUILD_EXAMPLE)
  add_executable(rtsdk_example examples/main.cpp)
  target_link_libraries(rtsdk_example PRIVATE realtime_stab_sdk)
endif()

if(RTSDK_BUILD_PC_VIDEO_TEST)
  add_executable(rtsdk_pc_video_test examples/pc_video_stabilize_ffmpeg.cpp)
  target_link_libraries(rtsdk_pc_video_test PRIVATE realtime_stab_sdk)
endif()

install(TARGETS realtime_stab_sdk
    EXPORT realtime_stab_sdkTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES include/realtime_stab_sdk.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

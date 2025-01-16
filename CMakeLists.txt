cmake_minimum_required(VERSION 3.6)
project(sturdr)

string(ASCII 27 Esc)
set(Reset       "${Esc}[m")
set(Bold        "${Esc}[1m")
set(Red         "${Esc}[31m")
set(Green       "${Esc}[32m")
set(Yellow      "${Esc}[33m")
set(Blue        "${Esc}[34m")
set(Magenta     "${Esc}[35m")
set(Cyan        "${Esc}[36m")
set(White       "${Esc}[37m")
set(BoldRed     "${Esc}[1;31m")
set(BoldGreen   "${Esc}[1;32m")
set(BoldYellow  "${Esc}[1;33m")
set(BoldBlue    "${Esc}[1;34m")
set(BoldMagenta "${Esc}[1;35m")
set(BoldCyan    "${Esc}[1;36m")
set(BoldWhite   "${Esc}[1;37m")

# --- Version Control ---
set(STURDR_MAJOR_VERSION 0)
set(STURDR_MINOR_VERSION 0)
set(STURDR_PATCH_VERSION 1)
set(PROJECT_VERSION ${STURDR_MAJOR_VERSION}.${STURDR_MINOR_VERSION}.${STURDR_PATCH_VERSION})
message(STATUS "${BoldCyan}${PROJECT_NAME} version ${PROJECT_VERSION}${Reset}")

# --- Compiler Flags ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -W")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Ofast -ffast-math -fomit-frame-pointer -fstrict-aliasing -flto -DNDEBUG -march=native")

# --- Add Dependencies ---
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(navtools REQUIRED)
find_package(satutils REQUIRED)
find_package(sturdio REQUIRED)
find_package(sturdins REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(FFTW REQUIRED fftw3 IMPORTED_TARGET)

find_library(
    FFTW_DOUBLE_THREADS_LIB
    NAMES "fftw3_threads"
    PATHS ${PKG_FFTW_LIBRARY_DIRS} ${LIB_INSTALL_DIR}
)
if(FFTW_DOUBLE_THREADS_LIB)
    set(FFTW_DOUBLE_THREADS_LIB_FOUND TRUE)
    set(FFTW_LIBRARIES ${FFTW_LIBRARIES} ${FFTW_DOUBLE_THREADS_LIB})
    add_library(FFTW::DoubleThreads INTERFACE IMPORTED)
    set_target_properties(FFTW::DoubleThreads
        PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${FFTW_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${FFTW_DOUBLE_THREADS_LIB}"
    )
else()
    set(FFTW_DOUBLE_THREADS_LIB_FOUND FALSE)
endif()

set(STURDR_HDRS
    include/sturdr/acquisition.hpp
    include/sturdr/channel.hpp
    include/sturdr/channel-gps-l1ca.hpp
    include/sturdr/concurrent-barrier.hpp
    include/sturdr/concurrent-queue.hpp
    include/sturdr/data-type-adapters.hpp
    include/sturdr/discriminator.hpp
    include/sturdr/fftw-wrapper.hpp
    include/sturdr/gnss-signal.hpp
    include/sturdr/lock-detectors.hpp
    include/sturdr/structs-enums.hpp
    include/sturdr/sturdr.hpp
    include/sturdr/tracking.hpp
)

set(STURDR_SRCS
    src/acquisition.cpp
    src/channel-gps-l1ca.cpp
    src/data-type-adapters.cpp
    src/discriminator.cpp
    src/fftw-wrapper.cpp
    src/gnss-signal.cpp
    src/lock-detectors.cpp
    src/structs-enums.cpp
    src/sturdr.cpp
    src/tracking.cpp
)

# --- Create the C++ Library ---
add_library(${PROJECT_NAME} ${STURDR_HDRS} ${STURDR_SRCS})
target_include_directories(
    ${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${Eigen3_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:${Eigen3_INCLUDE_DIRS}>
)
target_link_libraries(
    ${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    spdlog::spdlog
    navtools
    satutils
    sturdio
    sturdins
    PkgConfig::FFTW
    ${FFTW_DOUBLE_THREADS_LIB}
)

# --- Add Executables ---
if (NOT DEFINED INSTALL_STURDR_EXAMPLES OR NOT INSTALL_STURDR_EXAMPLES)
else()
    file(GLOB testfiles "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    foreach(file ${testfiles})
        # message(${file})
        get_filename_component(testname ${file} NAME_WE)
        add_executable(${testname} ${file})
        target_link_libraries( ${testname} PUBLIC ${PROJECT_NAME})
    endforeach(file ${testfiles})
endif()
add_executable(run_sturdr scripts/run_sturdr.cpp)
target_link_libraries(run_sturdr PUBLIC ${PROJECT_NAME})

# --- Make Library 'Findable' for other CMake Packages ---
include(CMakePackageConfigHelpers)

install(
    DIRECTORY include/ 
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

export(
    TARGETS ${PROJECT_NAME} 
    FILE ${PROJECT_NAME}Targets.cmake
)

install(
    EXPORT ${PROJECT_NAME}Targets 
    FILE ${PROJECT_NAME}Targets.cmake 
    DESTINATION lib/cmake/${PROJECT_NAME}
)

configure_package_config_file(
    "${PROJECT_NAME}-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    PATH_VARS PROJECT_NAME
)

install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)
cmake_minimum_required(VERSION 3.16)
project(aemot_realtime CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to an optimized build unless the caller asked for something else -
# without this, plain `cmake -B build` gives you an unoptimized binary, which
# has bad performance for real-time event rates.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Enable the build machine's full vector instruction set (AVX2/AVX-512/etc,
# whatever this CPU actually has) rather than the compiler's conservative
# default baseline. Verified directly: on the matrix/vector operations in
# kalman.cpp, this gives an additional ~1.9-2.1x speedup on top of any
# algorithmic change, since Eigen's fixed-size matrix ops vectorize well.
# Tradeoff: the resulting binary is tied to THIS machine's CPU and may
# crash with an "illegal instruction" error if copied to different
# hardware. Set -DAEMOT_MARCH_NATIVE=OFF to build a portable binary instead
# (e.g. if you build on one machine and deploy to another).
option(AEMOT_MARCH_NATIVE "Compile for this machine's exact CPU (not portable to other hardware)" ON)
if(AEMOT_MARCH_NATIVE)
    add_compile_options(-march=native)
endif()

# --- Dependencies ---------------------------------------------------------

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp) # fallback for other versions (this was needed in the past on some devices)
    add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
    target_include_directories(yaml-cpp::yaml-cpp INTERFACE ${YAML_CPP_INCLUDE_DIR})
    target_link_libraries(yaml-cpp::yaml-cpp INTERFACE ${YAML_CPP_LIBRARIES})
endif()

find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)


#find_package(Boost 1.70 REQUIRED)
#target_link_libraries(aemot_realtime PRIVATE Boost::boost)

# --- Executable ------------------------------------------------------------

set(SHARED_SOURCES
    src/processing.cpp
    src/TrackManager.cpp
    src/kalman.cpp
    src/SAEdetector.cpp
    src/parameters.cpp
    src/SAE.cpp
    src/track_logger.cpp
    src/track_summary_logger.cpp
    src/raw_event_logger.cpp
)

add_executable(aemot_realtime
    src/camera_driver.cpp
    ${SHARED_SOURCES}
)

add_executable(aemot_replay
    src/file_driver.cpp
    ${SHARED_SOURCES}
)

# gen4/common is a sibling of this project's own root (aemot_realtime/), holding
# the sepia/psee/evk4 camera headers. Giving it as a proper include path
# (rather than relying on a "../gen4/common/x.hpp" relative include inside
# each source file) means moving files around within src/ later can't silently break the camera includes
target_include_directories(aemot_realtime PRIVATE
    ${CMAKE_SOURCE_DIR}/../gen4/common
    ${LIBUSB_INCLUDE_DIRS}/..
)

target_include_directories(aemot_replay PRIVATE
    ${CMAKE_SOURCE_DIR}/../gen4/common
)

target_link_libraries(aemot_realtime PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    PkgConfig::LIBUSB
    Threads::Threads
)

# replay doesn't require libusb at all
target_link_libraries(aemot_replay PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    Threads::Threads
)

# processing.cpp resolves configs/<name>.yaml relative to the running
# executable's own location via /proc/self/exe (see executable_directory()
# in processing.cpp), assuming configs/ sits exactly one directory above
# wherever the binary lands. Putting the binary straight in the build
# directory root (the default here - no subdirectories) keeps that true as
# long as you build from the aemot_realtime/ project root, e.g. `cmake -B build`
# run from here puts the binary at aemot_realtime/build/aemot_realtime, and
# aemot_realtime/build/../configs correctly resolves to aemot_realtime/configs.

cmake_minimum_required(VERSION 3.15)

# set project name
project(yapgt)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# search for opencv and make it a required depenency
set(OPENCV_STATIC ON)
find_package(OpenCV REQUIRED)

# add the include folder for the header files
include_directories(include)

# create the variable SOURCES, which is a list of all source and header files
set(SOURCES
        src/calibration.cpp
        src/main.cpp
        src/ply-model-exporter.cpp
        src/argparser.cpp
        src/reconstruction.cpp
        src/image-pair.cpp 
        src/argparser.cpp
        src/sequence-matcher.cpp
        src/scene-sequence.cpp
        src/imagepair-sequence-iterator.cpp
        src/colors.cpp)

# add the files from SOURCES to the executable
add_executable(yapgt ${SOURCES})

# set options for linking static libraries when using MSVC 
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(yapgt PRIVATE /MDd)
    target_link_options(yapgt PRIVATE /INCREMENTAL:NO /NODEFAULTLIB:MSVCRT)
endif()

# add the link from the opencv libs
target_link_libraries(yapgt ${OpenCV_LIBS})

# Enable C++ 17 Standard
set_target_properties(yapgt PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
)

# copy each linked opencv library to the binary directory
if (Win32)
    foreach(CVLib ${OpenCV_LIBS})
        file(COPY ${_OpenCV_LIB_PATH}/${CVLib}${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll DESTINATION ${CMAKE_BINARY_DIR})
    endforeach()
endif()

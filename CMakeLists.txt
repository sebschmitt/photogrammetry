cmake_minimum_required(VERSION 3.15)

# set project name
project(yapgt)

# generate compilation database for YouCompleteMe
# option(CMAKE_EXPORT_COMPILE_COMMANDS "Generate compile_commands.json in build directory" ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(JSON_BuildTests OFF CACHE INTERNAL "")

# search for opencv and make it a required depenency
find_package(OpenCV REQUIRED)


# add the include folder for the header files
# target_include_directories(OpenCVTest PRIVATE ./include)
include_directories(include)

# add_subdirectory(third-party/json)

# create the variable SOURCES, which is a list of all source and header files
set(SOURCES 
        src/CameraCalibration.cpp
        src/calibration.cpp
        src/main.cpp
        src/feature-matching.cpp
        src/ply-model-exporter.cpp
        src/essential-matrix.cpp
        src/argparser.cpp
        src/reconstruction.cpp
        src/image-pair.cpp 
        src/argparser.cpp
        src/sequence-matcher.cpp
        src/scene-sequence.cpp
        src/imagepair-sequence-iterator.cpp)

# add the files from SOURCES to the executable
add_executable(yapgt ${SOURCES})

# add the link from the opencv libs
target_link_libraries(yapgt PRIVATE ${OpenCV_LIBS})

# Enable C++ 17 Standard
set_target_properties(yapgt PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
)
cmake_minimum_required(VERSION 3.15)
project(yapgt)

# search for opencv and make it a required depenency
find_package(OpenCV REQUIRED)

# add include folder for header files
include_directories(include)

# create the variable SOURCES, which is a list of all source and header files
set(SOURCES
        src/CameraCalibration.cpp
        src/calibration.cpp
        src/main.cpp
        src/feature-matching.cpp
        src/ply-model-exporter.cpp
        src/essential-matrix.cpp
        include/argparser.h src/argparser.cpp)

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

file(COPY resources DESTINATION ${CMAKE_BINARY_DIR})

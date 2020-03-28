cmake_minimum_required(VERSION 3.13.4)

# set project name
project(OpenCVTest)

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
	# include/calibration.hpp
    # include/feature-matching.hpp
)

# add the files from SOURCES to the executable
add_executable(OpenCVTest ${SOURCES})

# add the link from the opencv libs
target_link_libraries(OpenCVTest PRIVATE ${OpenCV_LIBS})# nlohmann_json::nlohmann_json)

# Enable C++ 17 Standard
set_target_properties(OpenCVTest PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

file(COPY resources DESTINATION ${CMAKE_BINARY_DIR})


cmake_minimum_required(VERSION 3.15)
project(yapgt)

# search for opencv and make it a required depenency
find_package(OpenCV REQUIRED)

# add include folder for header files
include_directories(include)

# create the variable SOURCES, which is a list of all source and header files
set(SOURCES
        src/main.cpp
)

# add the files from SOURCES to the executable
add_executable(yapgt ${SOURCES})

# add the link from the opencv libs
target_link_libraries(yapgt PRIVATE ${OpenCV_LIBS})# nlohmann_json::nlohmann_json)

# set c++ 17 standard
set(CMAKE_CXX_STANDARD 17)
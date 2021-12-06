# https://rvarago.medium.com/introduction-to-cmake-for-cpp-4c464272a239

cmake_minimum_required(VERSION 2.8)

# project name
project( SPOT_IT_3D )

# opencv - specify which version we want to use (e.g. 4.2.0 or 4.5.2?)
find_package( OpenCV 4 REQUIRED )
# find_package( OpenCV 4.5.2 REQUIRED )
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")
include_directories( ${OpenCV_INCLUDE_DIRS} )

# creates the variable EXEC and sets it to spot-it-3d
set(EXEC spot-it-3d)

# include headers
include_directories(include)

# add libraries
add_subdirectory(lib)

# puts all .cpp files inside src to the SOURCES variable
file(GLOB SOURCES src/*.cpp)

# compiles the files defined by SOURCES to generate the executable defined by EXEC
add_executable(${EXEC} ${SOURCES})

# Link executables to libraries
target_link_libraries( ${EXEC} ${OpenCV_LIBS} )
target_link_libraries( ${EXEC} PUBLIC Hungarian)
target_link_libraries( ${EXEC} PUBLIC Json)
################################
# Definitions
################################

cmake_minimum_required(VERSION 2.8)

# project name
project( SPOT_IT_3D )

# creates the variable EXEC and sets it to spot-it-3d
set(EXEC spot-it-3d)

# puts all .cpp files inside src to the SOURCES variable
file(GLOB SOURCES src/*.cpp)

# compiles the files defined by SOURCES to generate the executable defined by EXEC
add_executable(${EXEC} ${SOURCES})

################################
# Add dependencies and libraries
################################

# pkgconfig
find_package(PkgConfig REQUIRED)
set(ENV{PKG_CONFIG_PATH})

# opencv - specify which version we want to use (e.g. 4.2.0 or 4.5.2?)
find_package( OpenCV 4 REQUIRED )
# find_package( OpenCV 4.5.2 REQUIRED )
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(${EXEC} PRIVATE ${OpenCV_LIBS})

# gstreamer
pkg_check_modules(GST REQUIRED gstreamer-1.0)
message(STATUS "Found Gstreamer version ${GST_VERSION}")
include_directories("${GST_INCLUDE_DIRS}")
target_link_libraries(${EXEC} PRIVATE ${GST_LIBRARIES})

# add custom include and library folders
include_directories(${CMAKE_SOURCE_DIR}/include)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib)

# add custom libraries
target_link_libraries(${EXEC} PUBLIC Hungarian)
target_link_libraries(${EXEC} PUBLIC Json)
target_link_libraries(${EXEC} PUBLIC ${CMAKE_SOURCE_DIR}/lib/wsrt/libsrtreceiver.a) # static libraries
target_link_libraries(${EXEC} PUBLIC ${CMAKE_SOURCE_DIR}/lib/wsrt/libwsclient.a)

# add wsrt dependent libraries
find_library(GOBJECT gobject-2.0)
target_link_libraries(${EXEC} PUBLIC ${GOBJECT})
find_library(GLIB glib-2.0)
target_link_libraries(${EXEC} PUBLIC ${GLIB})
find_library(PTHREAD pthread)
target_link_libraries(${EXEC} PUBLIC ${PTHREAD})
find_library(GSTAPP gstapp-1.0)
target_link_libraries(${EXEC} PUBLIC ${GSTAPP})
find_library(TBB tbb)
target_link_libraries(${EXEC} PUBLIC ${TBB})
find_library(SOUP soup-2.4)
target_link_libraries(${EXEC} PUBLIC ${SOUP})
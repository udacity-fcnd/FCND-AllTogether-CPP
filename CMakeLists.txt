cmake_minimum_required(VERSION 3.5)
project(FCNDSim)

# set flags
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "-Wall -O2 -Wno-deprecated")

# set path
set(Qt5Core_DIR /home/jz/Qt/5.10.0/gcc_64/lib/cmake/Qt5Core)
set(Qt5Network_DIR /home/jz/Qt/5.10.0/gcc_64/lib/cmake/Qt5Network)
set(Qt5Widgets_DIR /home/jz/Qt/5.10.0/gcc_64/lib/cmake/Qt5Widgets)

# required library
find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Widgets REQUIRED)
# /System/Library/Frameworks/GLUT.framework
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})
# /System/Library/Frameworks/OpenGL.framework
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
# find_package(GL REQUIRED)
# find_package(pthread REQUIRED)

message("Root Directory:" ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# include_directories(modules)
# include_directories(third_party)

FILE(GLOB SOURCES
        modules/*.cpp
        modules/common/*.cpp 
        modules/control/*.cpp 
        modules/drawing/*.cpp
        modules/estimation/*.cpp 
        modules/math/*.cpp
        modules/mavlinkNode/*.cpp 
        modules/planning/*.cpp 
        modules/simulation/*.cpp 
        modules/utility/*.cpp)

FILE(GLOB HEADERS
        modules/common/*.h
        modules/control/*.h 
        modules/drawing/*.h
        modules/estimation/*.h 
        modules/math/*.h
        modules/mavlinkNode/*.h
        modules/planning/*.h
        modules/simulation/*.h
        modules/utility/*.h
        third_party/matrix/*.hpp
        third_party/mavlink/*.h
        third_party/mavlink/common/*.h)

# executable
add_executable(FCNDSim
        ${SOURCES}
        ${HEADERS})

target_link_libraries(FCNDSim
        Qt5::Core
        Qt5::Network
        Qt5::Widgets
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
        pthread)

############################## test ###############################
# test astar planner
enable_testing()
FILE(GLOB test_astar
        modules/planning/unittest_AstarPlanner.cpp
        modules/common/*.h
        modules/math/*.cpp
        modules/math/*.h
        modules/utility/*.cpp
        modules/utility/*.h
        modules/planning/*.cpp
        modules/planning/*.h)
add_executable(test_astar 
        ${test_astar})
target_link_libraries(test_astar 
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES})

# test voronoi diagram
set(test_voronoi
    modules/planning/unittest_jc_voronoi.cpp
    modules/planning/jc_voronoi_wrapper.c
    modules/planning/jc_voronoi_wrapper.h
    third_party/voronoi/jc_voronoi/src/stb_wrapper.c
    third_party/voronoi/jc_voronoi/src/jc_voronoi.h
    third_party/voronoi/jc_voronoi/src/stb_image_write.h )
add_executable(test_voronoi 
    ${test_voronoi})
target_link_libraries(test_voronoi m)

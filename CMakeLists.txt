cmake_minimum_required(VERSION 3.5)
project(FCNDSim)

set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "-Wno-deprecated")

set(Qt5Core_DIR /home/jz/Qt/5.10.0/gcc_64/lib/cmake/Qt5Core)
set(Qt5Network_DIR /home/jz/Qt/5.10.0/gcc_64/lib/cmake/Qt5Network)
set(Qt5Widgets_DIR /home/jz/Qt/5.10.0/gcc_64/lib/cmake/Qt5Widgets)

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

add_executable(FCNDSim
        ${SOURCES}
        ${HEADERS}
        )

target_link_libraries(FCNDSim
        Qt5::Core
        Qt5::Network
        Qt5::Widgets
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
        pthread
        )

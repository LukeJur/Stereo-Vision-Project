cmake_minimum_required(VERSION 3.6)
project(StereoVisionProject)
set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)


SET( CAMERA_INCLUDE /opt/vrmagic/sdk-4.5.0/x64/development_kit/include)
SET( CAMERA_WRAPPER /opt/vrmagic/sdk-4.5.0/x64/development_kit/wrappers/c++)
SET( CAMERA_LIBRARY /opt/vrmagic/sdk-4.5.0/x64/development_kit/lib/libvrmusbcam2.so)
SET( SOURCE /home/luke/CLionProjects/StereoVisionProject)

## OpenCv Libs*
include_directories(${OpenCV_INCLUDE_DIRS})

## Camera drivers
include_directories(${CAMERA_INCLUDE})
include_directories(${CAMERA_WRAPPER})


set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

#find_package (Boost 1.55.0 REQUIRED COMPONENTS system filesystem)
find_package(Boost 1.53.0 COMPONENTS log log_setup thread date_time filesystem system REQUIRED)
if (Boost_FOUND)
    include_directories (SYSTEM ${Boost_INCLUDE_DIR})
endif()



add_library(vrmusbwrap shared ${CAMERA_WRAPPER}/vrmusbcamcpp.o)
target_link_libraries( vrmusbwrap ${Boost_LIBRARIES} ${CAMERA_INCLUDE})

set(SOURCE_FILES main.cpp camerahandle.cpp stereovision.h stereovision.cpp stereovision.h)
add_executable(StereoVisionProject ${SOURCE_FILES} ${CAMERA_WRAPPER}/vrmusbcamcpp.cpp stereovision.h stereovision.cpp stereovision.h)



target_link_libraries(StereoVisionProject ${OpenCV_LIBS} ${CAMERA_LIBRARY} ${Boost_LIBRARIES} ${vrmusbwrap} )


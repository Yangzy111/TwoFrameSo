cmake_minimum_required(VERSION 2.8)
project(TwoFrameSO)

#set cmake build type
IF(NOT CMAKE_BUILD_TYPE)
#   SET(CMAKE_BUILD_TYPE Debug)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#print build type
MESSAGE("Build  type: " ${CMAKE_BUILD_TYPE})

#set debug compile setting
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -g  -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O0 -g -march=native")

#release compile setting
if(CMAKE_BUILD_TYPE STREQUAL "Release")
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  -march=native ")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
endif()

# osx settting 
if(CMAKE_HOST_APPLE)
   set(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG "-Wl,-rpath,")
   set(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG_SEP ":")
   cmake_policy(SET CMP0042 NEW)
endif()

#Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#set cmake modules find path
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3  REQUIRED)

#set include file search path

# recurrence include directories
function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})               #check directory
      #   message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif()
    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*) #insert sub dirs 
    foreach(sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub}) #
        endif()
    endforeach()
endfunction()

include_sub_directories_recursively(${PROJECT_SOURCE_DIR}/include)

include_directories(
${EIGEN3_INCLUDE_DIR} 
)

#set lib link path 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#build library 

set(TwoFrame "twoframe")

FILE(GLOB_RECURSE SRC_LIST_CPP ${PROJECT_SOURCE_DIR}/src/*.cpp) 

#link_directories(${PROJECT_SOURCE_DIR}/lib/opencv)
#link_directories(${PROJECT_SOURCE_DIR}/lib/g2o)
message(${EIGEN3_INCLUDE_DIR})

add_library(${TwoFrame} SHARED ${SRC_LIST_CPP})

#link libs
target_link_libraries(${TwoFrame}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/lib/g2o/libg2o.so
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_features2d.so.3.4
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_ml.so.3.4
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_shape.so.3.4
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_video.so.3.4
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_videoio.so.3.4
#${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_core.so.3.4
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_calib3d.so.3.4.9
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_core.so.3.4.9
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_flann.so.3.4.9
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_highgui.so.3.4.9
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_imgcodecs.so.3.4.9
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_imgproc.so.3.4.9
${PROJECT_SOURCE_DIR}/lib/opencv/libopencv_xfeatures2d.so.3.4.9
)

         

#set program running path 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output)

#build exe 
add_executable(TwoFrameSO
test.cpp 
)
target_link_libraries(TwoFrameSO ${TwoFrame})

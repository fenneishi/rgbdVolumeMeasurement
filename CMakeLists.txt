
# base set
cmake_minimum_required(VERSION 2.8)
project(rgbdVolumeMeasurement)
set(CMAKE_CXX_FLAGS  "-std=c++11")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_BUILD_TYPE "Debug")

# find PCL 
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})






# find openCv
find_package (OpenCV REQUIRED)
set(OpenCv_INCLUDE_DIRS ${OpenCv_INCLUDE_DIRS} /usr/local/include/opencv2 /usr/local/include/opencv)
include_directories (${OpenCv_INCLUDE_DIRS})
MESSAGE("OpenCv_INCLUDE_DIRS:::::::::::" ${OpenCv_INCLUDE_DIRS})
MESSAGE("OpenCV_LIBS::::::::::::" ${OpenCV_LIBS})





# set EXECUTABLE_OUTPUT_PATH 
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin/)
MESSAGE("EXECUTABLE_OUTPUT_PATH is :::::::::::" ${EXECUTABLE_OUTPUT_PATH})



# set src_path
set(src_path ./src)

# add_executable:bin2frames
add_executable(bin2frames ${src_path}/bin2frames.cpp ${src_path}/CameraReader.cpp )
target_link_libraries(bin2frames ${OpenCV_LIBS})



# add_executable:stl2pcd
add_executable(ply2pcd ${src_path}/stl2pcd.cpp)
target_link_libraries(ply2pcd ${PCL_LIBRARIES})



# add_executable:segment
add_executable(segment ${src_path}/segment.cpp)
target_link_libraries(segment ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_SOURCE_DIR}/bin/libgoodscanct.so)



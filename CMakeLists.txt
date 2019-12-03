project(FeatureExtractor)
cmake_minimum_required(VERSION 2.8)

#CGAL
find_package( CGAL REQUIRED COMPONENTS)
include( ${CGAL_USE_FILE})


#PCL
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
set(CMAKE_MODULE_PATH "/home/huifang/huifang/local/share/pcl-1.7/" )
find_package(PCL 1.7.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include(opencv.cmake)   # OPENCV_LIBRARIES


aux_source_directory(. SRC_LIST)
FILE(GLOB_RECURSE INC_ALL "./seg/*.h" "./*.h")
SET(SRC_CPP  seg/multiFrameMerge.cpp seg/objPartition.cpp seg/surfaceSegment.cpp segmentercy.cpp)



#add_executable(${PROJECT_NAME} ${SRC_CPP}  ${INC_ALL} main.cpp featureextractor.cpp easyViewer.cpp easysaver.cpp)
add_executable(${PROJECT_NAME} ${SRC_CPP}  ${INC_ALL} main.cpp featureextractor.cpp easyViewer.cpp easysaver.cpp
fastsegmentation.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})


cmake_minimum_required(VERSION 3.5)
project(robotic_multiview_registration)


# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PCL_DIR "/usr/local/include/pcl-1.10")
# set(VTK_DIR "/usr/include/vtk-9.3")
# PCL 라이브러리 찾기
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(VTK REQUIRED)
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)
#PointCloudLibrary
include_directories(${PCL_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS} ${GMP_INCLUDE_DIR} ${MPFR_INCLUDE_DIR}${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#include_directories("/usr/include/opencv4/")
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
#robotic_multiview_registration    
add_executable (robotic_multiview_registration src/robotic_multiview_registration.cpp dependencies/universalRobotsKinematics/universalRobotsKinematics.cpp dependencies/pointcloud/preprocess.cpp dependencies/CGAL/Meshing.cpp)
target_link_libraries(robotic_multiview_registration ${PCL_LIBRARIES} ${OpenCV_LIBS} ${VTK_LIBRARIES} ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} yaml-cpp)
target_include_directories(robotic_multiview_registration PRIVATE
    dependencies/universalRobotsKinematics
    dependencies/pointcloud
    dependencies/CGAL
)


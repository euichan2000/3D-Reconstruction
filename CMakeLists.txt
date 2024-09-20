cmake_minimum_required(VERSION 3.0.2)
project(3d_reconstruction)

# ROS 필수 패키지 찾기 (roscpp, std_msgs)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
)


set(CMAKE_BUILD_TYPE Release)
# VTK 버전 설정
set(VTK_DIR "/usr/lib/cmake/vtk-7.1")  # VTK 7.1 설치 경로로 설정
set(PCL_DIR "/usr/local/share/pcl-1.10")

# 외부 라이브러리 찾기 (PCL, OpenCV, CGAL, VTK, GMP, MPFR)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(VTK REQUIRED)
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)

###################################
## catkin 패키지 설정 ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS PCL OpenCV CGAL VTK GMP MPFR
)

###################################
## include 디렉토리 설정 ##
###################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${GMP_INCLUDE_DIR}
  ${MPFR_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

###################################
## 빌드 설정 ##
###################################

# robotic_multiview_registration 노드 빌드
add_executable(robotic_multiview_registration
  src/main.cpp
  src/universalRobotsKinematics.cpp
  src/preprocess.cpp
  src/Meshing.cpp
)

# 타겟 링크 라이브러리 설정 (PCL, OpenCV, CGAL 등과 연결)
target_link_libraries(robotic_multiview_registration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CGAL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${GMP_LIBRARIES}
  ${MPFR_LIBRARIES}
  yaml-cpp
)
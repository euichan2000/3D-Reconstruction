
cmake_minimum_required(VERSION 3.5)
project(PCL_Registration)


# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.4 REQUIRED)
find_package (OpenCV REQUIRED)

#PointCloudLibrary
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#OpenCVLibrary
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories("/usr/include/opencv4/")
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# #min_cut_Segmentation
# add_executable (min_cut_segmentation src/min_cut_segmentation.cpp dependencies/pointcloud/preprocess.cpp)
# target_link_libraries (min_cut_segmentation ${PCL_LIBRARIES})
# target_include_directories(min_cut_segmentation PRIVATE
#     dependencies/pointcloud/)
# #euclidean_Segmentation
# add_executable (euclidean_segmentation src/euclidean_segmentation.cpp)
# target_link_libraries (euclidean_segmentation ${PCL_LIBRARIES})
#charuco_Segmentation
add_executable(charuco_segmentation src/charuco_segmentation.cpp dependencies/universalRobotsKinematics/src/universalRobotsKinematics.cpp dependencies/mathLib/src/mathLib.cpp dependencies/pointcloud/preprocess.cpp)
target_link_libraries(charuco_segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
target_include_directories(charuco_segmentation PRIVATE
    dependencies/universalRobotsKinematics/src
    dependencies/mathLib/src
    dependencies/pointcloud)

#robotic_multiview_registration    
add_executable (robotic_multiview_registration src/robotic_multiview_registration.cpp dependencies/universalRobotsKinematics/src/universalRobotsKinematics.cpp dependencies/mathLib/src/mathLib.cpp dependencies/pointcloud/preprocess.cpp)
target_link_libraries(robotic_multiview_registration ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
target_include_directories(robotic_multiview_registration PRIVATE
    dependencies/universalRobotsKinematics/src
    dependencies/mathLib/src
    dependencies/pointcloud/
    
)
    




#Reconstruction
# add_executable (registration src/registration.cpp)
# target_link_libraries (registration ${PCL_LIBRARIES})

# add_executable(normal_distributions_transform normal_distributions_transform.cpp)
# target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})

# add_executable (min_cut_segmentation min_cut_segmentation.cpp)
# target_link_libraries (min_cut_segmentation ${PCL_LIBRARIES})

# add_executable (outlier_remove outlier_remove.cpp)
# target_link_libraries (outlier_remove ${PCL_LIBRARIES})

# add_executable (downsample downsample.cpp)
# target_link_libraries (downsample ${PCL_LIBRARIES})

# add_executable (triangulation triangulation.cpp)
# target_link_libraries (triangulation ${PCL_LIBRARIES})

# add_executable (transformation_icp transformation_icp.cpp)
# target_link_libraries (transformation_icp ${PCL_LIBRARIES})

# add_executable (transformation_gicp transformation_gicp.cpp)
# target_link_libraries (transformation_gicp ${PCL_LIBRARIES})



# add_executable(SAC_IA_registration SAC_IA_registration.cpp)
# target_link_libraries(SAC_IA_registration ${PCL_LIBRARIES})

# add_executable(B_spline B_spline.cpp)
# target_link_libraries(B_spline ${PCL_LIBRARIES})
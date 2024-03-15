
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(PCL_Registration)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Segmentation
# add_executable (euclidean_segmentation euclidean_segmentation.cpp)
# target_link_libraries (euclidean_segmentation ${PCL_LIBRARIES})
#Reconstruction
add_executable (registration registration.cpp)
target_link_libraries (registration ${PCL_LIBRARIES})

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

add_executable (test test.cpp)
target_link_libraries (test ${PCL_LIBRARIES})

# add_executable(SAC_IA_registration SAC_IA_registration.cpp)
# target_link_libraries(SAC_IA_registration ${PCL_LIBRARIES})

# add_executable(B_spline B_spline.cpp)
# target_link_libraries(B_spline ${PCL_LIBRARIES})
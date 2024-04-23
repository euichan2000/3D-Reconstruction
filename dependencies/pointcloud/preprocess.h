#pragma once
#include <vector>
#include <iomanip> // for setw, setfill
#include <filesystem>
#include <iostream>
#include <pcl/pcl_macros.h>
//#include <pcl/memory.h> // for make_shared
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace pointcloudpreprocess
{
    class pre
    {
    public:
        // void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models);
        PointCloud::Ptr downsampling(const PointCloud::Ptr cloud_src, float downsampleparam);
        PointCloud::Ptr statistical_outlier_remove(const PointCloud::Ptr cloud_src, float filterparam);
        PointCloud::Ptr radius_outlier_remove(const PointCloud::Ptr cloud_src);
        PointCloud::Ptr calibrate(PointCloud::Ptr cloud_src, Eigen::Matrix4f &base2tcp, Eigen::Matrix4f &tcp2cam);
        PointCloud::Ptr mincutsegmentation(const PointCloud::Ptr cloud_src, float radius, float x, float y, float z);
        PointCloud::Ptr charucosegmentation(const PointCloud::Ptr cloud_src, float min_pt[], float max_pt[]);
        void visualizePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadData(const std::vector<std::string> &file_paths);
        Eigen::Matrix4f fineICP(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt);
        Eigen::Matrix4f fineICPwithNormals(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt);
        Eigen::Matrix4f finetrICP(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt);
    };

}

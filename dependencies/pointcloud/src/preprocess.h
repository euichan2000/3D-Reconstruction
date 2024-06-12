#pragma once
#include <vector>
#include <iomanip> // for setw, setfill
#include <filesystem>
#include <iostream>
#include <pcl/pcl_macros.h>
#include <boost/filesystem.hpp>
// #include <pcl/memory.h> // for make_shared
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
        pcl::PointCloud<pcl::PointNormal> smoothing(const PointCloud::Ptr cloud_src, const float &smoothing_radius);
        PointCloud::Ptr downsampling(const PointCloud::Ptr cloud_src, const float &downsampleparam);
        PointCloud::Ptr statistical_outlier_remove(const PointCloud::Ptr cloud_src, const int &sor_mean, const double &sor_thresh);
        PointCloud::Ptr radius_outlier_remove(const PointCloud::Ptr cloud_src, const double &ror_radius, const int &ror_neighbor);
        PointCloud::Ptr dynamic_statistic_outlier_remove(const PointCloud::Ptr cloud_src, const int &mean_k, const float &std_mul, const float &range_mul,bool negative);
        PointCloud::Ptr euclidean_outlier_remove(const PointCloud::Ptr cloud_src);
        PointCloud::Ptr calibrate(const PointCloud::Ptr cloud_src, const Eigen::Matrix4f base2tcp, const Eigen::Matrix4f tcp2cam);
        PointCloud::Ptr mincutsegmentation(const PointCloud::Ptr cloud_src, float radius, float x, float y, float z);
        PointCloud::Ptr charucosegmentation(const PointCloud::Ptr cloud_src, float min_pt[], float max_pt[]);
        void visualizePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadData(const std::string &file_paths);
        Eigen::Matrix4f fineICP(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt);
        pcl::PointCloud<pcl::PointXYZ>::Ptr fineICPwithNormals(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt);
        pcl::PointCloud<pcl::PointXYZ>::Ptr finetrICP(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt);
        pcl::PointCloud<pcl::PointXYZ>::Ptr RecursiveRegistration(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_src);
        pcl::PointCloud<pcl::PointXYZ>::Ptr PointNormal2PointXYZ(const pcl::PointCloud<pcl::PointNormal> &cloud_src);
        pcl::PointCloud<pcl::PointXYZ>::Ptr addPoint(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_src);
    };

}

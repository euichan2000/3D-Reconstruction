#pragma once
#include <vector>
#include <iomanip> // for setw, setfill
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <pcl/pcl_macros.h>
#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

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
        PointCloud::Ptr calibrate(const PointCloud::Ptr cloud_src, const Eigen::Matrix4f base2tcp, const Eigen::Matrix4f tcp2cam);
        PointCloud::Ptr calibratemarker2base(const PointCloud::Ptr cloud_src);
        PointCloud::Ptr charucosegmentation(const PointCloud::Ptr cloud_src, float min_pt[], float max_pt[]);
        void visualizePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_src);
        pcl::PointCloud<pcl::PointXYZ>::Ptr PointNormal2PointXYZ(const pcl::PointCloud<pcl::PointNormal> &cloud_src);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadData(const std::string &file_paths);
        pcl::PointCloud<pcl::PointXYZ>::Ptr addPoint(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_src);
    };

}

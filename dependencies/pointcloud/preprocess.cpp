#include <vector>

#include <pcl/pcl_macros.h>
#include <pcl/memory.h> // for make_shared
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "preprocess.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr pointcloudpreprocess::pre::downsampling(const PointCloud::Ptr cloud_src, int number)
{
    PointCloud::Ptr cloud_tgt(new PointCloud);
    if (cloud_src->width * cloud_src->height > number)
    {
        // Create the filtering object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_src);
        sor.setLeafSize(0.005f, 0.005f, 0.005f);
        sor.filter(*cloud_tgt);

        std::cerr << "PointCloud after filtering: " << cloud_tgt->width * cloud_tgt->height
                  << " data points (" << getFieldsList(*cloud_tgt) << ")." << std::endl;
    }
    else
    {
        std::cerr << "Skipping filtering and downsampling. Point cloud has fewer than" << number << " points." << std::endl;
        *cloud_tgt = *cloud_src;
    }

    return cloud_tgt;
}

PointCloud::Ptr pointcloudpreprocess::pre::outlier_remove(const PointCloud::Ptr cloud_src)
{
    PointCloud::Ptr cloud_tgt(new PointCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_src);
    sor.setMeanK(16);
    sor.setStddevMulThresh(0.05);
    sor.filter(*cloud_tgt);

    return cloud_tgt;
}

PointCloud::Ptr pointcloudpreprocess::pre::calibrate(PointCloud::Ptr cloud_src, Eigen::Matrix4f &base2tcp, Eigen::Matrix4f &tcp2cam)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tcp2cam(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base2cam(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_src, *cloud_tcp2cam, tcp2cam);
    pcl::transformPointCloud(*cloud_tcp2cam, *cloud_base2cam, base2tcp);
    return cloud_base2cam;
}
// plane segmentation + min-cut segmentation
PointCloud::Ptr pointcloudpreprocess::pre::mincutsegmentation(const PointCloud::Ptr cloud_src, float radius, float x = 0, float y = 0, float z = 0.2)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *cloud_src;

    //////////////plane segementation//////////////////
    pcl::SACSegmentation<pcl::PointXYZ> SACseg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDWriter writer;
    SACseg.setOptimizeCoefficients(true);
    SACseg.setModelType(pcl::SACMODEL_PLANE);
    SACseg.setMethodType(pcl::SAC_RANSAC);
    SACseg.setMaxIterations(100);
    SACseg.setDistanceThreshold(0.02);

    // Segment the largest planar component from the remaining cloud
    SACseg.setInputCloud(cloud);
    SACseg.segment(*inliers, *coefficients);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract1;
    extract1.setInputCloud(cloud);
    extract1.setIndices(inliers);
    extract1.setNegative(false);

    // Get the points associated with the planar surface
    extract1.filter(*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract1.setNegative(true);
    extract1.filter(*cloud_f);
    *cloud_plane_filtered = *cloud_f;

    ///////////////////mincut segmentation/////////////////////

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_plane_filtered, *indices);

    pcl::MinCutSegmentation<pcl::PointXYZ> Minseg;
    Minseg.setInputCloud(cloud_plane_filtered);
    Minseg.setIndices(indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ point;
    point.x = x;
    point.y = y;
    point.z = z;
    foreground_points->points.push_back(point);
    Minseg.setForegroundPoints(foreground_points);

    Minseg.setSigma(0.25);
    Minseg.setRadius(radius);
    Minseg.setNumberOfNeighbours(14);
    Minseg.setSourceWeight(0.8);

    std::vector<pcl::PointIndices> clusters;
    Minseg.extract(clusters);

    // 클러스터된 포인트 클라우드 중 첫 번째 클러스터를 저장할 변수 정의
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 클러스터된 포인트 클라우드 중 첫 번째 클러스터의 인덱스를 가져옴
    pcl::PointIndices cloud_filtered_indices = clusters[1];

    // 첫 번째 클러스터의 인덱스를 이용하여 클러스터된 포인트를 가져옴
    for (size_t i = 0; i < cloud_filtered_indices.indices.size(); ++i)
    {
        int index = cloud_filtered_indices.indices[i];
        cloud_filtered->points.push_back(cloud_plane_filtered->points[index]);
    }

    // std::cout << "Maximum flow is " << Minseg.getMaxFlow() << std::endl;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = Minseg.getColoredCloud();
    // pcl::visualization::CloudViewer viewer("Cluster viewer");
    // viewer.showCloud(cloud_filtered);
    return cloud_filtered;
}

void pointcloudpreprocess::pre::visualizePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds)
{
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Dark grey background

    // Add each point cloud to the viewer with custom color
    for (size_t i = 0; i < clouds.size(); ++i)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(clouds[i], 255, 255, 255); // White color
        std::string cloud_name = "cloud_" + std::to_string(i);
        viewer.addPointCloud(clouds[i], cloud_color_handler, cloud_name);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);
    }

    viewer.addCoordinateSystem(1.0, "coordinate_system", 0); // Add coordinate system
    viewer.setPosition(800, 400);                            // Set viewer position

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointcloudpreprocess::pre::loadData(const std::vector<std::string> &file_paths)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

    for (const auto &file_path : file_paths)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *temp_cloud) == -1)
        {
            std::cerr << "Couldn't read file " << file_path << std::endl;
            continue;
        }
        std::cout << "Loaded " << temp_cloud->width * temp_cloud->height << " data points from " << file_path << std::endl;
        clouds.push_back(temp_cloud);
    }

    return clouds;
}


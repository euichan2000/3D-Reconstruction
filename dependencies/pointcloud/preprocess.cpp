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
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
    MyPointRepresentation()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray(const PointNormalT &p, float *out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};
/*
void pointcloudpreprocess::pre::loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models)
{
    std::string extension(".pcd");
    // Suppose the first argument is the actual test model
    for (int i = 1; i < argc; i++)
    {
        std::string fname = std::string(argv[i]);
        // Needs to be at least 5: .plot
        if (fname.size() <= extension.size())
            continue;

        std::transform(fname.begin(), fname.end(), fname.begin(), (int (*)(int))tolower);

        // check that the argument is a pcd file
        if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
        {
            // Load the cloud and saves it into the global list of models
            PCD m;
            m.f_name = argv[i];
            pcl::io::loadPCDFile(argv[i], *m.cloud);
            // remove NAN points from the cloud
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

            models.push_back(m);
        }
    }
}
*/

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
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
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

PointCloud::Ptr pointcloudpreprocess::pre::segmentation(const PointCloud::Ptr cloud_src)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *cloud_src;

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);
    int nr_points = (int)cloud_filtered->size();
    while (cloud_filtered->size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int cluster_count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_clusters(new pcl::PointCloud<pcl::PointXYZ>);
    float min_distance = std::numeric_limits<float>::max();
    pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_cloud;
    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        }
        cluster_count++;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        float distance = centroid.head<3>().norm();

        // cout << "Distance between centroid and origin: " << distance << endl;
        //  최소 거리인 경우 nearest_cloud를 업데이트합니다.
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_cloud = cloud_cluster;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final = nearest_cloud;
    return cloud_final;
}

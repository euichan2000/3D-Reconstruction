#include <pcl/ModelCoefficients.h>
#include <pcl/point_representation.h>
#include <filesystem>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

// convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct PCD
{
    PointCloud::Ptr cloud; // Ptr type 변수: cloud
    std::string f_name;    // string type 변수:  f_name

    PCD() : cloud(new PointCloud){}; // PCD 함수 생성
};

void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models)
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

int main(int argc, char **argv)
{
    std::vector<PCD, Eigen::aligned_allocator<PCD>> data;
    loadData(argc, argv, data);
    if (data.empty())
    {
        PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]\n", argv[0]);
        PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc\n");
        return (-1);
    }

    PCL_INFO("Loaded %d datasets.\n", (int)data.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_ptr, cloud_target_ptr;
    cloud_source_ptr = data[0].cloud;
    cloud_target_ptr = data[1].cloud;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method_ptr = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_source = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_target = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

    norm_est.setInputCloud(cloud_source_ptr);
    norm_est.setSearchMethod(search_method_ptr);
    norm_est.setRadiusSearch(0.02);
    norm_est.compute(*normals_source);

    norm_est.setInputCloud(cloud_target_ptr);
    norm_est.setSearchMethod(search_method_ptr);
    norm_est.setRadiusSearch(0.02);
    norm_est.compute(*normals_target);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_source = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_target = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;

    fpfh_est.setInputCloud(cloud_source_ptr);
    fpfh_est.setInputNormals(normals_source);
    fpfh_est.setSearchMethod(search_method_ptr);
    fpfh_est.setRadiusSearch(0.02);
    fpfh_est.compute(*features_source);

    fpfh_est.setInputCloud(cloud_target_ptr);
    fpfh_est.setInputNormals(normals_target);
    fpfh_est.setSearchMethod(search_method_ptr);
    fpfh_est.setRadiusSearch(0.02);
    fpfh_est.compute(*features_target);

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    Eigen::Matrix4f final_transformation;
    sac_ia.setInputSource(cloud_target_ptr);
    sac_ia.setSourceFeatures(features_target);
    sac_ia.setInputTarget(cloud_source_ptr);
    sac_ia.setTargetFeatures(features_source);
    sac_ia.setMaximumIterations(1000);
    sac_ia.setMinSampleDistance(0.01f);
    sac_ia.setMaxCorrespondenceDistance(0.2);
    pcl::PointCloud<pcl::PointXYZ> finalcloud;
    sac_ia.align(finalcloud);
    sac_ia.getCorrespondenceRandomness();
    Eigen::Matrix4f init_transform = sac_ia.getFinalTransformation();
    pcl::transformPointCloud(*cloud_target_ptr, *cloud_target_ptr, init_transform);
    pcl::PointCloud<pcl::PointXYZ> final = *cloud_source_ptr;
    final += *cloud_target_ptr;

    std::stringstream ss;
    ss << "./registrated_pcd/result_SAC_IA_"
       << ".pcd";
    pcl::io::savePCDFile(ss.str(), final, true);
}
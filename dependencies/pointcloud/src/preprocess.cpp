
#include <vector>
#include <iostream>
#include <boost/filesystem.hpp>
#include <algorithm>
// #include <pcl/memory.h> // for make_shared
#include <pcl/pcl_macros.h>
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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/recognition/ransac_based/trimmed_icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "preprocess.h"
#include <pcl/filters/crop_box.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace fs = boost::filesystem;

pcl::PointCloud<pcl::PointNormal> pointcloudpreprocess::pre::smoothing(const PointCloud::Ptr cloud_src, const float &smoothing_radius)
{
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> cloud_tgt;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud_src);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(smoothing_radius);

    // Reconstruct
    mls.process(cloud_tgt);

    return cloud_tgt;
}

PointCloud::Ptr pointcloudpreprocess::pre::downsampling(const PointCloud::Ptr cloud_src, const float &downsampleparam)
{
    PointCloud::Ptr cloud_tgt(new PointCloud);

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud_src);
    vox.setLeafSize(downsampleparam, downsampleparam, downsampleparam);
    vox.filter(*cloud_tgt);

    // std::cerr << "PointCloud after filtering: " << cloud_tgt->width * cloud_tgt->height
    //           << " data points (" << getFieldsList(*cloud_tgt) << ")." << std::endl;

    return cloud_tgt;
}

PointCloud::Ptr pointcloudpreprocess::pre::statistical_outlier_remove(const PointCloud::Ptr cloud_src, const int &sor_mean, const double &sor_thresh)
{
    PointCloud::Ptr cloud_tgt(new PointCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_src);
    sor.setMeanK(sor_mean);
    sor.setStddevMulThresh(sor_thresh);
    sor.filter(*cloud_tgt);
    // std::cout << "statistical outlier remove success" << std::endl;
    return cloud_tgt;
}

PointCloud::Ptr pointcloudpreprocess::pre::radius_outlier_remove(const PointCloud::Ptr cloud_src, const double &ror_radius, const int &ror_neighbor)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;

    ror.setInputCloud(cloud_src);

    ror.setRadiusSearch(ror_radius);
    ror.setMinNeighborsInRadius(ror_neighbor);
    ror.setKeepOrganized(true);
    ror.filter(*cloud_filtered);
    // std::cout << "radius outlier remove success" << std::endl;
    return cloud_filtered;
}

PointCloud::Ptr pointcloudpreprocess::pre::dynamic_statistic_outlier_remove(const PointCloud::Ptr cloud_src, const int &mean_k, const float &std_mul, const float &range_mul,bool negative)
{

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointT>::Ptr cloud_negative(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<PointT> kd_tree;
    kd_tree.setInputCloud(cloud_src);

    // Allocate enough space to hold the results
    std::vector<int> pointIdxNKNSearch(mean_k);
    std::vector<float> pointNKNSquaredDistance(mean_k);
    std::vector<float> mean_distances;

    // Go over all the points and check which doesn't have enough neighbors
    // perform filtering
    for (pcl::PointCloud<PointT>::iterator it = cloud_src->begin();
         it != cloud_src->end(); ++it)
    {
        // k nearest search
        kd_tree.nearestKSearch(*it, mean_k, pointIdxNKNSearch,
                               pointNKNSquaredDistance);

        // calculate mean distance
        double dist_sum = 0;
        for (int j = 1; j < mean_k; ++j)
        {
            dist_sum += sqrt(pointNKNSquaredDistance[j]);
        }
        mean_distances.push_back(static_cast<float>(dist_sum / (mean_k - 1)));
    }

    // Estimate the mean and the standard deviation of the distance vector
    double sum = 0, sq_sum = 0;
    for (size_t i = 0; i < mean_distances.size(); ++i)
    {
        sum += mean_distances[i];
        sq_sum += mean_distances[i] * mean_distances[i];
    }
    double mean = sum / static_cast<double>(mean_distances.size());
    double variance =
        (sq_sum - sum * sum / static_cast<double>(mean_distances.size())) /
        (static_cast<double>(mean_distances.size()) - 1);
    double stddev = sqrt(variance);
    std::cout << "mean: " << mean << " var: " << variance << std::endl;
    // calculate distance threshold (PCL sor implementation)
    double distance_threshold = (mean + std_mul * stddev);
    // iterate through vector
    int i = 0;
    for (pcl::PointCloud<PointT>::iterator it = cloud_src->begin();
         it != cloud_src->end(); ++it)
    {
        // calculate distance of every point from the sensor
        float range = sqrt(pow(it->x, 2) + pow(it->y, 2) + pow(it->z, 2));
        // dynamic threshold: as a point is farther away from the sensor,
        // the threshold increases
        double dynamic_threshold = distance_threshold * range_mul * range;

        std::cout << "dynamic threshold: " << dynamic_threshold << " mean_distance" << mean_distances[i] << std::endl;
        // a distance lower than the threshold is an inlier
        if (mean_distances[i] < dynamic_threshold)
        {
            cloud_filtered->push_back(*it);
        }
        else
        {
            cloud_negative->push_back(*it);
        }
        // update iterator
        i++;
    }
    if (negative)
    {
        return (cloud_negative);
    }
    else
    {
        return cloud_filtered;
    }
}

PointCloud::Ptr pointcloudpreprocess::pre::euclidean_outlier_remove(const PointCloud::Ptr cloud_src)
{ // Create the segmentation object for the planar model and set all the parameters

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud(cloud_src);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01); // 5mm
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);

    ec.setInputCloud(cloud_src);

    ec.extract(cluster_indices);

    // Find the largest cluster
    int largest_cluster_index = 0;
    size_t largest_cluster_size = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {

        if (cluster_indices[i].indices.size() > largest_cluster_size)
        {

            largest_cluster_size = cluster_indices[i].indices.size();
            largest_cluster_index = i;
        }
    }

    // Extract the largest cluster and save it
    pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &idx : cluster_indices[largest_cluster_index].indices)
    {

        largest_cluster->push_back((*cloud_src)[idx]);
    }

    return largest_cluster;
}

PointCloud::Ptr pointcloudpreprocess::pre::calibrate(const PointCloud::Ptr cloud_src, const Eigen::Matrix4f base2tcp, const Eigen::Matrix4f tcp2cam)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tcp2cam(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base2cam(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_marker2cam(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f marker2base, base2marker;

    marker2base << 0.696142, 0.717802, 0.0111144, 0.0541231,
        -0.71635, 0.695145, -0.043272, 0.60022,
        -0.0389855, 0.0221005, 0.998364, 0.00557716,
        0, 0, 0, 1;
    pcl::transformPointCloud(*cloud_src, *cloud_tcp2cam, tcp2cam);
    pcl::transformPointCloud(*cloud_tcp2cam, *cloud_base2cam, base2tcp);
    pcl::transformPointCloud(*cloud_base2cam, *cloud_marker2cam, marker2base); // 현재는 charuco marker 기준 calibration되어있는 상태
    // std::cout << "calibrate success" << std::endl;
    return cloud_marker2cam;
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

PointCloud::Ptr pointcloudpreprocess::pre::charucosegmentation(const PointCloud::Ptr cloud_src, float min_pt[], float max_pt[])
{

    pcl::CropBox<pcl::PointXYZ> cropBoxFilter(true);
    Eigen::Matrix4f marker2base, base2marker;

    Eigen::Vector4f min_pt_vec4f(min_pt[0], min_pt[1], min_pt[2], min_pt[3]);
    Eigen::Vector4f max_pt_vec4f(max_pt[0], max_pt[1], max_pt[2], max_pt[3]);

    cropBoxFilter.setInputCloud(cloud_src);
    cropBoxFilter.setMin(min_pt_vec4f);
    cropBoxFilter.setMax(max_pt_vec4f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // 현재 포인트 클라우드에 대해 필터링을 수행합니다.
    cropBoxFilter.filter(*filtered_cloud);

    return filtered_cloud;
}

void pointcloudpreprocess::pre::visualizePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds)
{
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Dark grey background

    // Define a list of bright colors
    std::vector<std::vector<double>> bright_colors = {
        {1.0, 0.0, 0.0}, // Red
        {0.0, 1.0, 0.0}, // Green
        {0.0, 0.0, 1.0}, // Blue
        {1.0, 1.0, 0.0}, // Yellow
        {1.0, 0.0, 1.0}, // Magenta
        {0.0, 1.0, 1.0}, // Cyan
        {1.0, 0.5, 0.0}, // Orange
        {0.0, 1.0, 0.5}, // Lime
        {0.5, 1.0, 0.0}  // Chartreuse
    };

    // Add each point cloud to the viewer with a different bright color
    for (size_t i = 0; i < clouds.size(); ++i)
    {
        // std::vector<double> color = bright_colors[i % bright_colors.size()]; // Cycle through bright colors
        std::vector<double> color = bright_colors[0]; // Cycle through bright colors
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(clouds[i], color[0] * 255, color[1] * 255, color[2] * 255);
        std::string cloud_name = "cloud_" + std::to_string(i);
        viewer.addPointCloud(clouds[i], cloud_color_handler, cloud_name);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);
    }

    viewer.addCoordinateSystem(0.5, "coordinate_system", 0); // Add coordinate system
    viewer.setPosition(800, 400);                            // Set viewer position

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointcloudpreprocess::pre::loadData(const std::string &file_paths)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    fs::path directory(file_paths);
    fs::directory_iterator end_iter;
    if (fs::exists(directory) && fs::is_directory(directory))
    {
        std::vector<std::string> fileNames;
        for (fs::directory_iterator dir_iter(directory); dir_iter != end_iter; ++dir_iter)
        {
            if (fs::is_regular_file(dir_iter->status()) && dir_iter->path().extension() == ".pcd")
            {
                fileNames.push_back(dir_iter->path().filename().string());
            }
        }
        std::sort(fileNames.begin(), fileNames.end());
        // 정렬된 파일 순서대로 읽어서 clouds에 저장
        for (const auto &fileName : fileNames)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>((directory / fileName).string(), *cloud) == 0)
            {
                clouds.push_back(cloud);
                std::cout << fileName << std::endl;
            }
        }
    }
    else
    {
        std::cerr << "Invalid folder path." << std::endl;
    }

    return clouds;
}

Eigen::Matrix4f pointcloudpreprocess::pre::fineICP(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt)
{
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);

    src = cloud_src;
    tgt = cloud_tgt;

    // Align
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setTransformationEpsilon(1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 1cm
    // Note: adjust this based on the size of your datasets
    icp.setMaxCorrespondenceDistance(0.03);
    // Set the point representation

    icp.setInputSource(src);
    icp.setInputTarget(tgt);

    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloud::Ptr icp_result(new PointCloud);
    icp.setMaximumIterations(1000);

    icp.align(*icp_result);

    // accumulate transformation between each Iteration
    Ti = icp.getFinalTransformation();
    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    double fine_score = icp.getFitnessScore();
    // std::cout << "converge score: " << score << std::endl;
    //
    //  Get the transformation from target to source
    targetToSource = Ti.inverse();

    return targetToSource;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudpreprocess::pre::fineICPwithNormals(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt)
{

    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new PointCloud);
    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> icp;
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;

    src = cloud_src;
    tgt = cloud_tgt;

    norm_est.setSearchMethod(tree);

    norm_est.setKSearch(100);

    norm_est.setInputCloud(src);

    norm_est.compute(*points_with_normals_src);

    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);

    norm_est.compute(*points_with_normals_tgt);

    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    // Align

    icp.setTransformationEpsilon(1e-6);

    // Set the maximum distance between two correspondences (src<->tgt) to 1cm
    // Note: adjust this based on the size of your datasets
    icp.setMaxCorrespondenceDistance(0.03);

    // Set the point representation
    icp.setInputSource(points_with_normals_src);

    icp.setInputTarget(points_with_normals_tgt);
    PointCloudWithNormals::Ptr icp_result = points_with_normals_src;
    // Run the same optimization in a loop and visualize the results
    icp.setMaximumIterations(500);

    icp.align(*icp_result);

    // accumulate transformation between each Iteration
    Ti = icp.getFinalTransformation();

    // double score = icp.getFitnessScore();
    // bool is_converged = icp.hasConverged();
    // std::cout << "converge score: " << score << std::endl;
    // //
    // // Get the transformation from target to source
    targetToSource = Ti.inverse();

    pcl::transformPointCloud(*tgt, *cloud_result, targetToSource);

    *cloud_result += *src;

    return cloud_result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudpreprocess::pre::finetrICP(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt)
{
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new PointCloud);
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
    pcl::recognition::TrimmedICP<pcl::PointXYZ, float> trimmed_icp;
    src = cloud_src;
    tgt = cloud_tgt;

    if (src->size() > 100000)
    {
        src = downsampling(src, 0.01);
    }
    if (tgt->size() > 100000)
    {
        tgt = downsampling(tgt, 0.01);
    }

    trimmed_icp.init(tgt);

    std::cout << "align pointcloud A: " << src->size() << " with pointcloud B: " << tgt->size() << std::endl;

    trimmed_icp.setNewToOldEnergyRatio(0.9); // Set the ratio of new energy to old energy

    // Align the source point cloud to the target point cloud
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f targetToSource = Eigen::Matrix4f::Identity(); // Initialize with identity matrix

    trimmed_icp.align(*src, 0.9 * src->size(), transformation);

    targetToSource = transformation.inverse();

    pcl::transformPointCloud(*tgt, *cloud_result, targetToSource);

    *cloud_result += *src;
    std::cout << "result pointcloud size: " << cloud_result->size() << std::endl;
    // double fine_score=trimmed_icp.getFitnessScore();
    // std::cout << "converge score: " << score << std::endl;

    return cloud_result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudpreprocess::pre::RecursiveRegistration(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_src)
{
    if (clouds_src.size() <= 1)
    {
        return clouds_src[0];
    }
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_result;

    for (int i = 0; i < clouds_src.size() - 1; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        result_cloud = finetrICP(clouds_src[i], clouds_src[i + 1]);
        clouds_result.push_back(result_cloud);
        std::cout << "--------------------------" << std::endl;
    }
    return RecursiveRegistration(clouds_result);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudpreprocess::pre::PointNormal2PointXYZ(const pcl::PointCloud<pcl::PointNormal> &cloud_src)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud_src.points.size(); ++i)
    {
        const pcl::PointNormal &mls_pt = cloud_src.points[i];
        pcl::PointXYZ pt(mls_pt.x, mls_pt.y, mls_pt.z);
        output_cloud->push_back(pt);
    }

    return output_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudpreprocess::pre::addPoint(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_src)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < clouds_src.size(); ++i)
    {
        *result_cloud += *clouds_src[i];
    }
    return result_cloud;
}
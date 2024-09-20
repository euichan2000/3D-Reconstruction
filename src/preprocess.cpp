#include "preprocess.h"

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

PointCloud::Ptr pointcloudpreprocess::pre::calibrate(const PointCloud::Ptr cloud_src, const Eigen::Matrix4f base2tcp, const Eigen::Matrix4f tcp2cam)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tcp2cam(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base2cam(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_marker2cam(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f marker2base;

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

PointCloud::Ptr pointcloudpreprocess::pre::calibratemarker2base(const PointCloud::Ptr cloud_src)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f marker2base, base2marker;

    marker2base << 0.696142, 0.717802, 0.0111144, 0.0541231,
        -0.71635, 0.695145, -0.043272, 0.60022,
        -0.0389855, 0.0221005, 0.998364, 0.00557716,
        0, 0, 0, 1;
    base2marker = marker2base.inverse();

    pcl::transformPointCloud(*cloud_src, *cloud_tgt, base2marker);
    return cloud_tgt;
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

void pointcloudpreprocess::pre::visualizePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds_src)
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
    for (size_t i = 0; i < clouds_src.size(); ++i)
    {
        // std::vector<double> color = bright_colors[i % bright_colors.size()]; // Cycle through bright colors
        std::vector<double> color = bright_colors[0]; // Cycle through bright colors
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(clouds_src[i], color[0] * 255, color[1] * 255, color[2] * 255);
        std::string cloud_name = "cloud_" + std::to_string(i);
        viewer.addPointCloud(clouds_src[i], cloud_color_handler, cloud_name);
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
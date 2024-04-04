#include "universalRobotsKinematics.h"
#include <chrono>     // measure operation time
#include <filesystem> //filesave
#include <vector>
#include "preprocess.h"
#include <pcl/filters/crop_box.h>

// 1. Calculate base2marker Transformation matrix
// 2. Pointcloud from marker = (TF matrix from base joint to marker) *(Pointcloud from base joint)
// 3. cut Pointcloud by Region
// 4. Re-transform to base joint coordinate

int main(int argc, char **argv)
{
    Eigen::Matrix4f base2tcp, tcp2cam, base2cam, cam2marker, marker2cam, base2marker, marker2base; // base2marker: calculation using another code
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> joint_based_clouds;                           // base coordinate clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> marker_based_clouds;                          // marker coordinate clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> marker_based_cut_clouds;                      // cut clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> joint_based_cut_clouds;                       // cut clouds

    std::vector<std::string> file_paths;
    universalRobots::UR robot;
    pointcloudpreprocess::pre pre;
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter(true);
    Eigen::Vector4f min_pt(0.0f, 0.0f, 0.0f, 1.0f);
    Eigen::Vector4f max_pt(1.0f, 1.0f, 1.0f, 1.0f);
    float thetaX = 0;
    float thetaY = 0;
    float thetaZ = 45 * M_PI / 180;
    float X = 0;
    float Y = 0;
    float Z = 0.0804;

    float scence1[6] = {-60.97, -79.39, -113.83, -73.49, 113.49, 99.92};

    for (int i = 1; i < argc; ++i)
    {
        file_paths.push_back(argv[i]);
    }
    joint_based_clouds = pre.loadData(file_paths);

    base2tcp = robot.forwardKinematics(scence1);
    tcp2cam << cos(thetaZ) * cos(thetaY), cos(thetaZ) * sin(thetaY) * sin(thetaX) - sin(thetaZ) * cos(thetaX), cos(thetaZ) * sin(thetaY) * cos(thetaX) + sin(thetaZ) * sin(thetaX), X,
        sin(thetaZ) * cos(thetaY), sin(thetaZ) * sin(thetaY) * sin(thetaX) + cos(thetaZ) * cos(thetaX), sin(thetaZ) * sin(thetaY) * cos(thetaX) - cos(thetaZ) * sin(thetaX), Y,
        -sin(thetaY), cos(thetaY) * sin(thetaX), cos(thetaY) * cos(thetaX), Z,
        0, 0, 0, 1;
    base2cam = base2tcp * tcp2cam;
    cam2marker = robot.calccam2marker("../image/3.png");
    marker2cam = cam2marker.inverse();
    base2marker = base2cam * cam2marker;
    marker2base = base2marker.inverse(); // marker to base matrix
    std::cout << "marker2cam\n"
              << marker2cam << std::endl;
    std::cout << "cam2marker\n"
              << cam2marker << std::endl;

    std::cout << "marker2base\n"
              << marker2base << std::endl;
    std::cout << "base2marker\n"
              << base2marker << std::endl;

    marker_based_clouds.resize(joint_based_clouds.size());
    for (int i = 0; i < joint_based_clouds.size(); ++i) // save marker_based_clouds
    {
        marker_based_clouds[i] = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*joint_based_clouds[i], *marker_based_clouds[i], marker2base);
    }

    marker_based_cut_clouds.resize(marker_based_clouds.size());

    // for (int i = 0; i < marker_based_clouds.size(); i++) // cut selected Region
    // {
    //     marker_based_cut_clouds[i] = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    //     cropBoxFilter.setInputCloud(marker_based_clouds[i]);
    //     cropBoxFilter.setMin(min_pt);
    //     cropBoxFilter.setMax(max_pt);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    //     // 현재 포인트 클라우드에 대해 필터링을 수행합니다.
    //     cropBoxFilter.filter(*filtered_cloud);
    //     // pcl::visualization::PCLVisualizer viewer("Filtered Point Cloud");

    //     // 시각화 윈도우 설정
    //     // viewer.setBackgroundColor(0.0, 0.0, 0.0);
    //     // viewer.addCoordinateSystem(0.1);
    //     // viewer.initCameraParameters();

    //     // // 입력된 포인트 클라우드를 시각화 객체에 추가
    //     // viewer.addPointCloud(filtered_cloud, "cloud");

    //     // // 시각화 시작
    //     // while (!viewer.wasStopped())
    //     // {
    //     //     viewer.spinOnce();
    //     // }

    //     // 필터링된 결과를 marker_based_cut_clouds 벡터에 추가합니다.
    //     marker_based_cut_clouds.push_back(filtered_cloud);

    // }
    // for (int i = 0; i < marker_based_cut_clouds.size(); i++) // transform to base coordinate system
    // {
    //     pcl::transformPointCloud(*marker_based_cut_clouds[i], *joint_based_cut_clouds[i], base2marker);
    // }

    pre.visualizePointClouds(joint_based_cut_clouds);

    return 0;
}
/*Segmentation+OutlierRemove+Downsampling+Registration*/
#include <chrono>     // measure operation time
#include <filesystem> //filesave
#include <vector>
#include "preprocess.h"
#include "universalRobotsKinematics.h"
#include "Meshing.h"
#include <ros/ros.h>
#include <typeinfo>
using namespace std;

universalRobots::UR robot;     // universalRobots::UR 클래스의 객체 생성
pointcloudpreprocess::pre pre; // pointcloud 관련 클래스의 객체 생성
CGAL::Meshing mesh;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "3d_reconstruction_node");

  chrono::system_clock::time_point t_start = chrono::system_clock::now(); // start measuring registration time

  
  std::vector<Eigen::Matrix4f> base2tcp;                 // base to tcp TF Matrix
  Eigen::Matrix4f tcp2cam = Eigen::Matrix4f::Identity(); // TCP to Camera TF Matrix
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, downsampled_clouds, segmented_clouds, calibrated_clouds, radius_outlier_removed_clouds, statistical_outlier_removed_clouds, registrated_clouds, final_clouds, smoothed_clouds, robot_base_calibrated_clouds;
  pcl::PointCloud<pcl::PointNormal> smoothed_cloud_withNormal;
  pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 최종 저장용 클라우드
  pcl::PointCloud<pcl::PointXYZ>::Ptr calibrated_cloud, segmented_cloud, statistical_outlier_removed_cloud, radius_outlier_removed_cloud, result_cloud, downsampled_cloud;

  float thetaX, thetaY, thetaZ, X, Y, Z, downsampleparam;
  double sor_thresh, ror_radius;
  float minrange[4], maxrange[4], smoothing_radius;
  int n, sor_mean, ror_neighbor;
  const double relative_alpha = std::stod(argv[2]);
  const double relative_offset = std::stod(argv[3]);
  // pcd Load
  std::string folderPath = "../pcd";
  clouds = pre.loadData(folderPath);
  float scene[(int)(clouds.size())][6];

  robot.loadYAML("../reconstruction.yaml", thetaX, thetaY, thetaZ, X, Y, Z, scene, n, minrange, maxrange, downsampleparam, sor_mean, sor_thresh, ror_radius, ror_neighbor, smoothing_radius);

  tcp2cam << cos(thetaZ) * cos(thetaY), cos(thetaZ) * sin(thetaY) * sin(thetaX) - sin(thetaZ) * cos(thetaX), cos(thetaZ) * sin(thetaY) * cos(thetaX) + sin(thetaZ) * sin(thetaX), X,
      sin(thetaZ) * cos(thetaY), sin(thetaZ) * sin(thetaY) * sin(thetaX) + cos(thetaZ) * cos(thetaX), sin(thetaZ) * sin(thetaY) * cos(thetaX) - cos(thetaZ) * sin(thetaX), Y,
      -sin(thetaY), cos(thetaY) * sin(thetaX), cos(thetaY) * cos(thetaX), Z,
      0, 0, 0, 1;

  // forward kinematics 계산
  for (int i = 0; i < clouds.size(); ++i)
  {
    base2tcp.push_back(robot.forwardKinematics(scene[i]));
  }

  for (int i = 0; i < clouds.size(); ++i)
  {
    radius_outlier_removed_clouds.push_back(pre.radius_outlier_remove(clouds[i], ror_radius, ror_neighbor)); // Noise filter
    downsampled_clouds.push_back(pre.downsampling(radius_outlier_removed_clouds[i], downsampleparam));
    calibrated_clouds.push_back(pre.calibrate(downsampled_clouds[i], base2tcp[i], tcp2cam));       // transform to world coordinate
    segmented_clouds.push_back(pre.charucosegmentation(calibrated_clouds[i], minrange, maxrange)); // Cut ROI
    robot_base_calibrated_clouds.push_back(pre.calibratemarker2base(segmented_clouds[i]));
    statistical_outlier_removed_clouds.push_back(pre.statistical_outlier_remove(robot_base_calibrated_clouds[i], sor_mean, sor_thresh)); // Noise filter
  }
  // pre.visualizePointClouds(clouds);
  // pre.visualizePointClouds(radius_outlier_removed_clouds);
  // pre.visualizePointClouds(downsampled_clouds);
  // pre.visualizePointClouds(calibrated_clouds);
  // pre.visualizePointClouds(segmented_clouds);
  // pre.visualizePointClouds(euclidean_outlier_removed_clouds);
  // pre.visualizePointClouds(statistical_outlier_removed_clouds);

  //////////////////Simple Add Version/////////////////////////
  result_cloud = pre.downsampling((pre.addPoint(statistical_outlier_removed_clouds)), 0.003);
  final_clouds.push_back(result_cloud);
  smoothed_cloud_withNormal = (pre.smoothing(final_clouds[0], smoothing_radius));
  smoothed_cloud = pre.PointNormal2PointXYZ(smoothed_cloud_withNormal);
  smoothed_clouds.push_back(smoothed_cloud);
  pre.visualizePointClouds(smoothed_clouds);

  //////////////////////Save Final PCD///////////////
  std::stringstream ss6;
  ss6 << "../pcd/registrated_pcd/lid.pcd";
  pcl::io::savePCDFile(ss6.str(), *(smoothed_cloud), false);

  //////////////////////Meshing Process Start///////////////
  CGAL::Meshing::Point_set points = mesh.read_points_from_file(ss6.str());
  points = mesh.outlier_remove(points);
  // points = mesh.grid_simplify(points);
  CGAL::Meshing::PointList point_list = mesh.convert_to_point_list(points);
  point_list = mesh.estimate_normal(point_list);
  point_list = mesh.bilateral_smooth(point_list);
  mesh.generate_mesh(point_list, "../mesh/lid.stl", relative_alpha, relative_offset); // 100. 1000.
  std::cout
      << "Remeshing done." << std::endl;
  chrono::system_clock::time_point t_end = chrono::system_clock::now();
  /*******************************************/
  chrono::duration<double> t_reg = t_end - t_start;
  cout << "Takes " << t_reg.count() << " sec..." << endl;
}

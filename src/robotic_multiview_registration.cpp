/*Segmentation+OutlierRemove+Downsampling+Registration*/
#include <chrono>     // measure operation time
#include <filesystem> //filesave
#include <vector>
#include "preprocess.h"
#include "universalRobotsKinematics.h"
#include <typeinfo>
using namespace std;

int main(int argc, char **argv)
{                                                                         // measure time start
  chrono::system_clock::time_point t_start = chrono::system_clock::now(); // start measuring registration time
  universalRobots::UR robot;                                              // universalRobots::UR 클래스의 객체 생성
  pointcloudpreprocess::pre pre;                                          // pointcloud 관련 클래스의 객체 생성
  std::vector<Eigen::Matrix4f> base2tcp;                                  // base to tcp TF Matrix
  Eigen::Matrix4f tcp2cam = Eigen::Matrix4f::Identity();                  // TCP to Camera TF Matrix
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, downsampled_clouds, segmented_clouds, calibrated_clouds, radius_outlier_removed_clouds, statistical_outlier_removed_clouds, euclidean_outlier_removed_clouds, registrated_clouds, final_clouds, smoothed_clouds;
  pcl::PointCloud<pcl::PointNormal> smoothed_cloud_withNormal;
  pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 최종 저장용 클라우드

  std::vector<std::string> file_paths;

  float thetaX, thetaY, thetaZ, X, Y, Z, downsampleparam;
  double sor_thresh, ror_radius;

  float minrange[4], maxrange[4], smoothing_radius;
  int n, sor_mean, ror_neighbor;

  // pcd Load
  for (int i = 1; i < argc; ++i)
  {
    file_paths.push_back(argv[i]);
  }
  clouds = pre.loadData(file_paths);
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

    calibrated_clouds.push_back(pre.calibrate(clouds[i], base2tcp[i], tcp2cam));                                                             // transform to world coordinate
    segmented_clouds.push_back(pre.charucosegmentation(calibrated_clouds[i], minrange, maxrange));                                           // Cut ROI
    euclidean_outlier_removed_clouds.push_back(pre.euclidean_outlier_remove(segmented_clouds[i]));                                           // Noise filter
    statistical_outlier_removed_clouds.push_back(pre.statistical_outlier_remove(euclidean_outlier_removed_clouds[i], sor_mean, sor_thresh)); // Noise filter
    radius_outlier_removed_clouds.push_back(pre.radius_outlier_remove(statistical_outlier_removed_clouds[i], ror_radius, ror_neighbor));     // Noise filter
  }
  pre.visualizePointClouds(calibrated_clouds);
  pre.visualizePointClouds(segmented_clouds);
  pre.visualizePointClouds(euclidean_outlier_removed_clouds);
  pre.visualizePointClouds(statistical_outlier_removed_clouds);
  pre.visualizePointClouds(radius_outlier_removed_clouds);

  //////////////////ICP Version/////////////////////////
  // registrated_clouds.push_back(pre.RecursiveRegistration(radius_outlier_removed_clouds));
  // final_clouds.push_back(pre.downsampling(pre.statistical_outlier_remove(pre.radius_outlier_remove(registrated_clouds[0], ror_radius, ror_neighbor), sor_mean, sor_thresh), downsampleparam));
  // smoothed_cloud_withNormal = (pre.smoothing(final_clouds[0], smoothing_radius));
  // smoothed_cloud = pre.PointNormal2PointXYZ(smoothed_cloud_withNormal);
  // smoothed_clouds.push_back(smoothed_cloud);
  // pre.visualizePointClouds(smoothed_clouds);
  //////////////////Simple Add Version/////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  result_cloud = pre.addPoint(radius_outlier_removed_clouds);
  final_clouds.push_back(pre.downsampling(pre.statistical_outlier_remove(pre.radius_outlier_remove(result_cloud, ror_radius, ror_neighbor), sor_mean, sor_thresh), downsampleparam));
  smoothed_cloud_withNormal = (pre.smoothing(final_clouds[0], smoothing_radius));
  smoothed_cloud = pre.PointNormal2PointXYZ(smoothed_cloud_withNormal);
  smoothed_clouds.push_back(smoothed_cloud);
  pre.visualizePointClouds(smoothed_clouds);

  //////////////////////Save Intermidate PCD///////////////

  // save aligned pair, transformed into the first cloud's frame
  // std::stringstream ss2;
  // std::cout << radius_outlier_removed_clouds.size() << std::endl;
  // for (int i = 0; i < radius_outlier_removed_clouds.size(); ++i)
  // {
  //   ss2 << "../pcd/outlier_removed_pcd/fenda/fenda_" << std::to_string(i + 1)
  //       << ".pcd";
  //   pcl::io::savePCDFile(ss2.str(), *radius_outlier_removed_clouds[i], false);
  // }

  //////////////////////Save Final PCD///////////////
  std::stringstream ss;

  ss << "../pcd/registrated_pcd/plate/plate2.pcd";
  pcl::io::savePCDFile(ss.str(), *(smoothed_cloud), false);

  chrono::system_clock::time_point t_end = chrono::system_clock::now();
  /*******************************************/
  chrono::duration<double> t_reg = t_end - t_start;
  cout << "Takes " << t_reg.count() << " sec..." << endl;
}

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr calibrated_cloud, segmented_cloud, euclidean_outlier_removed_cloud, statistical_outlier_removed_cloud, radius_outlier_removed_cloud, result_cloud, exp_downsampled_cloud;

  float thetaX, thetaY, thetaZ, X, Y, Z, downsampleparam;
  double sor_thresh, ror_radius;

  float minrange[4], maxrange[4], smoothing_radius;
  int n, sor_mean, ror_neighbor;

  // pcd Load
  std::string folderPath = argv[1];
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
  exp_downsampled_cloud = pre.downsampling(clouds[0],downsampleparam);

  pcl::io::savePCDFile("../pcd/downsampled_pcd/sample/2/downsampled_sample1_l515.pcd", *exp_downsampled_cloud, false);

  for (int i = 0; i < clouds.size(); ++i)
  {
    radius_outlier_removed_clouds.push_back(pre.radius_outlier_remove(clouds[i], ror_radius, ror_neighbor)); // Noise filter
    downsampled_clouds.push_back(pre.downsampling(radius_outlier_removed_clouds[i], downsampleparam));
    calibrated_clouds.push_back(pre.calibrate(downsampled_clouds[i], base2tcp[i], tcp2cam));       // transform to world coordinate
    segmented_clouds.push_back(pre.charucosegmentation(calibrated_clouds[i], minrange, maxrange)); // Cut ROI
    // euclidean_outlier_removed_clouds.push_back(pre.euclidean_outlier_remove(segmented_clouds[i]));
    // Noise filter
    statistical_outlier_removed_clouds.push_back(pre.statistical_outlier_remove(segmented_clouds[i], sor_mean, sor_thresh)); // Noise filter
  }
  //pre.visualizePointClouds(clouds);
  //pre.visualizePointClouds(radius_outlier_removed_clouds);
  //pre.visualizePointClouds(downsampled_clouds);
  //pre.visualizePointClouds(calibrated_clouds);
  //pre.visualizePointClouds(segmented_clouds);
  // pre.visualizePointClouds(euclidean_outlier_removed_clouds);
  //pre.visualizePointClouds(statistical_outlier_removed_clouds);

  //////////////////ICP Version/////////////////////////
  // registrated_clouds.push_back(pre.RecursiveRegistration(radius_outlier_removed_clouds));
  // final_clouds.push_back(pre.downsampling(pre.statistical_outlier_remove(pre.radius_outlier_remove(registrated_clouds[0], ror_radius, ror_neighbor), sor_mean, sor_thresh), downsampleparam));
  // smoothed_cloud_withNormal = (pre.smoothing(final_clouds[0], smoothing_radius));
  // smoothed_cloud = pre.PointNormal2PointXYZ(smoothed_cloud_withNormal);
  // smoothed_clouds.push_back(smoothed_cloud);
  // pre.visualizePointClouds(smoothed_clouds);
  //////////////////Simple Add Version/////////////////////////

  result_cloud = pre.addPoint(statistical_outlier_removed_clouds);
  final_clouds.push_back(result_cloud);
  //final_clouds.push_back((pre.statistical_outlier_remove(result_cloud, sor_mean, sor_thresh * 1.5)));
  // final_clouds.push_back(pre.downsampling(pre.euclidean_outlier_remove(pre.statistical_outlier_remove(result_cloud, sor_mean, sor_thresh)),downsampleparam));
  //  final_clouds.push_back(pre.downsampling((pre.radius_outlier_remove(result_cloud, ror_radius, ror_neighbor)), downsampleparam));
  //smoothed_cloud_withNormal = (pre.smoothing(final_clouds[0], smoothing_radius));
  //smoothed_cloud = pre.PointNormal2PointXYZ(smoothed_cloud_withNormal);
  //smoothed_clouds.push_back(smoothed_cloud);
  //pre.visualizePointClouds(final_clouds);
  //pre.visualizePointClouds(smoothed_clouds);

  //////////////////////Save Intermidate PCD///////////////

  std::stringstream ss;
  calibrated_cloud = pre.addPoint(calibrated_clouds);
  ss << "../pcd/calibrated_pcd/sample/2/calibrated_sample_l515.pcd";
  pcl::io::savePCDFile(ss.str(), *calibrated_cloud, false);

  std::stringstream ss2;
  segmented_cloud = pre.addPoint(segmented_clouds);
  ss2 << "../pcd/segmented_pcd/sample/2/segmented_sample_l515.pcd";
  pcl::io::savePCDFile(ss2.str(), *segmented_cloud, false);

  // std::stringstream ss3;
  // euclidean_outlier_removed_cloud = pre.addPoint(euclidean_outlier_removed_clouds);
  // ss3 << "../pcd/outlier_removed_pcd/lid/euclidean_removed_lid_l515.pcd";
  // pcl::io::savePCDFile(ss3.str(), *euclidean_outlier_removed_cloud, false);

  std::stringstream ss4;
  statistical_outlier_removed_cloud = pre.addPoint(statistical_outlier_removed_clouds);
  ss4 << "../pcd/outlier_removed_pcd/sample/2/sor_removed_sample_l515.pcd";
  pcl::io::savePCDFile(ss4.str(), *statistical_outlier_removed_cloud, false);

  std::stringstream ss5;
  radius_outlier_removed_cloud = pre.addPoint(radius_outlier_removed_clouds);
  ss5 << "../pcd/outlier_removed_pcd/sample/2/ror_removed_sample_l515.pcd";
  pcl::io::savePCDFile(ss5.str(), *radius_outlier_removed_cloud, false);

  //////////////////////Save Final PCD///////////////
  std::stringstream ss6;
  ss6 << "../pcd/registrated_pcd/sample/2/sample_l515.pcd";
  pcl::io::savePCDFile(ss6.str(), *(result_cloud), false);

  chrono::system_clock::time_point t_end = chrono::system_clock::now();
  /*******************************************/
  chrono::duration<double> t_reg = t_end - t_start;
  cout << "Takes " << t_reg.count() << " sec..." << endl;
}

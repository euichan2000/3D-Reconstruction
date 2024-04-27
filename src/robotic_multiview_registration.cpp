/*Segmentation+OutlierRemove+Downsampling+Registration*/
#include <chrono>     // measure operation time
#include <filesystem> //filesave
#include <vector>
#include "preprocess.h"
#include "universalRobotsKinematics.h"
using namespace std;

int main(int argc, char **argv)
{                                                                         // measure time start
  chrono::system_clock::time_point t_start = chrono::system_clock::now(); // start measuring registration time
  universalRobots::UR robot; // universalRobots::UR 클래스의 객체 생성
  pointcloudpreprocess::pre pre;
  std::vector<Eigen::Matrix4f> base2tcp; // base to tcp TF Matrix
  Eigen::Matrix4f tcp2cam = Eigen::Matrix4f::Identity();
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, downsampled_clouds, segmented_clouds, calibrated_clouds, radius_outlier_removed_clouds, statistical_outlier_removed_clouds, final_clouds;

  pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<std::string> file_paths;

  float thetaX, thetaY, thetaZ, X, Y, Z, downsampleparam, sor_thresh, ror_radius;
  float scene[10][6];
  float minrange[4], maxrange[4];
  int n, sor_mean, ror_neighbor;
  robot.loadYAML("../reconstruction.yaml", thetaX, thetaY, thetaZ, X, Y, Z, scene, n, minrange, maxrange, downsampleparam, sor_mean, sor_thresh, ror_radius, ror_neighbor);

  tcp2cam << cos(thetaZ) * cos(thetaY), cos(thetaZ) * sin(thetaY) * sin(thetaX) - sin(thetaZ) * cos(thetaX), cos(thetaZ) * sin(thetaY) * cos(thetaX) + sin(thetaZ) * sin(thetaX), X,
      sin(thetaZ) * cos(thetaY), sin(thetaZ) * sin(thetaY) * sin(thetaX) + cos(thetaZ) * cos(thetaX), sin(thetaZ) * sin(thetaY) * cos(thetaX) - cos(thetaZ) * sin(thetaX), Y,
      -sin(thetaY), cos(thetaY) * sin(thetaX), cos(thetaY) * cos(thetaX), Z,
      0, 0, 0, 1;

  for (int i = 1; i < argc; ++i)
  {
    file_paths.push_back(argv[i]);
  }
  clouds = pre.loadData(file_paths);

  for (int i = 0; i < clouds.size(); ++i)
  {
    base2tcp.push_back(robot.forwardKinematics(scene[i]));
  }
  std::cout << "clouds size: "
            << clouds.size() << std::endl;

  for (int i = 0; i < clouds.size(); ++i)
  {

    calibrated_clouds.push_back(pre.calibrate(clouds[i], base2tcp[i], tcp2cam)); // transform to base coordinate
    segmented_clouds.push_back(pre.charucosegmentation(calibrated_clouds[i], minrange, maxrange));
    statistical_outlier_removed_clouds.push_back(pre.statistical_outlier_remove(segmented_clouds[i],sor_mean,sor_thresh));
    radius_outlier_removed_clouds.push_back(pre.radius_outlier_remove(statistical_outlier_removed_clouds[i],ror_radius,ror_neighbor)); // outlier remove
  }
  //pre.visualizePointClouds(calibrated_clouds);
  //pre.visualizePointClouds(segmented_clouds);
  //pre.visualizePointClouds(statistical_outlier_removed_clouds);
  //pre.visualizePointClouds(radius_outlier_removed_clouds);
  final_clouds.push_back(pre.radius_outlier_remove(pre.RecursiveRegistration(radius_outlier_removed_clouds),0.01,50));
  //pre.visualizePointClouds(final_clouds);

  // save aligned pair, transformed into the first cloud's frame
  std::stringstream ss;
  for (int i = 0; i < final_clouds.size(); ++i)
  {
    ss << "../pcd/registrated_pcd/plate/plate_" << i + 1
       << ".pcd";
    pcl::io::savePCDFile(ss.str(), *(final_clouds[i]), false);
  }

  chrono::system_clock::time_point t_end = chrono::system_clock::now();
  /*******************************************/
  chrono::duration<double> t_reg = t_end - t_start;
  cout << "Takes " << t_reg.count() << " sec..." << endl;
}

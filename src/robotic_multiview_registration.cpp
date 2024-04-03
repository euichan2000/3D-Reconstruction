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
  PointCloud::Ptr result(new PointCloud);
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f target2source = Eigen::Matrix4f::Identity();
  universalRobots::UR robot; // universalRobots::UR 클래스의 객체 생성
  pointcloudpreprocess::pre pre;
  std::vector<Eigen::Matrix4f> base2tcp; // base to tcp TF Matrix
  Eigen::Matrix4f tcp2cam = Eigen::Matrix4f::Identity();
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, downsampled_clouds, segmented_clouds, calibrated_clouds, outlier_removed_clouds;
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> result_clouds(4, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>(4));
  pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PCLVisualizer viewer;
  std::vector<std::string> file_paths;

  float thetaX, thetaY, thetaZ, X, Y, Z;
  float scene1[6], scene2[6], scene3[6], scene4[6];

  robot.loadYAML("../setting.yaml", thetaX, thetaY, thetaZ, X, Y, Z, scene1, scene2, scene3, scene4);

  tcp2cam << cos(thetaZ) * cos(thetaY), cos(thetaZ) * sin(thetaY) * sin(thetaX) - sin(thetaZ) * cos(thetaX), cos(thetaZ) * sin(thetaY) * cos(thetaX) + sin(thetaZ) * sin(thetaX), X,
      sin(thetaZ) * cos(thetaY), sin(thetaZ) * sin(thetaY) * sin(thetaX) + cos(thetaZ) * cos(thetaX), sin(thetaZ) * sin(thetaY) * cos(thetaX) - cos(thetaZ) * sin(thetaX), Y,
      -sin(thetaY), cos(thetaY) * sin(thetaX), cos(thetaY) * cos(thetaX), Z,
      0, 0, 0, 1;

  base2tcp.push_back(robot.forwardKinematics(scene1)); //
  base2tcp.push_back(robot.forwardKinematics(scene2)); //
  base2tcp.push_back(robot.forwardKinematics(scene3)); //
  base2tcp.push_back(robot.forwardKinematics(scene4)); //

  for (int i = 1; i < argc; ++i)
  {
    file_paths.push_back(argv[i]);
  }
  clouds = pre.loadData(file_paths);

  for (int n = 0; n < clouds.size(); n++)
  {

    outlier_removed_clouds.push_back(pre.outlier_remove(clouds[n]));                                   // outlier remove
    segmented_clouds.push_back(pre.mincutsegmentation(outlier_removed_clouds[n], 0.2, 0.0, 0.0, 0.2)); // radius,x,y,z of centerpoint of object
    calibrated_clouds.push_back(pre.calibrate(segmented_clouds[n], base2tcp[n], tcp2cam));             // transform to base coordinate
    downsampled_clouds.push_back(pre.downsampling(calibrated_clouds[n], 3000));                        // downsample to under 3000 points
  }
  pre.visualizePointClouds(downsampled_clouds);
  
    std::cout << "size of downsampled_clouds" << downsampled_clouds.size() << std::endl;
    for (int i = 0; i < downsampled_clouds.size(); ++i)
    {
      result_clouds[0][i] = downsampled_clouds[i];
    }

    // pre.visualizePointClouds(downsampled_clouds);
    //*result_data[1][1].cloud = *result_data[0][0].cloud + *result_data[1][0].cloud;

    //  compute registration & make final PCD

    for (int i = 0; i < clouds.size() - 1; ++i)
    {

      int j = 0;
      while (true)
      {

        source = result_clouds[i][j];
        target = result_clouds[i][j + 1];

        if (source && target)
        {
          // source와 target이 nullptr이 아닌 경우에만 실행
          target2source = pre.fineICP(source, target);
          std::cerr << "icp complete." << std::endl;
        }
        else
        {
          // source 또는 target이 nullptr인 경우 처리
          std::cerr << "source 또는 target이 nullptr입니다." << std::endl;
        }

        // // transform current pair into the global transform
        pcl::transformPointCloud(*target, *result, target2source);
        *result+=*source;
        // // update the global transform
        GlobalTransform *= target2source;
        // // result = downsampling(result, 1000);
        result_clouds[i + 1][j] = result;
        if (j >= (int)clouds.size() - (i + 2))
        {

          break;
        }
        j++;
      }
    }

    // Modify the width and height
    // result_clouds[clouds.size() - 1][0]->width = 1; // Set to the desired modified width
    // result_clouds[clouds.size() - 1][0]->height = result_clouds[clouds.size() - 1][0]->points.size();

    // save aligned pair, transformed into the first cloud's frame
    std::stringstream ss;
    ss << "../pcd/registrated_pcd/result_grinder"
       << ".pcd";
    pcl::io::savePCDFile(ss.str(), *(result_clouds[clouds.size() - 1][0]), true);
  
  chrono::system_clock::time_point t_end = chrono::system_clock::now();
  /*******************************************/
  chrono::duration<double> t_reg = t_end - t_start;
  cout << "Takes " << t_reg.count() << " sec..." << endl;
}

/* ]--- */
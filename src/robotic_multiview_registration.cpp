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
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
  universalRobots::UR robot; // universalRobots::UR 클래스의 객체 생성
  pointcloudpreprocess::pre pre;
  std::vector<Eigen::Matrix4f> base2tcp; // base to tcp TF Matrix
  Eigen::Matrix4f tcp2cam = Eigen::Matrix4f::Identity();
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, downsampled_clouds, segmented_clouds, calibrated_clouds, outlier_removed_clouds;
  pcl::visualization::PCLVisualizer viewer;
  std::vector<std::string> file_paths;
  float thetaX, thetaY, thetaZ, X,Y,Z;
  float scene1[6], scene2[6], scene3[6],scene4[6];

  robot.loadYAML("../setting.yaml",thetaX,thetaY,thetaZ,X,Y,Z,scene1,scene2);

  tcp2cam << cos(thetaZ) * cos(thetaY), cos(thetaZ) * sin(thetaY) * sin(thetaX) - sin(thetaZ) * cos(thetaX), cos(thetaZ) * sin(thetaY) * cos(thetaX) + sin(thetaZ) * sin(thetaX), X,
      sin(thetaZ) * cos(thetaY), sin(thetaZ) * sin(thetaY) * sin(thetaX) + cos(thetaZ) * cos(thetaX), sin(thetaZ) * sin(thetaY) * cos(thetaX) - cos(thetaZ) * sin(thetaX), Y,
      -sin(thetaY), cos(thetaY) * sin(thetaX), cos(thetaY) * cos(thetaX), Z,
      0, 0, 0, 1;

  base2tcp.push_back(robot.forwardKinematics(scene1)); // 4.04, -102.28, -113.73, -85.89, 119.93, -16.69; // 655.22, -68.88, 261.57, 2.319, -1.871, -1.163;
  base2tcp.push_back(robot.forwardKinematics(scene2)); // 92.01, -78.17, -144.05, -86.14, 32.691, -16.78; // 226.75, 366.35, 232.88, 2.056, 0.759, 1.073;
  // base2tcp.push_back(robot.forwardKinematics(scene3)); // 33.18, -110.73, -97.43, -96.93, 76.07, -16.77;  // 739.64, 265.13, 290.95, 2.386, -0.671, -0.232;
  // base2tcp.push_back(robot.forwardKinematics(scene4)); //-26.47, -79.56, -150.49, -69.63, 154.08, 42.70; // 266.13, -220.97, 200.26, 0.055, -2.856, -1.861;

  for (int i = 1; i < argc; ++i)
  {
    file_paths.push_back(argv[i]);
  }
  clouds = pre.loadData(file_paths);

  for (int n = 0; n < clouds.size(); n++)
  {

    outlier_removed_clouds.push_back(pre.outlier_remove(clouds[n])); //outlier remove
    segmented_clouds.push_back(pre.mincutsegmentation(outlier_removed_clouds[n], 0.2, 0.0, 0.0, 0.2)); //radius,x,y,z of centerpoint of object
    calibrated_clouds.push_back(pre.calibrate(segmented_clouds[n], base2tcp[n], tcp2cam)); //transform to base coordinate
    downsampled_clouds.push_back(pre.downsampling(calibrated_clouds[n], 3000)); // downsample to under 3000 points
    // result_data[0][n].cloud = outlier_remove(data[n].cloud);
  }

  //*result_data[1][1].cloud = *result_data[0][0].cloud + *result_data[1][0].cloud;
  pre.visualizePointClouds(downsampled_clouds);

  //  compute registration & make final PCD
  /*
    for (size_t i = 0; i < (int)data.size() - 1; ++i)
    {

      int j = 0;
      while (true)
      {

        source = result_data[i][j].cloud;
        target = result_data[i][j + 1].cloud;

        // Add visualization data
        // showCloudsLeft(source, target);
        PointCloud::Ptr temp(new PointCloud);
        PCL_INFO("Aligning %s (%zu) with %s (%zu).\n", data[j - 1].f_name.c_str(), static_cast<size_t>(source->size()), data[j].f_name.c_str(), static_cast<size_t>(target->size()));
        PCL_INFO("i: %d j: %d", i, j);
        fineICP(source, target, temp, pairTransform);

        // transform current pair into the global transform
        pcl::transformPointCloud(*temp, *result, GlobalTransform);
        // update the global transform
        GlobalTransform *= pairTransform;
        // result = downsampling(result, 1000);

        result_data[i + 1][j].cloud->points = result->points;
        if (j >= (int)data.size() - (i + 2))
        {

          break;
        }
        j++;
      }
    }

    // Modify the width and height
    result_data[dataSize - 1][0].cloud->width = 1; // Set to the desired modified width
    result_data[dataSize - 1][0].cloud->height = result_data[dataSize - 1][0].cloud->points.size();
  */
  // save aligned pair, transformed into the first cloud's frame
  std::stringstream ss;
  ss << "../pcd/registrated_pcd/bunny/result_bunny"
     << ".pcd";
  pcl::io::savePCDFile(ss.str(), *(downsampled_clouds[0]), true);

  chrono::system_clock::time_point t_end = chrono::system_clock::now();
  /*******************************************/
  chrono::duration<double> t_reg = t_end - t_start;
  cout << "Takes " << t_reg.count() << " sec..." << endl;
}

/* ]--- */
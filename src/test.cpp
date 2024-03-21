/*Segmentation+OutlierRemove+Downsampling+Registration*/
#include <chrono>     // measure operation time
#include <filesystem> //filesave
#include <vector>
#include "preprocess.h"
#include "universalRobotsKinematics.h"

// #include <opencv4/opencv2/opencv.hpp> //opencv

using namespace std;

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
{                                                                         // measure time start
  chrono::system_clock::time_point t_start = chrono::system_clock::now(); // start measuring registration time
  PointCloud::Ptr result(new PointCloud), source, target, filtered_result;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
  universalRobots::UR robot; // universalRobots::UR 클래스의 객체 생성
  pointcloudpreprocess::pre pre;
  std::vector<Eigen::Matrix4f> base2tcp; // base to tcp TF Matrix
  Eigen::Matrix4f tcp2cam = Eigen::Matrix4f::Identity();
  std::vector<PCD, Eigen::aligned_allocator<PCD>> data;
  pcl::visualization::PCLVisualizer viewer;
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(result_data[1][0].cloud, 255, 255, 255);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(result_data[0][0].cloud, 230, 20, 20); // Red
  float scene1[6] = {-7.76, -88.08, -110.72, -87.69, 134.65, -19.56};
  float scene2[6] = {78.38, -75.95, -129.98, -95.34, 32.80, -14.10};
  // float scene3[6] = {33.18, -110.73, -97.43, -96.93, 76.07, -16.77};
  // float scene4[6] = {-26.47, -79.56, -150.49, -69.63, 154.08, 42.70};

  double thetaX = 0 * M_PI / 180;
  double thetaY = 0 * M_PI / 180;
  double thetaZ = 180 * M_PI / 180;

  tcp2cam << cos(thetaZ) * cos(thetaY), cos(thetaZ) * sin(thetaY) * sin(thetaX) - sin(thetaZ) * cos(thetaX), cos(thetaZ) * sin(thetaY) * cos(thetaX) + sin(thetaZ) * sin(thetaX), 0,
      sin(thetaZ) * cos(thetaY), sin(thetaZ) * sin(thetaY) * sin(thetaX) + cos(thetaZ) * cos(thetaX), sin(thetaZ) * sin(thetaY) * cos(thetaX) - cos(thetaZ) * sin(thetaX), 0,
      -sin(thetaY), cos(thetaY) * sin(thetaX), cos(thetaY) * cos(thetaX), 0.09363,
      0, 0, 0, 1;

  base2tcp.push_back(robot.forwardKinematics(scene1)); // 4.04, -102.28, -113.73, -85.89, 119.93, -16.69; // 655.22, -68.88, 261.57, 2.319, -1.871, -1.163;
  base2tcp.push_back(robot.forwardKinematics(scene2)); // 92.01, -78.17, -144.05, -86.14, 32.691, -16.78; // 226.75, 366.35, 232.88, 2.056, 0.759, 1.073;
  // base2tcp.push_back(robot.forwardKinematics(scene3)); // 33.18, -110.73, -97.43, -96.93, 76.07, -16.77;  // 739.64, 265.13, 290.95, 2.386, -0.671, -0.232;
  // base2tcp.push_back(robot.forwardKinematics(scene4)); //-26.47, -79.56, -150.49, -69.63, 154.08, 42.70; // 266.13, -220.97, 200.26, 0.055, -2.856, -1.861;

  // tcp2cam << cos(theta), -sin(theta), 0, -0.105158, // tcp 2 camera TF Matrix. 회전 방향 다를 수 있음.
  //     sin(theta), cos(theta), 0, 0,
  //     0, 0, 1, 0.040145,
  //     0, 0, 0, 1;

  // for (int i = 0; i < 2; i++)
  // {

  //   std::cout << "base2tcp[" << i << "]\n"
  //             << base2tcp[i] << "\n"
  //             << std::endl;
  // }

  // std::cout << "tcp2cam"
  //           << "\n"
  //           << tcp2cam << "\n"
  //           << std::endl;
  loadData(argc, argv, data);

  if (data.empty())
  {
    PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]\n", argv[0]);
    PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc\n");
    return (-1);
  }
  PCL_INFO("Loaded %d datasets.\n", (int)data.size());
  int dataSize = static_cast<int>(data.size());
  // create 2 dim result_data array
  std::vector<std::vector<PCD>> result_data(dataSize, std::vector<PCD>(dataSize)); // registration result data

  for (int n = 0; n < dataSize; n++)
  {
    result_data[n][0].cloud = pre.calibrate(data[n].cloud, base2tcp[n], tcp2cam);

    // result_data[0][n].cloud = outlier_remove(data[n].cloud);
  }

  *result_data[1][1].cloud = *result_data[0][0].cloud + *result_data[1][0].cloud;
  
  viewer.addPointCloud(result_data[1][0].cloud, source_cloud_color_handler, "original_cloud");

  
  viewer.addPointCloud(result_data[0][0].cloud, transformed_cloud_color_handler, "transformed_cloud");
  viewer.addCoordinateSystem(1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  // viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped())
  { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce();
  }

  //  compute registration & make final PCD

  //   for (size_t i = 0; i < (int)data.size() - 1; ++i)
  //   {

  //   int j = 0;
  //   while (true)
  //   {

  //     source = result_data[i][j].cloud;
  //     target = result_data[i][j + 1].cloud;

  //     // Add visualization data
  //     // showCloudsLeft(source, target);
  //     PointCloud::Ptr temp(new PointCloud);
  //     PCL_INFO("Aligning %s (%zu) with %s (%zu).\n", data[j - 1].f_name.c_str(), static_cast<size_t>(source->size()), data[j].f_name.c_str(), static_cast<size_t>(target->size()));
  //     PCL_INFO("i: %d j: %d", i, j);
  //     fineICP(source, target, temp, pairTransform);

  //     // transform current pair into the global transform
  //     pcl::transformPointCloud(*temp, *result, GlobalTransform);
  //     // update the global transform
  //     GlobalTransform *= pairTransform;
  //     // result = downsampling(result, 1000);

  //     result_data[i + 1][j].cloud->points = result->points;
  //     if (j >= (int)data.size() - (i + 2))
  //     {

  //       break;
  //     }
  //     j++;
  //   }
  // }

  // Modify the width and height
  result_data[dataSize - 1][0].cloud->width = 1; // Set to the desired modified width
  result_data[dataSize - 1][0].cloud->height = result_data[dataSize - 1][0].cloud->points.size();

  // save aligned pair, transformed into the first cloud's frame
  std::stringstream ss;
  ss << "./registrated_pcd/result_bunny"
     << ".pcd";
  pcl::io::savePCDFile(ss.str(), *(result_data[1][1].cloud), true);

  chrono::system_clock::time_point t_end = chrono::system_clock::now();
  /*******************************************/
  chrono::duration<double> t_reg = t_end - t_start;
  cout << "Takes " << t_reg.count() << " sec..." << endl;
}

/* ]--- */
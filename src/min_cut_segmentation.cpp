#include "preprocess.h"

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
{
  std::vector<PCD, Eigen::aligned_allocator<PCD>> data;

  loadData(argc, argv, data);
  if (data.empty())
  {
    PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]\n", argv[0]);
    PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc\n");
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = *data[0].cloud;
  if (cloud->empty())
  {
    PCL_ERROR("PointCloud is empty.\n");
    return (-1);
  }

  pointcloudpreprocess::pre seg;
  cloud_filtered = seg.segmentation(cloud,0,0,0.2);

  //std::cout << "Maximum flow is " << Minseg.getMaxFlow() << std::endl;

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = Minseg.getColoredCloud();
  pcl::visualization::CloudViewer viewer("Cluster viewer");

  viewer.showCloud(cloud_filtered);

  while (!viewer.wasStopped())
  {
  }
  std::stringstream ss;
  ss << "../pcd/segmented_pcd/bunny/segmented_bunny"
     << ".pcd";
  pcl::io::savePCDFile(ss.str(), *cloud_filtered, true);

  return (0);
}
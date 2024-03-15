/*outlier remove*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <filesystem>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = *data[0].cloud;
  if (cloud->empty())
  {
    PCL_ERROR("PointCloud is empty.\n");
    return (-1);
  }
  // Fill in the cloud data
  // pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  // reader.read<pcl::PointXYZ> ("gun1.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  std::filesystem::path filePath(data[0].f_name);
  std::string fileName = filePath.stem().string();
  std::string clusterFileName = "./outlier_removed_pcd/gun/outlier_removed_" + fileName + ".pcd";

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>(clusterFileName, *cloud_filtered, false);

  // sor.setNegative (true);
  // sor.filter (*cloud_filtered);
  // writer.write<pcl::PointXYZ> ("gun1_outliers.pcd", *cloud_filtered, false);

  return (0);
}
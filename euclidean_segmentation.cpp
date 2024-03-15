/*Euclidean Segmentation*/
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <iomanip> // for setw, setfill
#include <filesystem>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct PCD
{
  PointCloud::Ptr cloud; // Ptr type 변수: cloud
  std::string f_name;    // string type 변수:  f_name

  PCD() : cloud(new PointCloud){}; // PCD 함수 생성
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
 * \param argc the number of arguments (pass from main ())
 * \param argv the actual command line arguments (pass from main ())
 * \param models the resultant vector of point cloud datasets
 */
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

  // Read in the cloud data
  // pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  // reader.read("./temp_pcd/banana_6.pcd", *cloud);

  *cloud = *data[0].cloud;
  if (cloud->empty())
  {
    PCL_ERROR("PointCloud is empty.\n");
    return (-1);
  }

  std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*
  // std::string output_directory = "/3dscan/build/seg/";
  //  Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  int nr_points = (int)cloud_filtered->size();
  while (cloud_filtered->size() > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  int cluster_count = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr selected_clusters(new pcl::PointCloud<pcl::PointXYZ>);
  float min_distance = std::numeric_limits<float>::max();
  pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &idx : cluster.indices)
    {
      cloud_cluster->push_back((*cloud_filtered)[idx]);
    }
    cluster_count++;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    float distance = centroid.head<3>().norm();

    // std::cout << "Distance between centroid and origin: " << distance << std::endl;
    //  최소 거리인 경우 nearest_cloud를 업데이트합니다.
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_cloud = cloud_cluster;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_final = nearest_cloud;

  cloud_final->width = cloud_final->size();
  cloud_final->height = 1;
  cloud_final->is_dense = true;

  std::cout << "nearest_cloud: " << min_distance << " data points." << std::endl;
  std::cout << "PointCloud representing the Cluster: " << cloud_final->size() << " data points." << std::endl;

  std::filesystem::path filePath(data[0].f_name);
  std::string fileName = filePath.stem().string();
  std::string clusterFileName = "./segmented_pcd/bunny/segmented_" + fileName + ".pcd";
  pcl::io::savePCDFile(clusterFileName, *cloud_final, true);
  // writer.write<pcl::PointXYZ>(clusterFileName, *cloud_final, false);

  return (0);
}
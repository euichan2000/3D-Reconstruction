/*Segmentation+OutlierRemove+Downsampling+Registration*/
#include <vector>
#include <chrono>
#include <filesystem>
#include <Eigen/Core>

#include <pcl/pcl_macros.h>
#include <pcl/memory.h> // for pcl::make_shared
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

// convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// for visualization
pcl::visualization::PCLVisualizer *p;
// its left and right viewports
int vp_1, vp_2;

// convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud; // Ptr type 변수: cloud
  std::string f_name;    // string type 변수:  f_name

  PCD() : cloud(new PointCloud){}; // PCD 함수 생성
};
// PCD 파일 정렬
struct PCDComparator
{
  bool operator()(const PCD &p1, const PCD &p2)
  {
    return (p1.f_name < p2.f_name);
  }
};
// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
  MyPointRepresentation()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray(const PointNormalT &p, float *out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

/////////////Function///////////////////////////////////////////////////////////////////

void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud("vp1_target");
  p->removePointCloud("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
  p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO("Press q to begin the registration.\n");
  p->spin();
}

////////////////////////////////////////////////////////////////////////////////

void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud("source");
  p->removePointCloud("target");

  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
  if (!tgt_color_handler.isCapable())
    PCL_WARN("Cannot create curvature color handler!\n");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
  if (!src_color_handler.isCapable())
    PCL_WARN("Cannot create curvature color handler!\n");

  p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

PointCloud::Ptr downsampling(const PointCloud::Ptr cloud_src, int number)
{
  PointCloud::Ptr cloud_tgt(new PointCloud);
  if (cloud_src->width * cloud_src->height > number)
  {
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_src);
    sor.setLeafSize(0.005f, 0.005f, 0.005f);
    sor.filter(*cloud_tgt);

    std::cerr << "PointCloud after filtering: " << cloud_tgt->width * cloud_tgt->height
              << " data points (" << pcl::getFieldsList(*cloud_tgt) << ")." << std::endl;
  }
  else
  {
    std::cerr << "Skipping filtering and downsampling. Point cloud has fewer than" << number << " points." << std::endl;
    *cloud_tgt = *cloud_src;
  }

  return cloud_tgt;
}

////////////////////////////////////////////////////////////////////////////////

PointCloud::Ptr outlier_remove(const PointCloud::Ptr cloud_src)
{
  if (cloud_src == nullptr || cloud_src->empty())
  {
    cout << "no data in outler_remove" << endl;
  }
  PointCloud::Ptr cloud_tgt(new PointCloud);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_src);
  sor.setMeanK(10);
  sor.setStddevMulThresh(2.0);
  sor.filter(*cloud_tgt);

  return cloud_tgt;
}

////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4f coarseregistration(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt)
{
  if (cloud_src == nullptr || cloud_src->empty())
  {
    cout << "no data in coarseregistration" << endl;
  }
  PointCloud::Ptr cloud_source_ptr(new PointCloud);
  PointCloud::Ptr cloud_target_ptr(new PointCloud);

  cloud_source_ptr = cloud_src;
  cloud_target_ptr = cloud_tgt;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method_ptr = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals_source = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr normals_target = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

  norm_est.setInputCloud(cloud_source_ptr);
  norm_est.setSearchMethod(search_method_ptr);
  norm_est.setRadiusSearch(0.02);
  norm_est.compute(*normals_source);

  norm_est.setInputCloud(cloud_target_ptr);
  norm_est.setSearchMethod(search_method_ptr);
  norm_est.setRadiusSearch(0.02);
  norm_est.compute(*normals_target);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_source = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_target = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;

  fpfh_est.setInputCloud(cloud_source_ptr);
  fpfh_est.setInputNormals(normals_source);
  fpfh_est.setSearchMethod(search_method_ptr);
  fpfh_est.setRadiusSearch(0.02);
  fpfh_est.compute(*features_source);

  fpfh_est.setInputCloud(cloud_target_ptr);
  fpfh_est.setInputNormals(normals_target);
  fpfh_est.setSearchMethod(search_method_ptr);
  fpfh_est.setRadiusSearch(0.02);
  fpfh_est.compute(*features_target);

  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
  Eigen::Matrix4f final_transformation;
  sac_ia.setInputSource(cloud_source_ptr);
  sac_ia.setSourceFeatures(features_source);
  sac_ia.setInputTarget(cloud_target_ptr);
  sac_ia.setTargetFeatures(features_target);
  sac_ia.setMaximumIterations(1000);
  sac_ia.setMinSampleDistance(0.01f);
  sac_ia.setMaxCorrespondenceDistance(0.1);
  pcl::PointCloud<pcl::PointXYZ> finalcloud;
  sac_ia.align(finalcloud);
  sac_ia.getCorrespondenceRandomness();
  Eigen::Matrix4f init_transform = sac_ia.getFinalTransformation();
  double coarse_score = sac_ia.getFitnessScore();
  std::cout << "coarse_score: " << coarse_score << std::endl;
  return init_transform;
}

////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4f fineregistration(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src(new PointCloud);
  PointCloud::Ptr tgt(new PointCloud);

  src = cloud_src;
  tgt = cloud_tgt;

  // Align
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 1cm
  // Note: adjust this based on the size of your datasets
  icp.setMaxCorrespondenceDistance(0.1);
  // Set the point representation

  icp.setInputSource(src);
  icp.setInputTarget(tgt);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
  PointCloud::Ptr icp_result(new PointCloud);
  icp.setMaximumIterations(1000);

  icp.align(*icp_result);

  // accumulate transformation between each Iteration
  Ti = icp.getFinalTransformation();
  // if the difference between this transformation and the previous one
  // is smaller than the threshold, refine the process by reducing
  // the maximal correspondence distance
  double fine_score = icp.getFitnessScore();
  // std::cout << "converge score: " << score << std::endl;
  //
  //  Get the transformation from target to source
  targetToSource = Ti.inverse();

  return targetToSource;
}

////////////////////////////////////////////////////////////////////////////////

PointCloud::Ptr segmentation(const PointCloud::Ptr cloud_src)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = *cloud_src;

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*cloud_filtered);
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_cloud;
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final = nearest_cloud;
  return cloud_final;
}

/* ---[ */
int main(int argc, char **argv)
{ // measure time start
  chrono::system_clock::time_point t_start = chrono::system_clock::now();
  // Load data
  std::vector<PCD, Eigen::aligned_allocator<PCD>> data;
  loadData(argc, argv, data);

  // Check user input
  if (data.empty())
  {
    PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]\n", argv[0]);
    PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc\n");
    return (-1);
  }

  PCL_INFO("Loaded %d datasets.\n", (int)data.size());
  int dataSize = static_cast<int>(data.size());
  // create 2 dim result_data array
  std::vector<std::vector<PCD>> result_data(dataSize, std::vector<PCD>(dataSize));

  // segmentation

  // insert input data in first row of result_data

  // outlier_remove
  for (int n = 0; n < dataSize; n++)
  {
    data[n].cloud = outlier_remove(data[n].cloud);
  }

  // Create a PCLVisualizer object
  // p = new pclPointNormalTewPort(0.0, 0, 0.5, 1.0, vp_1);
  // p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

  PointCloud::Ptr result(new PointCloud), source(new PointCloud), target(new PointCloud), target_2(new PointCloud);
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), coarseTransform, fineTransform;

  pcl::GeneralizedIterativeClosestPoint<PointNormalT, PointNormalT> icp;
  //  compute registration & make final PCD
  source = data[0].cloud;
  target = data[1].cloud;
  PointCloud::Ptr true_result(new PointCloud), cloud_src_output_ptr(new PointCloud), cloud_tgt_output_ptr(new PointCloud);
  coarseTransform = coarseregistration(source, target);
  pcl::transformPointCloud(*target, *target_2, coarseTransform);
  fineTransform = fineregistration(source, target_2);
  // //  transform current pair into the global transform
  pcl::transformPointCloud(*target_2, *result, fineTransform);
  *true_result = *result + *source;
  true_result = outlier_remove(true_result);
  true_result = downsampling(true_result, 1000);
  std::stringstream ss;
  ss << "./registrated_pcd/result_test_"
     << ".pcd";
  pcl::io::savePCDFile(ss.str(), *true_result, true);
}
/* ]--- */
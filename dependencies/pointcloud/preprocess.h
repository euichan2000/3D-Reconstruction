#include <vector>

#include <pcl/pcl_macros.h>
#include <pcl/memory.h> // for make_shared
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


namespace pointcloudpreprocess
{
    class pre
    {
    public:
        //void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models);
        PointCloud::Ptr downsampling(const PointCloud::Ptr cloud_src, int number);
        PointCloud::Ptr outlier_remove(const PointCloud::Ptr cloud_src);
        PointCloud::Ptr calibrate(PointCloud::Ptr cloud_src, Eigen::Matrix4f &base2tcp, Eigen::Matrix4f &tcp2cam);
        PointCloud::Ptr segmentation(const PointCloud::Ptr cloud_src);
        

    };

}

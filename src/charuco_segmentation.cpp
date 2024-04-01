#include "universalRobotsKinematics.h"
#include <chrono>     // measure operation time
#include <filesystem> //filesave
#include <vector>
#include "preprocess.h"

// 1. Calculate base2marker Transformation matrix
// 2. Pointcloud from marker = (TF matrix from base joint to marker) *(Pointcloud from base joint)
// 3. cut Pointcloud by Region
// 4. Re-transform to base joint coordinate

int main()
{
    Eigen::Matrix4f base2marker, marker2base;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> joint_based_clouds;       // base coordinate clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> marker_based_clouds;     // marker coordinate clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> marker_based_cut_clouds; // cut clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> joint_based_cut_clouds; // cut clouds

    universalRobots::UR robot;
    pointcloudpreprocess::pre pre;

    float xmin, xmax, ymin, ymax, zmin, zmax;

    marker2base = base2marker.inverse(); // marker to base matrix

    for (int i = 0; n < joint_based_clouds.size(); i++) // save marker_based_clouds
    {
        pcl::transformPointCloud(*joint_based_clouds[i], *marker_based_clouds[i], marker2base);
    }

    for (int i = 0; i < marker_based_clouds.size(); ++i) //cut selected Region
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < marker_based_clouds[i]->size(); ++j)
        {
            const pcl::PointXYZ &point = marker_based_clouds[i]->points[j];
            if (point.x >= xmin && point.x <= xmax &&
                point.y >= ymin && point.y <= ymax &&
                point.z >= zmin && point.z <= zmax)
            {
                cut_cloud->push_back(point);
            }
        }
        marker_based_cut_clouds.push_back(cut_cloud);
    }
    for (int i = 0; i < marker_based_cut_clouds.size(); i++) //transform to base coordinate system
    {
         pcl::transformPointCloud(*marker_based_cut_clouds[i], *joint_based_cut_clouds[i], base2marker);
    }



    return 0;
}
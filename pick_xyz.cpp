/*Pick point and get xyz position. not completed*/

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <chrono>
#include <thread>

typedef pcl::PointXYZ PointType;

void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent &event, void *viewer_void)
{
    float x, y, z;
    if (event.getPointIndex() != -1)
    {
        event.getPoint(x, y, z);
        std::cout << "Picked point at: " << x << ", " << y << ", " << z << std::endl;
    }
}

int main()
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::io::loadPCDFile("shampoo_1.pcd", *cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"cloud");
    viewer->registerPointPickingCallback(pointPickingEventOccurred, (void *)&viewer);


    while (!viewer->wasStopped())
    {
         viewer->spinOnce();  // 시각화 창 업데이트
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    return 0;
}

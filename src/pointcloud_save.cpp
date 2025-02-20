#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sstream>
#include <iomanip> // 파일 번호를 위한 std::setw, std::setfill

// 전역 변수로 파일 번호를 관리
int file_counter = 0;

// 포인트 클라우드를 PCD 파일로 저장하는 함수
void savePointCloudToPCD(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, ros::Subscriber &sub, const std::string &file_path)
{
    // PCL 포맷으로 변환
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // 파일명 생성 (cloud_00.pcd, cloud_01.pcd 등 번호 붙이기)
    std::stringstream ss;
    ss << file_path << std::setw(2) << std::setfill('0') << file_counter << ".pcd"; // 파일 경로 추가

    // PCD 파일 저장
    if (pcl::io::savePCDFileASCII(ss.str(), cloud) == 0)
    {
        ROS_INFO("Saved PCD file: %s", ss.str().c_str());
        file_counter++; // 다음 파일 저장을 위해 카운터 증가
    }
    else
    {
        ROS_ERROR("Failed to save PCD file.");
    }

    // 구독 중지 (한 번만 저장하기 위해)
    sub.shutdown();
}

// 명령 수신 후 PCD 파일을 저장하는 함수
void keyboardCallback(const std_msgs::String::ConstPtr &msg, ros::NodeHandle &nh, ros::Subscriber &sub, const std::string &file_path)
{
    if (msg->data == "capture")
    {
        ROS_INFO("Received 'capture' command. Waiting for point cloud...");

        // 포인트 클라우드 토픽을 한 번만 구독하고 PCD 파일로 저장한 후 구독을 중지
        sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1,
                                                     boost::bind(savePointCloudToPCD, _1, boost::ref(sub), file_path));
    }
}

// 문자열 대체 함수
std::string resolve_template(const std::string &param_value, const std::string &file_name)
{
    size_t pos = param_value.find("$(file_name)");
    if (pos != std::string::npos)
    {
        return param_value.substr(0, pos) + file_name + param_value.substr(pos + 12);
    }
    return param_value;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_save_node");
    ros::NodeHandle nh;
    ros::Subscriber sub;
    std::string file_name, file_path;

    if (!nh.getParam("/file_name", file_name))
    {
        ROS_ERROR("Failed to get '/file_name' parameter.");
        return EXIT_FAILURE;
    }

    if (!nh.getParam("/pointcloud_save_node/file_path", file_path))
    {
        ROS_ERROR("Failed to get '/file_path' parameter.");
        return EXIT_FAILURE;
    }

    file_path = resolve_template(file_path, file_name);
    std::cout << file_path << std::endl;
    // 명령 토픽을 구독하여 'capture' 명령을 수신
    ros::Subscriber command_sub = nh.subscribe<std_msgs::String>(
        "keyboard_command", 10, boost::bind(keyboardCallback, _1, boost::ref(nh), boost::ref(sub), file_path));

    ROS_INFO("Waiting for 'capture' command to save point cloud as PCD...");

    ros::spin();

    return 0;
}
#include <iostream>
#include <vector>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = boost::filesystem;

void readPCDFiles(const std::string& path, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds) {
    fs::path directory(path);
    fs::directory_iterator end_iter;

    if (fs::exists(directory) && fs::is_directory(directory)) {
        for (fs::directory_iterator dir_iter(directory); dir_iter != end_iter; ++dir_iter) {
            if (fs::is_regular_file(dir_iter->status()) && dir_iter->path().extension() == ".pcd") {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(dir_iter->path().string(), *cloud) == 0) {
                    clouds.push_back(cloud);
                } else {
                    std::cerr << "Failed to read PCD file: " << dir_iter->path().string() << std::endl;
                }
            }
        }
    } else {
        std::cerr << "Invalid folder path." << std::endl;
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <folderpath>" << std::endl;
        return 1;
    }

    std::string folderPath = argv[1];
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

    readPCDFiles(folderPath, clouds);

    // 출력 예제
    for (const auto& cloud : clouds) {
        std::cout << "Loaded cloud with " << cloud->points.size() << " points." << std::endl;
    }

    return 0;
}

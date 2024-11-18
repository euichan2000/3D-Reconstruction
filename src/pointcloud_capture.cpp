// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>

#define device_num 1

class D435
{
private:
    int sq = 0;
    int decimate_temp;
    int width, height, frame_rate;

    sensor_msgs::PointCloud2 _msg_pointcloud;

    rs2::config config;
    rs2::pipeline pipe;
    std::string serial_no;
    rs2::pointcloud pc;
    rs2::points points;
    rs2::decimation_filter df;

    const rs2::vertex *vertex;

public:
    D435(int decimate_temp_, int width_, int height_, int frame_rate_)
    {
        decimate_temp = decimate_temp_;
        width = width_;
        height = height;
        frame_rate = frame_rate_;
    }

    void render_camera();
    void enable_camera();
    void initializing_IMU();
    int get_data(ros::Publisher &pub);
};

void D435::render_camera()
{
    rs2::pipeline_profile pipeline_profile = pipe.start(config); // camera starts capturing
    rs2::device rs_dev = pipeline_profile.get_device();
    std::cout << "Device Name" << ": " << rs_dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    std::cout << "Firmware Version" << ": " << rs_dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    std::cout << "Serial Number" << ": " << rs_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    serial_no = rs_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::cout << "Product Id" << ": " << rs_dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;
    pipe.stop(); // camera stops capturing
}

void D435::enable_camera()
{
    std::cout << serial_no << std::endl;

    // ì¹´ë©”ë¼ íŒŒì´í”„ë¼ì¸ ì‹œìž‘ (configë¡œ ì„¤ì •ëœ ìŠ¤íŠ¸ë¦¼ ì ìš©)
    rs2::pipeline_profile pipeline_profile = pipe.start(config); // ì¹´ë©”ë¼ ì‹œìž‘ì€ ì—¬ê¸°ì„œ í•œ ë²ˆë§Œ í˜¸ì¶œ

    // ì¹´ë©”ë¼ ìž¥ì¹˜ ì •ë³´ ê°€ì ¸ì˜¤ê¸°
    rs2::device rs_dev = pipeline_profile.get_device();
    std::string device_name = rs_dev.get_info(RS2_CAMERA_INFO_NAME);

    std::cout << "Device Name: " << device_name << std::endl;

    // ì§€ì›ë˜ëŠ” ìŠ¤íŠ¸ë¦¼ í™•ì¸ (Depth ìŠ¤íŠ¸ë¦¼ ì‚¬ìš©)
    auto sensors = rs_dev.query_sensors();
    bool depth_supported = false;

    for (rs2::sensor &sensor : sensors)
    {
        // í•´ë‹¹ ì„¼ì„œê°€ Depth ìŠ¤íŠ¸ë¦¼ì„ ì§€ì›í•˜ëŠ”ì§€ í™•ì¸
        for (rs2::stream_profile &profile : sensor.get_stream_profiles())
        {
            if (profile.stream_type() == RS2_STREAM_DEPTH)
            {
                depth_supported = true;
                break;
            }
        }
    }

    // Depth ìŠ¤íŠ¸ë¦¼ì´ ì§€ì›ë˜ëŠ” ê²½ìš°ì—ë§Œ ì„¤ì •
    if (depth_supported)
    {
        config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, frame_rate);
    }
    else
    {
        std::cerr << "This device does not support Depth stream" << std::endl;
        return;
    }

    std::cout << "Camera started successfully with serial number: " << serial_no << std::endl;
    pipe.stop();
}

void D435::initializing_IMU()
{
    std::cout << "Initializing IMU..." << std::endl;
    auto profile = pipe.start(config, [&](rs2::frame frame)
                              {
        // Cast the frame that arrived to motion frame
        auto motion = frame.as<rs2::motion_frame>();
        
        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get gyro measures
            rs2_vector gyro_data = motion.get_motion_data();
            std::cout << gyro_data << std::endl;
        }
        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get accelerometer measures
            rs2_vector accel_data = motion.get_motion_data();
            std::cout << accel_data << std::endl;
        } });
    pipe.stop();
}

int D435::get_data(ros::Publisher &pub)
{
    try
    {
        // pipe.start(config);
        rs2::pipeline_profile pipeline_profile = pipe.start(config); // camera starts capturing
        auto sensor = pipeline_profile.get_device().first<rs2::depth_sensor>();
        sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
        // // ì¹´ë©”ë¼ ì˜µì…˜ ì„¤ì •

        sensor.set_option(rs2_option::RS2_OPTION_ALTERNATE_IR, 0.0f); // Alternate IR
        // // Apd Temperature - ì„¤ì • ë¶ˆê°€, ì‹¤ì‹œê°„ ê°’
        sensor.set_option(rs2_option::RS2_OPTION_CONFIDENCE_THRESHOLD, 3.0f); // Confidence Threshold

        sensor.set_option(rs2_option::RS2_OPTION_DIGITAL_GAIN, 2.0f);               // Digital Gain
        sensor.set_option(rs2_option::RS2_OPTION_ENABLE_IR_REFLECTIVITY, 0.0f);     // Enable IR Reflectivity
        sensor.set_option(rs2_option::RS2_OPTION_ENABLE_MAX_USABLE_RANGE, 0.0f);    // Enable Max Usable Range
        sensor.set_option(rs2_option::RS2_OPTION_ERROR_POLLING_ENABLED, 1.0f);      // Error Polling Enabled
        sensor.set_option(rs2_option::RS2_OPTION_FRAMES_QUEUE_SIZE, 16.0f);         // Frames Queue Size
        sensor.set_option(rs2_option::RS2_OPTION_FREEFALL_DETECTION_ENABLED, 1.0f); // Freefall Detection Enabled
        sensor.set_option(rs2_option::RS2_OPTION_GLOBAL_TIME_ENABLED, 0.0f);        // Global Time Enabled

        sensor.set_option(rs2_option::RS2_OPTION_INTER_CAM_SYNC_MODE, 0.0f); // Inter Cam Sync Mode
        sensor.set_option(rs2_option::RS2_OPTION_INVALIDATION_BYPASS, 0.0f); // Invalidation Bypass

        sensor.set_option(rs2_option::RS2_OPTION_LASER_POWER, 0.0f); // Laser Power

        sensor.set_option(rs2_option::RS2_OPTION_MIN_DISTANCE, 0.0f); // Min Distance

        sensor.set_option(rs2_option::RS2_OPTION_NOISE_FILTERING, 6.0f);            // Noise Filtering
        sensor.set_option(rs2_option::RS2_OPTION_POST_PROCESSING_SHARPENING, 2.0f); // Post Processing Sharpening
        sensor.set_option(rs2_option::RS2_OPTION_PRE_PROCESSING_SHARPENING, 0.0f);  // Pre Processing Sharpening

        sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, 0.0f); // Visual Preset
        std::cout << "now pipe start" << std::endl;

        df.set_option(rs2_option::RS2_OPTION_FILTER_MAGNITUDE, pow(2, decimate_temp));

        width /= pow(2, decimate_temp), height /= pow(2, decimate_temp);

        sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);

        // auto frames, depth;
        const rs2::vertex *vertex;

        _msg_pointcloud.header.frame_id = "camera_link";
        _msg_pointcloud.width = width * height;
        _msg_pointcloud.height = 1;

        _msg_pointcloud.point_step = 16;
        _msg_pointcloud.row_step = width * height * _msg_pointcloud.point_step;
        _msg_pointcloud.is_dense = false;

        while (ros::ok()) // Application still alive?
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();
            // rs2::frameset frames;

            // auto depth = df.prcess(frames.get_depth_frame());

            // depth = df.process(depth);

            // if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
            // {
            //     rs2_vector gyro_sample = gyro_frame.get_motion_data();
            //     printf("gyrp: (%.3f,%.3f,%.3f)\n", gyro_sample.x, gyro_sample.y, gyro_sample.z);
            // }

            // Generate the pointcloud and texture mappings
            points = pc.calculate(df.process(frames.get_depth_frame()));

            vertex = points.get_vertices();

            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(points.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(_msg_pointcloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(_msg_pointcloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(_msg_pointcloud, "z");

            // std::cout << points.size() << std::endl;
            for (size_t point_idx = 0; point_idx < points.size(); point_idx++, vertex++)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
            _msg_pointcloud.header.seq = sq;
            _msg_pointcloud.header.stamp = ros::Time::now();
            pub.publish(_msg_pointcloud);

            // std::cout << "pointcloud " << sq << " published!!" << std::endl;
            sq++;
        }
        return EXIT_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

int main(int argc, char *argv[])
{
    int decimate_temp = 2;
    int width = 640, height = 480, frame_rate = 30;

    ros::init(argc, argv, "pointcloud_node");
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Rate loop_rate(frame_rate);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 100);

    D435 my_cam(decimate_temp, width, height, frame_rate);

    for (int i = 0; i < device_num; i++)
    {
        my_cam.render_camera();
        my_cam.enable_camera();
        // my_cam.initializing_IMU();
        my_cam.get_data(pub);
    }
}
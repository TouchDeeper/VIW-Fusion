//
// Created by td on 2021/3/8.
//

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>

bool WriteImu(std::string& imu_file, std::string topic, rosbag::Bag& bag, double time_offset);
void WriteFeature(std::string& base_file, std::string topic, rosbag::Bag& bag, double time_offset);
bool WritePose(std::string& pose_file, std::string topic, rosbag::Bag& bag, double time_offset);
bool WriteWheel(std::string& odom_file, std::string topic, rosbag::Bag& bag, double time_offset);
bool ReadFile(std::ifstream &ifs, std::string file_path);
int main(int argc, char **argv) {
    ros::init(argc, argv, "bag_writer");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    rosbag::Bag bag;
    double time_offset = 1.65e9;
    std::string base_path = ros::package::getPath("vins") + "/..";
    std::string txt_base_path = "/home/td/slam/thesis_project/vio_data_simulation/bin";
    bag.open(base_path + "/sim_data/sim.bag", rosbag::bagmode::Write);

    std::string sImu_data_file = txt_base_path + "/imu_pose.txt";
    WriteImu(sImu_data_file,"/sim/imu/data",bag,time_offset);

    std::string sImu_noise_data_file = txt_base_path + "/imu_pose_noise.txt";
    WriteImu(sImu_noise_data_file, "/sim/imu/data_noise", bag, time_offset);

    std::string sWheel_data_file = txt_base_path + "/wheel_pose.txt";
    WriteWheel(sWheel_data_file,"/sim/wheel/data",bag,time_offset);

    std::string sWheel_noise_data_file = txt_base_path + "/wheel_pose_noise.txt";
    WriteWheel(sWheel_noise_data_file, "/sim/wheel/data_noise", bag, time_offset);

    std::string sGroundtruth_file = txt_base_path + "/imu_pose_tum_correspondence_cam.txt";
    WritePose(sGroundtruth_file, "/sim/groundtruth", bag, time_offset);

    std::string sFeature_data_file = txt_base_path + "/keyframe/pixel_all_points";
    WriteFeature(sFeature_data_file, "/sim/cam0/feature", bag, time_offset);

    std::string sFeature_noise_data_file = txt_base_path + "/keyframe/pixel_noise_all_points";
    WriteFeature(sFeature_noise_data_file, "/sim/cam0/feature_noise", bag, time_offset);

    bag.close();

}
bool ReadFile(std::ifstream &ifs, std::string file_path) {
    ifs.open(file_path, std::ifstream::in);
    if(!ifs)
    {
        std::cout << "无法打开文件: " << std::endl << file_path << std::endl << std::endl;
        return false;
    }
    return false;
}
bool WriteImu(std::string& imu_file, std::string topic, rosbag::Bag& bag, double time_offset){
    std::ifstream ifs_imu;
    ifs_imu.open(imu_file.c_str());
    if (!ifs_imu.is_open())
    {
        std::cerr << "Failed to open imu file! " << imu_file << std::endl;
        return false;
    }

    std::string sImu_line;
    double dStampSec = 0.0;
    Eigen::Vector3d vAcc;
    Eigen::Vector3d vGyr;
    while (std::getline(ifs_imu, sImu_line) && !sImu_line.empty()) // read imu data
    {
        std::istringstream ssImuData(sImu_line);
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        sensor_msgs::Imu imu;
        ssImuData >> dStampSec >> q.w() >> q.x() >> q.y() >> q.z() >> t(0) >> t(1) >> t(2) >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
        imu.header.stamp = ros::Time(dStampSec + time_offset);
        imu.angular_velocity.x = vGyr[0];
        imu.angular_velocity.y = vGyr[1];
        imu.angular_velocity.z = vGyr[2];
        imu.linear_acceleration.x = vAcc[0];
        imu.linear_acceleration.y = vAcc[1];
        imu.linear_acceleration.z = vAcc[2];
        std::cout << "Imu t: " << std::fixed << imu.header.stamp.toSec() << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << std::endl;

        bag.write(topic,imu.header.stamp,imu);
    }
    ifs_imu.close();
    return true;
}
bool WriteWheel(std::string& odom_file, std::string topic, rosbag::Bag& bag, double time_offset){
    std::ifstream ifs_odom;
    ifs_odom.open(odom_file.c_str());
    if (!ifs_odom.is_open())
    {
        std::cerr << "Failed to open imu file! " << odom_file << std::endl;
        return false;
    }

    std::string sOdom_line;
    double dStampSec = 0.0;
    Eigen::Vector3d vVel;
    Eigen::Vector3d vGyr;
    while (std::getline(ifs_odom, sOdom_line) && !sOdom_line.empty()) // read imu data
    {
        std::istringstream ssOdomData(sOdom_line);
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        nav_msgs::Odometry odom;
        ssOdomData >> dStampSec >> q.w() >> q.x() >> q.y() >> q.z() >> t(0) >> t(1) >> t(2) >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vVel.x() >> vVel.y() >> vVel.z();
        odom.header.stamp = ros::Time(dStampSec + time_offset);
        odom.twist.twist.angular.x = vGyr[0];
        odom.twist.twist.angular.y = vGyr[1];
        odom.twist.twist.angular.z = vGyr[2];
        odom.twist.twist.linear.x = vVel[0];
        odom.twist.twist.linear.y = vVel[1];
        odom.twist.twist.linear.z = vVel[2];
        std::cout << "Wheel t: " << std::fixed << odom.header.stamp.toSec() << " gyr: " << vGyr.transpose() << " vel: " << vVel.transpose() << std::endl;

        bag.write(topic, odom.header.stamp, odom);
    }
    ifs_odom.close();
    return true;
}
bool WritePose(std::string& pose_file, std::string topic, rosbag::Bag& bag, double time_offset){
    std::ifstream ifs_odometry;
    ifs_odometry.open(pose_file.c_str());
    if (!ifs_odometry.is_open())
    {
        std::cerr << "Failed to open odometry file! " << pose_file << std::endl;
        return false;
    }

    std::string sOdom_line;
    double dStampSec = 0.0;
    while (std::getline(ifs_odometry, sOdom_line) && !sOdom_line.empty()) // read odom data
    {
        std::istringstream ssOdomData(sOdom_line);
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        geometry_msgs::PoseStamped pose;
        ssOdomData >> dStampSec >> t(0) >> t(1) >> t(2) >>  q.x() >> q.y() >> q.z() >> q.w();
        pose.header.stamp = ros::Time(dStampSec + time_offset);
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.position.x = t[0];
        pose.pose.position.y = t[1];
        pose.pose.position.z = t[2];
        std::cout << "Pose t: " << std::fixed << pose.header.stamp.toSec() << " orientation: " << q.coeffs().transpose() << " position: " << t.transpose() << std::endl;

        bag.write(topic, pose.header.stamp, pose);
    }
    ifs_odometry.close();
    return true;
}
void WriteFeature(std::string& base_file, std::string topic, rosbag::Bag& bag, double time_offset){
    int i=0;
    while (true) {
        std::string feature_file = base_file + "_" + std::to_string(i) + ".txt";
        std::ifstream ifs_feature;
        ifs_feature.open(feature_file.c_str());
        if (!ifs_feature.is_open())
        {
            std::cerr << "Failed to open feature file! " << feature_file << std::endl;
            break;
        }
        std::string sFeature_line;
        double dStampSec = 0.0;
        Eigen::Vector3d vAcc;
        Eigen::Vector3d vGyr;
        sensor_msgs::PointCloud features;
        while (std::getline(ifs_feature, sFeature_line) && !sFeature_line.empty()) // read imu data
        {
            std::istringstream ssFeatureData(sFeature_line);
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            geometry_msgs::Point32_<std::allocator<void>> feature;
            Eigen::Vector4d point;//useless
            ssFeatureData >> point(0) >> point(1) >> point(2) >> point(3) >> feature.x >> feature.y >> dStampSec;
            features.points.push_back(feature);
        }
        features.header.stamp = ros::Time(dStampSec+time_offset);
        bag.write(topic, features.header.stamp, features);
        std::cout << "features t: " << std::fixed << features.header.stamp.toSec() << std::endl;
        ifs_feature.close();
        i++;
    }
}
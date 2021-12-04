/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/plane_factor.h"
#include "../factor/wheel_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/orientation_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"


class Estimator
{
  public:
    Estimator();
    ~Estimator();
    void setParameter();

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void inputWheel(double t, const Vector3d &linearVelocity, const Vector3d &angularVelocity);
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void inputFeature(double t, const vector<cv::Point2f>& _features0, const vector<cv::Point2f>& _features1=vector<cv::Point2f>());//仿真的feature
    void inputGroundtruth(double t, Eigen::Matrix<double, 7, 1>& data);
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processWheel(double t, double dt, const Vector3d &linear_velocity, const Vector3d &angular_velocity);
    void integrateWheelPreintegration( double t, Eigen::Vector3d& P, Eigen::Quaterniond& Q, const Eigen::Matrix<double, 7, 1>& pose);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);
    void processMeasurements();
    void changeSensorType(int use_imu, int use_stereo);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    bool getWheelInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &velVector,
                                     vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates();
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    void fastPredictWheel(double t, Eigen::Vector3d linear_velocity, Eigen::Vector3d angular_velocity);
    void fastPredictPureWheel(double t, Eigen::Vector3d linear_velocity, Eigen::Vector3d angular_velocity, Eigen::Vector3d &P, Eigen::Quaterniond &Q, Eigen::Vector3d &V);
    bool IMUAvailable(double t);
    bool WheelAvailable(double t);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
    void initPlane();

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    std::mutex mProcess;
    std::mutex mBuf;
    std::mutex mGTBuf;
    std::mutex mWheelBuf;
    std::mutex mPropagate;
    std::mutex mWheelPropagate;
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> wheelVelBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
    queue<pair<double, Eigen::Vector3d>> wheelGyrBuf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
    queue<pair<double, Eigen::Matrix<double,7,1>>> groundtruthBuf;
    double prevTime, curTime;
    double prevTime_wheel, curTime_wheel;
    bool openExEstimation;
    bool openExWheelEstimation;
    bool openIxEstimation;
    bool openPlaneEstimation;
    std::thread trackThread;
    std::thread processThread;

    FeatureTracker featureTracker;

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;

    Matrix3d ric[2]; //存储双目与imu之间的外参 ric[0] = R_i_cl; ric[1] = R_i_cr;
    Vector3d tic[2]; //tic[0] = t_i_cl; tic[1] = t_i_cr;
    Matrix3d rio;
    Vector3d tio;

    //平面参数
    Matrix3d rpw;
    double zpw;

    double sx = 1, sy = 1, sw = 1;
    Vector3d        Ps[(WINDOW_SIZE + 1)];
    Vector3d        Vs[(WINDOW_SIZE + 1)];
    Matrix3d        Rs[(WINDOW_SIZE + 1)];
    Vector3d        Bas[(WINDOW_SIZE + 1)];
    Vector3d        Bgs[(WINDOW_SIZE + 1)];
    double td;
    double td_wheel;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)]; //滑窗中图像帧的时间戳

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    WheelIntegrationBase *pre_integrations_wheel[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    Vector3d vel_0_wheel, gyr_0_wheel;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    vector<double> dt_buf_wheel[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_velocity_buf_wheel[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf_wheel[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool first_wheel;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[2][SIZE_POSE];
    double para_Ex_Pose_wheel[1][SIZE_POSE];
    double para_plane_R[1][SIZE_ROTATION];
    double para_plane_Z[1][1];
    double para_Ix_sx_wheel[1][1];
    double para_Ix_sy_wheel[1][1];
    double para_Ix_sw_wheel[1][1];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Td_wheel[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks; //保存上一次边缘化后，保留的状态向量的内存地址

    map<double, ImageFrame> all_image_frame; //存储所有的图像帧数据
    IntegrationBase *tmp_pre_integration;
    WheelIntegrationBase *tmp_wheel_pre_integration;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;
    //IMU
    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;
    //Wheel
    double latest_time_wheel;
    Eigen::Vector3d latest_P_wheel, latest_V_wheel, latest_vel_wheel_0, latest_gyr_wheel_0;
    double latest_sx, latest_sy, latest_sw;
    Eigen::Quaterniond latest_Q_wheel;

    bool initFirstPoseFlag; //标记位姿是否初始化
    bool initThreadFlag;
};

/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include "../utility/utility.h"
#include "../utility/sophus_utils.hpp"
#include "../estimator/parameters.h"

#include <ceres/ceres.h>
using namespace Eigen;

class WheelIntegrationBase
{
  public:
    WheelIntegrationBase() = delete;
    WheelIntegrationBase(const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_gyr_0,
            const double &_linearized_sx, const double &_linearized_sy, const double &_linearized_sw, const double &_linearized_td)
        : vel_0{_vel_0}, gyr_0{_gyr_0}, linearized_vel{_vel_0}, linearized_gyr{_gyr_0},
          linearized_sx{_linearized_sx}, linearized_sy{_linearized_sy}, linearized_sw{_linearized_sw}, linearized_td{_linearized_td},
          jacobian{Eigen::Matrix<double, 6, 3>::Zero()}, covariance{Eigen::Matrix<double, 6, 6>::Zero()},
          sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}

    {

        noise = Eigen::Matrix<double, 12, 12>::Zero();
        noise.block<3, 3>(0, 0) = (VEL_N_wheel * VEL_N_wheel) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) = (GYR_N_wheel * GYR_N_wheel) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) = (VEL_N_wheel * VEL_N_wheel) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) = (GYR_N_wheel * GYR_N_wheel) * Eigen::Matrix3d::Identity();

        step_jacobian_ix.setZero();
    }

    void push_back(double dt, const Eigen::Vector3d &vel, const Eigen::Vector3d &gyr)
    {
        dt_buf.push_back(dt);
        vel_buf.push_back(vel);
        gyr_buf.push_back(gyr);
        propagate(dt, vel, gyr);
//        checkIntrinsicUpdateJacobian();
    }

    void repropagate(const double &_linearized_sx, const double &_linearized_sy, const double &_linearized_sw)
    {
        sum_dt = 0.0;
        vel_0 = linearized_vel;
        gyr_0 = linearized_gyr;
        delta_p.setZero();
        delta_q.setIdentity();
//        delta_v.setZero();
        linearized_sx = _linearized_sx;
        linearized_sy = _linearized_sy;
        linearized_sw = _linearized_sw;
        jacobian.setZero();
        covariance.setZero();
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], vel_buf[i], gyr_buf[i]);
    }
    //预积分，且进行了预积分的协方差累计，进行了预积分相对bias的变化的变化雅可比的计算
    void midPointIntegration(double _dt,
                             const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_gyr_0,
                             const Eigen::Vector3d &_vel_1, const Eigen::Vector3d &_gyr_1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
                             const double &linearized_sx, const double &linearized_sy, const double &linearized_sw,
                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q,
                             double &result_linearized_sx, double &result_linearized_sy, double &result_linearized_sw, bool update_jacobian)
    {
        //ROS_INFO("midpoint integration");

        Eigen::Matrix3d sv = Eigen::Vector3d(linearized_sx, linearized_sy, 1).asDiagonal();
        Vector3d un_vel_0 = delta_q * sv * _vel_0;
        Vector3d un_gyr = 0.5 * linearized_sw * (_gyr_0 + _gyr_1);
        Eigen::Quaterniond delta_delta_q = Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
        result_delta_q = delta_q * delta_delta_q;
        Vector3d un_vel_1 = result_delta_q * sv * _vel_1;
        Vector3d un_vel = 0.5 * (un_vel_0 + un_vel_1);
        result_delta_p = delta_p + un_vel * _dt;



        result_linearized_sx = linearized_sx;
        result_linearized_sy = linearized_sy;
        result_linearized_sw = linearized_sw;

        if(update_jacobian)
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1)*linearized_sw;
            Vector3d vel_0_x = sv * _vel_0;
            Vector3d vel_1_x = sv * _vel_1;
            Matrix3d R_w_x, R_vel_0_x, R_vel_1_x;

            R_w_x<<0, -w_x(2), w_x(1),
                w_x(2), 0, -w_x(0),
                -w_x(1), w_x(0), 0;
            R_vel_0_x << 0, -vel_0_x(2), vel_0_x(1),
                vel_0_x(2), 0, -vel_0_x(0),
                -vel_0_x(1), vel_0_x(0), 0;
            R_vel_1_x << 0, -vel_1_x(2), vel_1_x(1),
                vel_1_x(2), 0, -vel_1_x(0),
                -vel_1_x(1), vel_1_x(0), 0;

            MatrixXd F = MatrixXd::Zero(6, 6);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.5 * _dt * (delta_q.toRotationMatrix() * R_vel_0_x + result_delta_q.toRotationMatrix() * R_vel_1_x * delta_delta_q.toRotationMatrix().transpose());
            F.block<3, 3>(3, 3) = delta_delta_q.toRotationMatrix().transpose();
            //cout<<"A"<<endl<<A<<endl;
            Eigen::Matrix3d Jr;
            Sophus::rightJacobianSO3(un_gyr * _dt, Jr);

            MatrixXd V = MatrixXd::Zero(6,12);
            V.block<3, 3>(0, 0) =  0.5 * _dt * delta_q.toRotationMatrix() * sv ;
            V.block<3, 3>(0, 3) =-0.25 * _dt * _dt * result_delta_q.toRotationMatrix() * R_vel_1_x * Jr;
            V.block<3, 3>(0, 6) =  0.5 * _dt * result_delta_q.toRotationMatrix() * sv;
            V.block<3, 3>(0, 9) =-0.25 * _dt * _dt * result_delta_q.toRotationMatrix() * R_vel_1_x * Jr ;
            V.block<3, 3>(3, 3) =  0.5 * Jr * linearized_sw * _dt;
            V.block<3, 3>(3, 9) =  0.5 * Jr * linearized_sw * _dt;

//            step_jacobian = F;
//            step_V = V;
            Eigen::Matrix3d I1 = Eigen::Vector3d(1,0,0).asDiagonal();
            Eigen::Matrix3d I2 = Eigen::Vector3d(0,1,0).asDiagonal();
//            step_jacobian_ix.block<3,1>(0,0) = 0.5 * delta_q.toRotationMatrix() * (I1* _vel_0 + delta_delta_q.toRotationMatrix() * I1 * _vel_1) * _dt;
//            step_jacobian_ix.block<3,1>(0,1) = 0.5 * delta_q.toRotationMatrix() * (I2* _vel_0 + delta_delta_q.toRotationMatrix() * I2 * _vel_1) * _dt;
//            step_jacobian_ix.block<3,1>(3,2) = Jr * 0.5 * (_gyr_0 + _gyr_1) * _dt;
//            step_jacobian_ix.block<3,1>(0,2) = 0.5 * result_delta_q.toRotationMatrix() * Utility::skewSymmetric(Jr * 0.5 * (_gyr_0 + _gyr_1) * _dt) * sv * _vel_1 * _dt;

            jacobian.block<3,1>(0,0) = jacobian.block<3,1>(0,0).eval() + 0.5 * (delta_q.toRotationMatrix() * I1 * _vel_0 + result_delta_q.toRotationMatrix() * I1 * _vel_1) * _dt;
            jacobian.block<3,1>(0,1) = jacobian.block<3,1>(0,1).eval() + 0.5 * (delta_q.toRotationMatrix() * I2 * _vel_0 + result_delta_q.toRotationMatrix() * I2 * _vel_1) * _dt;
            Eigen::Vector3d dr_dsw_last = jacobian.block<3,1>(3,2);
            jacobian.block<3,1>(3,2) = jacobian.block<3,1>(3,2).eval() + Jr * 0.5 * (_gyr_0 + _gyr_1) * _dt;
            jacobian.block<3,1>(0,2) = jacobian.block<3,1>(0,2).eval() + 0.5 * (delta_q.toRotationMatrix()  * Utility::skewSymmetric(dr_dsw_last) * sv * _vel_0
            + result_delta_q.toRotationMatrix() * Utility::skewSymmetric(jacobian.block<3,1>(3,2)) * sv * _vel_1) * _dt;

//            jacobian.setZero();
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();//协方差传递
        }

    }

    void propagate(double _dt, const Eigen::Vector3d &_vel_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        vel_1 = _vel_1;
        gyr_1 = _gyr_1;
        Vector3d result_delta_p;
        Quaterniond result_delta_q;
//        Vector3d result_delta_v;
        double result_linearized_sx;
        double result_linearized_sy;
        double result_linearized_sw;

        midPointIntegration(_dt, vel_0, gyr_0, _vel_1, _gyr_1, delta_p, delta_q,linearized_sx, linearized_sy, linearized_sw,
                            result_delta_p, result_delta_q, result_linearized_sx, result_linearized_sy, result_linearized_sw,
                            1);

//        checkJacobian(_dt, vel_0, gyr_0, vel_1, gyr_1, delta_p, delta_q,
//                      linearized_sx, linearized_sy, linearized_sw);

        delta_p = result_delta_p;
        delta_q = result_delta_q;
//        delta_v = result_delta_v;
        linearized_sx = result_linearized_sx;
        linearized_sy = result_linearized_sy;
        linearized_sw = result_linearized_sw;
        delta_q.normalize();
        sum_dt += dt;
        vel_0 = vel_1;
        gyr_0 = gyr_1;  
     
    }
    //evaulate的参数应该包含所有与该因子相连的节点
    Eigen::Matrix<double, 6, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Quaterniond &qio, const Eigen::Vector3d &tio, const double sx, const double sy, const double sw,
                                         const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const double td)
    {
        Eigen::Matrix<double, 6, 1> residuals;

        Eigen::Vector3d dp_dsx = jacobian.block<3, 1>(0, 0);
        Eigen::Vector3d dp_dsy = jacobian.block<3, 1>(0, 1);
        Eigen::Vector3d dp_dsw = jacobian.block<3, 1>(0, 2);
//        Eigen::Vector3d dq_dsx = jacobian.block<3, 1>(3, 0);
//        Eigen::Vector3d dq_dsy = jacobian.block<3, 1>(3, 1);
        Eigen::Vector3d dq_dsw = jacobian.block<3, 1>(3, 2);

        double dsx = sx - linearized_sx;
        double dsy = sy - linearized_sy;
        double dsw = sw - linearized_sw;
        Eigen::Matrix3d sv = Eigen::Vector3d(sx, sy, 1).asDiagonal();
//        ROS_INFO("dsx: %f, dsy: &f, dsw: %f", dsx, dsy, dsw);
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d rio = qio.toRotationMatrix();

        corrected_delta_p = delta_p + dp_dsx * dsx + dp_dsy * dsy + dp_dsw * dsw ;
        corrected_delta_q = (Sophus::SO3d(delta_q) * Sophus::SO3d::exp(dq_dsw * dsw)).unit_quaternion();
        double dtd = td - linearized_td;
//        std::cout<<"linearized_td: "<<linearized_td<<" dtd:"<<dtd<<std::endl;
        Eigen::Quaterniond delta_q_time = (Sophus::SO3d::exp(sw * linearized_gyr * dtd) * Sophus::SO3d(corrected_delta_q) * Sophus::SO3d::exp(-sw * gyr_1 * dtd)).unit_quaternion();
        Eigen::Vector3d delta_p_time = Sophus::SO3d::exp(sw * linearized_gyr * dtd).matrix() * ( sv * linearized_vel * dtd + corrected_delta_p - corrected_delta_q * sv * vel_1 * dtd);

//        residuals.block<3, 1>(O_P, 0) = (Ri * rio).transpose() * (Rj * tio + Pj - Ri * tio - Pi) - corrected_delta_p;
//        residuals.block<3, 1>(O_R, 0) =  Sophus::SO3d( corrected_delta_q.inverse() * (Qi * qio).inverse() * Qj * qio).log();

        residuals.block<3, 1>(O_P, 0) = (Ri * rio).transpose() * (Rj * tio + Pj - Ri * tio - Pi) - delta_p_time;
        residuals.block<3, 1>(O_R, 0) =  Sophus::SO3d( delta_q_time.inverse() * (Qi * qio).inverse() * Qj * qio).log();
//        std::cout<<"wheel residuals: "<<residuals.transpose()<<std::endl;


//        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
//        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }

    double dt;
    Eigen::Vector3d vel_0, gyr_0;
    Eigen::Vector3d vel_1, gyr_1;

    const Eigen::Vector3d linearized_vel, linearized_gyr;//预积分起始轮速数据
    double linearized_sx, linearized_sy, linearized_sw;
    double linearized_td;
    Eigen::Matrix<double, 6, 3> jacobian;
    Eigen::Matrix<double, 6, 3> step_jacobian_ix;//轮速内参的step_jacobian
    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Matrix<double, 6, 6> step_jacobian;

    Eigen::Matrix<double, 6, 12> step_V;
    Eigen::Matrix<double, 12, 12> noise;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d corrected_delta_p;
    Eigen::Quaterniond corrected_delta_q;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> vel_buf;
    std::vector<Eigen::Vector3d> gyr_buf;

};
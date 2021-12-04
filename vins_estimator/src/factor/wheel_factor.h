/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "wheel_integration_base.h"
#include "../utility/sophus_utils.hpp"
#include <ceres/ceres.h>

class WheelFactor : public ceres::SizedCostFunction<6, 7, 7, 7, 1, 1, 1, 1>
{
  public:
    WheelFactor() = delete;
    WheelFactor(WheelIntegrationBase* _pre_integration): pre_integration(_pre_integration)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

//        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
//        Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
//        Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d tio(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qio(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        double sx = parameters[3][0];
        double sy = parameters[4][0];
        double sw = parameters[5][0];

        Eigen::Matrix3d sv = Eigen::Vector3d(sx, sy, 1).asDiagonal();
//        double td;
        double td = parameters[6][0];

//        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
//        Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
//        Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

//Eigen::Matrix<double, 15, 15> Fd;
//Eigen::Matrix<double, 15, 12> Gd;

//Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
//Eigen::Quaterniond pQj = Qi * delta_q;
//Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
//Eigen::Vector3d pBaj = Bai;
//Eigen::Vector3d pBgj = Bgi;

//Vi + Qi * delta_v - g * sum_dt = Vj;
//Qi * delta_q = Qj;

//delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
//delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
//delta_q = Qi.inverse() * Qj;

#if 0
        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
#endif

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        residual = pre_integration->evaluate(Pi, Qi, qio, tio, sx, sy, sw, Pj, Qj, td);

        Eigen::Matrix<double, 6, 1> raw_residual = residual;

        Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(pre_integration->covariance.inverse()).matrixL().transpose();
//        sqrt_info.setIdentity();
//        std::cout<<"sqrt_info :\n"<<sqrt_info<<std::endl;
        residual = sqrt_info * residual;

        if (jacobians)
        {
//            double sum_dt = pre_integration->sum_dt;
            Eigen::Vector3d dp_dsx = pre_integration->jacobian.template block<3, 1>(O_P, 0);
            Eigen::Vector3d dp_dsy = pre_integration->jacobian.template block<3, 1>(O_P, 1);
            Eigen::Vector3d dp_dsw = pre_integration->jacobian.template block<3, 1>(O_P, 2);
//            Eigen::Vector3d dq_dsx = pre_integration->jacobian.template block<3, 1>(O_R, 0);
//            Eigen::Vector3d dq_dsy = pre_integration->jacobian.template block<3, 1>(O_R, 1);
            Eigen::Vector3d dq_dsw = pre_integration->jacobian.template block<3, 1>(O_R, 2);
            double dtd = td - pre_integration->linearized_td;
            if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
            {
                ROS_WARN("numerical unstable in preintegration");
                //std::cout << pre_integration->jacobian << std::endl;
///                ROS_BREAK();
            }

            Eigen::Vector3d raw_residual_r = raw_residual.block<3,1>(3,0);
            Eigen::Matrix3d Jr_delta_q_inv;
            Sophus::rightJacobianInvSO3(raw_residual_r, Jr_delta_q_inv);

            Eigen::Vector3d drdsw = dq_dsw * (sw - pre_integration->linearized_sw);
            Eigen::Matrix3d Jr_drdsw;
            Sophus::rightJacobianSO3(drdsw,Jr_drdsw);

            Eigen::Matrix3d ri = Qi.toRotationMatrix();
            Eigen::Matrix3d rj = Qj.toRotationMatrix();
            Eigen::Matrix3d rio = qio.toRotationMatrix();
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]); //jacobians[i] is a row-major array of size num_residuals x parameter_block_sizes_[i]
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -(Qi * qio).inverse().toRotationMatrix();
//                jacobian_pose_i.block<3, 3>(O_P, O_R) = (Qi * qio).inverse().toRotationMatrix() + qio.inverse().toRotationMatrix() * Utility::skewSymmetric(Qi.inverse() * (Qj * tio + Pj - Qi * tio - Pi));
                jacobian_pose_i.block<3, 3>(O_P, O_R) = (ri * rio).transpose() * (ri * Utility::skewSymmetric(tio)) + rio.transpose() * Utility::skewSymmetric(ri.transpose() * (rj * tio + Pj - ri * tio - Pi));

#if 0
            jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else


//                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dsx * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -Jr_delta_q_inv * ((Qj * qio).inverse() * Qi).toRotationMatrix();
#endif

//                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in preintegration");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = (Qi * qio).inverse().toRotationMatrix();
                jacobian_pose_j.block<3, 3>(O_P, O_R) = -((Qi * qio).inverse() * Qj).toRotationMatrix() * Utility::skewSymmetric(tio);

#if 0
            jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
//                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dsx * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) =   Jr_delta_q_inv * (qio.inverse()).toRotationMatrix();
#endif

                jacobian_pose_j = sqrt_info * jacobian_pose_j;

//                ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
//                ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
                jacobian_ex_pose.setZero();

                jacobian_ex_pose.block<3, 3>(O_P, O_P) = (Qi * qio).inverse().toRotationMatrix() * (Qj.toRotationMatrix() - Qi.toRotationMatrix());

                jacobian_ex_pose.block<3, 3>(O_P, O_R) = Utility::skewSymmetric((Qi * qio).inverse() * (Qj * tio + Pj - Qi* tio - Pi));

                jacobian_ex_pose.block<3, 3>(O_R, O_R) = Jr_delta_q_inv * (Eigen::Matrix3d::Identity() - ((Qj * qio).inverse() * Qi * qio).toRotationMatrix());

                jacobian_ex_pose = sqrt_info * jacobian_ex_pose;

//                ROS_ASSERT(fabs(jacobian_ex_pose.maxCoeff()) < 1e8);
//                ROS_ASSERT(fabs(jacobian_ex_pose.minCoeff()) < 1e8);
            }
            Eigen::Vector3d forward_compensate_w = sw * pre_integration->linearized_gyr * dtd;
            Eigen::Vector3d forward_compensate_v = sv * pre_integration->linearized_vel * dtd;
            Eigen::Vector3d back_compensate_v = sv * pre_integration->vel_1 * dtd;
            Eigen::Vector3d back_compensate_w = sw * pre_integration->gyr_1 * dtd;

            Eigen::Matrix3d Jrtd;
            Eigen::Matrix3d Jr_minus_td;
            Sophus::rightJacobianSO3(forward_compensate_w, Jrtd);
            Sophus::rightJacobianSO3(-forward_compensate_w, Jr_minus_td);
            Eigen::Matrix3d I1 = Eigen::Vector3d(1, 0, 0).asDiagonal();
            Eigen::Matrix3d I2 = Eigen::Vector3d(0, 1, 0).asDiagonal();
            if(jacobians[3]){
                Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_ix_sx(jacobians[3]);
                jacobian_ix_sx.setZero();

                jacobian_ix_sx.block<3, 1>(O_P, 0) = - Sophus::SO3d::exp(forward_compensate_v).matrix() * (I1 * pre_integration->linearized_vel * dtd + dp_dsx - pre_integration->corrected_delta_q.toRotationMatrix() * I1 * pre_integration->vel_1 * dtd);


                jacobian_ix_sx = sqrt_info * jacobian_ix_sx;

//                ROS_ASSERT(fabs(jacobian_ix_sx.maxCoeff()) < 1e8);
//                ROS_ASSERT(fabs(jacobian_ix_sx.minCoeff()) < 1e8);
            }
            if(jacobians[4]){
                Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_ix_sy(jacobians[4]);
                jacobian_ix_sy.setZero();

                jacobian_ix_sy.block<3, 1>(O_P, 0) = - Sophus::SO3d::exp(forward_compensate_v).matrix() * (I2 * pre_integration->linearized_vel * dtd + dp_dsy - pre_integration->corrected_delta_q.toRotationMatrix() * I2 * pre_integration->vel_1 * dtd);


                jacobian_ix_sy = sqrt_info * jacobian_ix_sy;

//                ROS_ASSERT(fabs(jacobian_ix_sy.maxCoeff()) < 1e8);
//                ROS_ASSERT(fabs(jacobian_ix_sy.minCoeff()) < 1e8);
            }
            if(jacobians[5]){
                Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_ix_sw(jacobians[5]);
                jacobian_ix_sw.setZero();

                jacobian_ix_sw.block<3, 1>(O_P, 0) = - Sophus::SO3d::exp(forward_compensate_w).matrix() * (dp_dsw - pre_integration->corrected_delta_q.toRotationMatrix() * Utility::skewSymmetric(Jr_drdsw * dq_dsw) * sv * pre_integration->vel_1 * dtd + Utility::skewSymmetric(Jrtd * pre_integration->linearized_gyr * dtd) * (forward_compensate_v + pre_integration->corrected_delta_p - pre_integration->corrected_delta_q * back_compensate_v));

                jacobian_ix_sw.block<3, 1>(O_R, 0) = - Jr_delta_q_inv * Sophus::SO3d::exp(-raw_residual_r).matrix() * Sophus::SO3d::exp(back_compensate_w).matrix() *( pre_integration->corrected_delta_q.inverse().toRotationMatrix() *  Jrtd * pre_integration->linearized_gyr * dtd + Jr_drdsw * dq_dsw);


                jacobian_ix_sw = sqrt_info * jacobian_ix_sw;

//                ROS_ASSERT(fabs(jacobian_ix_sw.maxCoeff()) < 1e8);
//                ROS_ASSERT(fabs(jacobian_ix_sw.minCoeff()) < 1e8);
            }
            //对于加入的参数，就算setConstant，这里也要赋值，不然会有nan
            if(jacobians[6]){
                Eigen::Map<Eigen::Matrix<double, 6, 1>> jacobian_td(jacobians[6]);
                jacobian_td.block<3,1>(O_P, 0) = - Sophus::SO3d::exp(forward_compensate_w).matrix() * (sv * pre_integration->linearized_vel - pre_integration->corrected_delta_q.toRotationMatrix() * sv * pre_integration->vel_1 + Utility::skewSymmetric(Jrtd * sw * pre_integration->linearized_gyr) *  (forward_compensate_v + pre_integration->corrected_delta_p - pre_integration->corrected_delta_q.toRotationMatrix() * back_compensate_v));
                jacobian_td.block<3,1>(O_R,0) = -Jr_delta_q_inv * Sophus::SO3d::exp(-raw_residual_r).matrix() * (Sophus::SO3d::exp(back_compensate_w).matrix() * pre_integration->corrected_delta_q.inverse().toRotationMatrix() * Jrtd * sw * pre_integration->linearized_gyr - Jr_minus_td * sw * pre_integration->gyr_1);

                jacobian_td = sqrt_info * jacobian_td;

//                ROS_ASSERT(fabs(jacobian_td.maxCoeff()) < 1e8);
//                ROS_ASSERT(fabs(jacobian_td.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    WheelIntegrationBase* pre_integration;

};


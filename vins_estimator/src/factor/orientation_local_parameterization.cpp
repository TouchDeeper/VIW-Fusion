/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "orientation_local_parameterization.h"

bool OrientationLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Quaterniond> _q(x);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta));

    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);

    q = (_q * dq).normalized();

    return true;
}
bool OrientationLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);//关于该jacobian的说明参加 https://fzheng.me/2018/01/23/ba-demo-ceres/
    j.topRows<3>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

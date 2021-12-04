/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "orientation_subset_parameterization.h"
OrientationSubsetParameterization::OrientationSubsetParameterization(const std::vector<int>& constant_parameters):constancy_mask_(4, 0){
    if(constant_parameters.empty())
        return;
    std::vector<int> constant = constant_parameters;
    std::sort(constant.begin(), constant.end());
//    CHECK_GE(constant.front(), 0)
//        << "Indices indicating constant parameter must be greater than zero.";
//    CHECK_LT(constant.back(), 4)
//        << "Indices indicating constant parameter must be less than the size "
//        << "of the parameter block.";
//    CHECK(std::adjacent_find(constant.begin(), constant.end()) == constant.end())
//    << "The set of constant parameters cannot contain duplicates";
    for (size_t i = 0; i < constant_parameters.size(); ++i) {
        constancy_mask_[constant_parameters[i]] = 1;
    }
}
bool OrientationSubsetParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Quaterniond> _q(x);

    double delta_[LocalSize()];
    mempcpy(delta_, delta, sizeof(double) * LocalSize());
    for (int i = 0; i < 3; ++i) {
        delta_[i] = constancy_mask_[i] ? 0 : delta_[i];
    }

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta_));

    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);

    q = (_q * dq).normalized();

    return true;
}
bool OrientationSubsetParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);//关于该jacobian的说明参加 https://fzheng.me/2018/01/23/ba-demo-ceres/
    j.topRows<3>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

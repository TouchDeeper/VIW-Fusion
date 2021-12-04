/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose_subset_parameterization.h"
PoseSubsetParameterization::PoseSubsetParameterization(const std::vector<int>& constant_parameters):constancy_mask_(7, 0){
    if(constant_parameters.empty())
        return;
    std::vector<int> constant = constant_parameters;
    std::sort(constant.begin(), constant.end());
//    CHECK_GE(constant.front(), 0)
//        << "Indices indicating constant parameter must be greater than zero.";
//    CHECK_LT(constant.back(), 7)
//        << "Indices indicating constant parameter must be less than the size "
//        << "of the parameter block.";
//    CHECK(std::adjacent_find(constant.begin(), constant.end()) == constant.end())
//    << "The set of constant parameters cannot contain duplicates";
    for (size_t i = 0; i < constant_parameters.size(); ++i) {
        constancy_mask_[constant_parameters[i]] = 1;
    }
}
bool PoseSubsetParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    double delta_[LocalSize()];
    mempcpy(delta_, delta, sizeof(double)*LocalSize());
    for (int i = 0; i < 6; ++i) {
        delta_[i] = constancy_mask_[i] ? 0 : delta_[i];
    }
//    for (int j = 0; j < 6; ++j) {
//        std::cout<<delta[j]<<" ";
//    }
//    std::cout<<std::endl;
//    for (int j = 0; j < 6; ++j) {
//        std::cout<<delta_[j]<<" ";
//    }
//    std::cout<<std::endl;
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta_);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta_ + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}
bool PoseSubsetParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

const int NUM_THREADS = 4; //边缘化过程中构建A、b的线程数目

struct ResidualBlockInfo //模拟ceres中的costfunction的操作，主要完成残差与雅克比计算
{
    ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    void Evaluate();

    ceres::CostFunction *cost_function;
    ceres::LossFunction *loss_function;
    std::vector<double *> parameter_blocks; //与该残差相关的优化变量数据
    std::vector<int> drop_set; //待边缘化的优化变量ID（PS:待边缘化的优化变量是与残差相关的优化变量数据的子集）

    double **raw_jacobians; //雅克比
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals; //残差向量：预积分——15x1； 视觉——2x1

    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
};

struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::unordered_map<long, int> parameter_block_size; //global size
    std::unordered_map<long, int> parameter_block_idx; //local size
};

class MarginalizationInfo
{
  public:
    MarginalizationInfo(){valid = true;};
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    void preMarginalize();
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

    std::vector<ResidualBlockInfo *> factors; //所有观测量
    int m, n;//m表征需要边缘化的变量的localSize和，n表征保留的变量的localSize和， 二者均以localSize计算表示
    //下述unordered_map关键字long使用的是状态向量的内存地址
    std::unordered_map<long, int> parameter_block_size; //global size 存储每个状态向量的尺寸 <变量的内存地址，变量的localSize>
    int sum_block_size;
    std::unordered_map<long, int> parameter_block_idx; //local size 存储每个状态向量的索引。<变量的内存地址， 构建矩阵的索引（结果是前m为边缘化部分，后n为保留部分）>
    std::unordered_map<long, double *> parameter_block_data; //存储状态向量的数据 <变量的内存地址， 变量数据>
    // 存储边缘化后最终保留的量
    std::vector<int> keep_block_size; //global size
    std::vector<int> keep_block_idx;  //local size
    std::vector<double *> keep_block_data;

    Eigen::MatrixXd linearized_jacobians;  //边缘化得到的雅克比 该雅克比和残差用于后续先验因子MarginalizationFactor的Evaluate中计算雅克比与残差
    Eigen::VectorXd linearized_residuals;  //边缘化得到的残差
    const double eps = 1e-8;
    bool valid;

};

class MarginalizationFactor : public ceres::CostFunction
{
  public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    MarginalizationInfo* marginalization_info;
};

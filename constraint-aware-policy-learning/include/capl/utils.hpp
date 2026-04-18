// utils.hpp
#ifndef CAPL_UTILS_HPP
#define CAPL_UTILS_HPP

#include "capl/data_types.hpp"

namespace capl {
namespace Utils {

// 可视化轨迹
void visualizeTrajectory(const std::vector<Eigen::VectorXd>& trajectory);

// 保存演示数据到文件
void saveDemonstration(const std::string& filename, const Demonstration& demo);

// 从文件加载演示数据
Demonstration loadDemonstration(const std::string& filename);

// 计算轨迹误差
double computeTrajectoryError(
    const std::vector<Eigen::VectorXd>& trajectory,
    const std::vector<Eigen::VectorXd>& reference);

}  // namespace Utils
}  // namespace capl

#endif // CAPL_UTILS_HPP    
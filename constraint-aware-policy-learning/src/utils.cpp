#include "capl/utils.hpp"
#include <fstream>
#include <iostream>

namespace capl {
namespace Utils {

void visualizeTrajectory(const std::vector<Eigen::VectorXd>& trajectory) {
    std::cout << "可视化轨迹 (仅输出前10个点):" << std::endl;
    for (size_t i = 0; i < std::min(trajectory.size(), size_t(10)); ++i) {
        std::cout << "点 " << i << ": " << trajectory[i].transpose() << std::endl;
    }
    
    if (trajectory.size() > 10) {
        std::cout << "..." << std::endl;
        std::cout << "总共有 " << trajectory.size() << " 个点" << std::endl;
    }
    
    // 实际应用中可以使用如Python脚本或ROS可视化工具
}

void saveDemonstration(const std::string& filename, const Demonstration& demo) {
    std::ofstream file(filename);
    if (file.is_open()) {
        // 保存元数据
        file << "dt: " << demo.dt << std::endl;
        file << "size: " << demo.x.size() << std::endl;
        file << "dim: " << demo.x[0].size() << std::endl;
        
        // 保存数据
        for (size_t i = 0; i < demo.x.size(); ++i) {
            file << demo.t[i] << " ";
            for (int j = 0; j < demo.x[i].size(); ++j) {
                file << demo.x[i](j) << " ";
            }
            for (int j = 0; j < demo.u[i].size(); ++j) {
                file << demo.u[i](j) << " ";
            }
            file << std::endl;
        }
        file.close();
    } else {
        std::cerr << "无法打开文件: " << filename << std::endl;
    }
}

Demonstration loadDemonstration(const std::string& filename) {
    Demonstration demo;
    std::ifstream file(filename);
    
    if (file.is_open()) {
        std::string line;
        int size = 0, dim = 0;
        
        // 读取元数据
        std::getline(file, line);
        sscanf(line.c_str(), "dt: %lf", &demo.dt);
        
        std::getline(file, line);
        sscanf(line.c_str(), "size: %d", &size);
        
        std::getline(file, line);
        sscanf(line.c_str(), "dim: %d", &dim);
        
        // 调整容器大小
        demo.x.resize(size);
        demo.u.resize(size);
        demo.t.resize(size);
        
        // 读取数据
        for (int i = 0; i < size; ++i) {
            std::getline(file, line);
            std::istringstream iss(line);
            
            demo.x[i] = Eigen::VectorXd::Zero(dim);
            demo.u[i] = Eigen::VectorXd::Zero(dim);
            
            iss >> demo.t[i];
            for (int j = 0; j < dim; ++j) {
                iss >> demo.x[i](j);
            }
            for (int j = 0; j < dim; ++j) {
                iss >> demo.u[i](j);
            }
        }
        
        file.close();
    } else {
        std::cerr << "无法打开文件: " << filename << std::endl;
    }
    
    return demo;
}

double computeTrajectoryError(
    const std::vector<Eigen::VectorXd>& trajectory,
    const std::vector<Eigen::VectorXd>& reference) {
    
    double error = 0.0;
    int n = std::min(trajectory.size(), reference.size());
    
    for (int i = 0; i < n; ++i) {
        error += (trajectory[i] - reference[i]).squaredNorm();
    }
    
    return std::sqrt(error / n);
}

}  // namespace Utils
}  // namespace capl    
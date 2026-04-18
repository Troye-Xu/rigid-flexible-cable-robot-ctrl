#ifndef DY_IDENT_H
#define DY_IDENT_H
#include <Eigen/Dense>
#include <vector>

class BaseParametersIdentifier {
public:
    BaseParametersIdentifier(int base_params_num);
    
    // 添加数据样本（使用原始回归矩阵）
    void AddSample(const Eigen::VectorXd& Y_row, 
                   const Eigen::VectorXd& torque);
    
    // 执行基参数辨识
    bool IdentifyBaseParameters(Eigen::VectorXd& base_params);
    
    // 获取转换矩阵（需预先计算）
    void SetTransformationMatrix(const Eigen::MatrixXd& T);

private:
    int base_params_num_;  // 基参数数量
    Eigen::MatrixXd Phi_;  // 转换后的回归矩阵
    Eigen::VectorXd Tau_;  // 力矩观测值
    Eigen::MatrixXd T_;    // 转换矩阵
    size_t data_count_;
};

#endif // BASE_PARAMETERS_IDENT_H
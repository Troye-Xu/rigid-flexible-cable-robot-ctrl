#include "capl/constraint_estimator.hpp"
#include <Eigen/Eigenvalues>

namespace capl {

ConstraintParams ConstraintEstimator::estimate(
    const Demonstration& demo, 
    int s, //矩阵A（sxq),b(q)
    const std::vector<Eigen::MatrixXd>& Phi_A,
    const std::vector<Eigen::MatrixXd>& Phi_B) const {
    
    int T = demo.x.size();
    int q = demo.x[0].size();  // 状态维度，关节角度
    int dim_A = Phi_A[0].rows();  // Phi_A 的行数
    int dim_B = Phi_B[0].rows();  // Phi_B 的行数
    
    // 构建回归矩阵 H
    Eigen::MatrixXd H = buildRegressionMatrix(demo, Phi_A, Phi_B);
    
    // 计算 R = H*H^T/T
    Eigen::MatrixXd R = H * H.transpose() / T;//近似积分；公式18
    
    // 构建 Q 矩阵
    Eigen::MatrixXd Q = buildQMatrix(dim_A + dim_B);
    
    // 求解广义特征值问题 R*w = λ*Q*w,求（R-λ*Q）特征值和特征向量；R，Q对称
    Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges(R, Q);//通用求解器
    Eigen::VectorXd eigenvalues = ges.eigenvalues().real();//real取实部
    Eigen::MatrixXd eigenvectors = ges.eigenvectors().real();
    
    // 提取前 s 个最小特征值对应的特征向量（求解约束参数），默认升序
    ConstraintParams params;
    params.s = s;//s为约束维度，对应任务矩阵A的行数?;
    params.w_A = eigenvectors.leftCols(s).topRows(dim_A);//取前 dim_A 行
    params.w_B = eigenvectors.leftCols(s).bottomRows(dim_B);
    
    return params;
}

Eigen::MatrixXd ConstraintEstimator::buildRegressionMatrix(
    const Demonstration& demo,
    const std::vector<Eigen::MatrixXd>& Phi_A,
    const std::vector<Eigen::MatrixXd>& Phi_B) const {
    
    int T = demo.x.size();
    int dim_A = Phi_A[0].rows();
    int dim_B = Phi_B[0].rows();
    
    Eigen::MatrixXd H(dim_A + dim_B, T);
    
    for (int t = 0; t < T; ++t) {
        // 构建 H 的列：[Phi_A(t)*u(t); -Phi_B(t)]
        H.topRows(dim_A).col(t) = Phi_A[t] * demo.u[t];
        H.bottomRows(dim_B).col(t) = -Phi_B[t].col(0);  // 假设 Phi_B 是列向量
    }
    
    return H;
}


//计算正则化Q矩阵（简化计算）
Eigen::MatrixXd ConstraintEstimator::buildQMatrix(int rows) const {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(rows, rows);

    Q.topLeftCorner(rows-1, rows-1) = Eigen::MatrixXd::Identity(rows-1, rows-1);
    Q(rows-1, rows-1) = 1e-6;  // 防止奇异性
    return Q;
}

// 无简化，计算积分形式的Q矩阵
Eigen::MatrixXd computeIntegralQ(const std::vector<Eigen::MatrixXd>& Phi_A,
    const std::vector<Eigen::MatrixXd>& Phi_B) {
int T = Phi_A.size();
int dim_A = Phi_A[0].rows() * Phi_A[0].cols();
int dim_B = Phi_B[0].rows() * Phi_B[0].cols();

Eigen::MatrixXd Q_A = Eigen::MatrixXd::Zero(dim_A, dim_A);
Eigen::MatrixXd Q_B = Eigen::MatrixXd::Zero(dim_B, dim_B);

// 计算积分
for (int t = 0; t < T; ++t) {
Eigen::VectorXd phi_A_vec = vectorizeMatrix(Phi_A[t]);
Eigen::VectorXd phi_B_vec = vectorizeMatrix(Phi_B[t]);

Q_A += phi_A_vec * phi_A_vec.transpose();
Q_B += phi_B_vec * phi_B_vec.transpose();
}

Q_A /= T;  // 平均
Q_B /= T;  // 平均

// 构建块对角矩阵
Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_A + dim_B, dim_A + dim_B);
Q.topLeftCorner(dim_A, dim_A) = Q_A;
Q.bottomRightCorner(dim_B, dim_B) = 1e-6 * Q_B;

return Q;
}

//公式15，16,线性化计算
void ConstraintEstimator::computeConstraintHistory(ConstraintParams& params, const Demonstration& demo,
    const std::vector<Eigen::MatrixXd>& Phi_A,
    const std::vector<Eigen::MatrixXd>& Phi_B) {
int T = demo.u.size();
params.A_history.resize(T);
params.b_history.resize(T);

for (int t = 0; t < T; ++t) {
// 计算A(t) = w_A * Phi_A(t)
Eigen::MatrixXd A = params.w_A * vectorizeMatrix(Phi_A[t]).transpose();
A.resize(Phi_A[t].rows(), Phi_A[t].cols());
params.A_history[t] = A;

// 计算b(t) = w_B * Phi_B(t)
Eigen::VectorXd b = params.w_b * vectorizeMatrix(Phi_B[t]);
params.b_history[t] = b;
}
}

//phi_A和phi_B的计算

}  // namespace capl    
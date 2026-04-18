#include "capl/nullspace_learner.hpp"
#include <algorithm>
#include <random>

namespace capl {

NullspacePolicy NullspaceLearner::learn(
    const Demonstration& demo,
    const ConstraintParams& params,//w1,w2
    const std::vector<Eigen::MatrixXd>& Phi_A,
    const std::vector<Eigen::MatrixXd>& Phi_B,
    const Config& config) const {
    
    int T = demo.x.size();
    int q = demo.x[0].size();  // 关节角度
    
    // 1. 计算每个时间步的约束矩阵 A(t) 和零空间投影矩阵 N(t)
    std::vector<Eigen::MatrixXd> A_t(T);
    std::vector<Eigen::MatrixXd> N_t(T);
    
    for (int t = 0; t < T; ++t) {
        A_t[t] = computeConstraintMatrix(params, Phi_A[t]);
        N_t[t] = computeNullspaceProjection(A_t[t]);
    }
    
    // 2. 计算零空间分量 u_ns(t) = N(t) * u(t)
    std::vector<Eigen::VectorXd> u_ns(T);
    for (int t = 0; t < T; ++t) {
        u_ns[t] = N_t[t] * demo.u[t];
    }
    
    // 3. 使用 k-means 聚类确定局部模型中心
    std::vector<Eigen::VectorXd> centers = kMeans(demo.x, config.num_local_models);
    
    // 4. 对每个中心训练局部加权回归模型
    std::vector<Eigen::MatrixXd> w_pi(config.num_local_models);
    
    for (int m = 0; m < config.num_local_models; ++m) {
        w_pi[m] = locallyWeightedRegression(demo.x, u_ns, centers[m], config.gaussian_variance);
    }
    
    // 5. 构建零空间策略；整合上述计算参数
    NullspacePolicy policy;
    policy.centers = centers;//聚类中心
    policy.w_pi = w_pi;//参数w_pi
    policy.D = Eigen::VectorXd::Constant(config.num_local_models, config.gaussian_variance);//高斯核方差
    policy.m = config.num_local_models;//局部模型数量
    
    return policy;
}
//计算A(t) = w_A * Phi_A(t)
Eigen::MatrixXd NullspaceLearner::computeConstraintMatrix(
    const ConstraintParams& params,
    const Eigen::MatrixXd& Phi_A_t) const {
    return params.w_A * Phi_A_t;
}

Eigen::MatrixXd NullspaceLearner::computeNullspaceProjection(
    const Eigen::MatrixXd& A_t) const {
    
    // 计算伪逆 A^† = A^T*(A*A^T)^-1，注意是否需要奇异分解
    Eigen::MatrixXd A_dag = A_t.transpose() * (A_t * A_t.transpose()).inverse();
    
    // 计算零空间投影矩阵 N = I - A^† A
    int q = A_t.cols();
    return Eigen::MatrixXd::Identity(q,q) - A_dag * A_t;
}

//kMeans算法
std::vector<Eigen::VectorXd> NullspaceLearner::kMeans(
    const std::vector<Eigen::VectorXd>& points,
    int k,
    int max_iterations) const {
    
    int n = points.size();
    int dim = points[0].size();
    
    // 随机初始化中心
    std::vector<Eigen::VectorXd> centers(k);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);
    
    for (int i = 0; i < k; ++i) {
        centers[i] = points[dis(gen)];
    }
    
    // K-means 迭代
    std::vector<int> assignments(n);
    bool changed = true;
    int iteration = 0;
    
    while (changed && iteration < max_iterations) {
        changed = false;
        iteration++;
        
        // 1. 分配点到最近的中心
        for (int i = 0; i < n; ++i) {
            double min_dist = std::numeric_limits<double>::max();
            int closest_center = -1;
            
            for (int j = 0; j < k; ++j) {
                double dist = (points[i] - centers[j]).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_center = j;
                }
            }
            
            if (closest_center != assignments[i]) {
                assignments[i] = closest_center;
                changed = true;
            }
        }
        
        // 2. 更新中心
        for (int j = 0; j < k; ++j) {
            Eigen::VectorXd sum = Eigen::VectorXd::Zero(dim);
            int count = 0;
            
            for (int i = 0; i < n; ++i) {
                if (assignments[i] == j) {
                    sum += points[i];
                    count++;
                }
            }
            
            if (count > 0) {
                centers[j] = sum / count;
            }
        }
    }
    
    return centers;
}


// 基于聚类中心距离计算方差（自适应数据处理）
double computeVarianceFromClusters(const std::vector<Eigen::VectorXd>& centers) {
    int k = centers.size();//centers数量取决于任务复杂度，论文曲面25个
    double avg_dist_sq = 0.0;
    for (int i = 0; i < k; ++i) {
        double min_dist_sq = std::numeric_limits<double>::max();
        for (int j = 0; j < k; ++j) {
            if (i == j) continue;
            double dist_sq = (centers[i] - centers[j]).squaredNorm();
            if (dist_sq < min_dist_sq) min_dist_sq = dist_sq;
        }
        avg_dist_sq += min_dist_sq;
    }
    avg_dist_sq /= k;  // 平均最小距离平方
    return 0.25 * avg_dist_sq;  // 取1/4作为方差（经验系数）
}

// 高斯核函数
double NullspaceLearner::gaussianKernel(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& center,
    double variance) const {
    
    return std::exp(-0.5 * (x - center).squaredNorm() / (variance * variance));
}


//LWR,
Eigen::MatrixXd NullspaceLearner::locallyWeightedRegression(
    const std::vector<Eigen::VectorXd>& X,
    const std::vector<Eigen::VectorXd>& Y,
    const Eigen::VectorXd& center,
    double variance) const {
    
    int n = X.size();
    int input_dim = X[0].size();
    int output_dim = Y[0].size();
    
    // 构建加权最小二乘问题（求H+ g）
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(input_dim, input_dim);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(input_dim, output_dim);
    
    for (int i = 0; i < n; ++i) {
        double weight = gaussianKernel(X[i], center, variance);
        
        // 加权贡献
        A += weight * X[i] * X[i].transpose();//Φ_T*W*Φ
        B += weight * X[i] * Y[i].transpose();//Φ_T*W*Y
    }
    
    // 求解加权最小二乘问题 w = A^-1 * B
    return A.ldlt().solve(B);//wγ
}

}  // namespace capl    
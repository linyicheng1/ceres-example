#include "ceres/ceres.h"

// e = 10 - x
struct CostFunctor
{
    template <typename T>
    bool operator()(const T *const x, T *residual) const
    {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

int main(int argc, char** argv)
{
    double x = 0;
    const double init_x = x;
    // 定义问题
    ceres::Problem problem;
    // 构造误差函数
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    // 添加到求解问题中
    problem.AddResidualBlock(cost_function, nullptr, &x);
    // 求解问题
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);
    // 输出调试信息
    
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << init_x << " -> " << x << "\n";
    return 0;
}

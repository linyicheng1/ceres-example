#include "ceres/ceres.h"

// e = 10 - x
struct CostFunctor
{
    // 修改点1 --- 不需要模板函数了
    bool operator()(const double *const x, double *residual) const
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
    // 修改点2 --- NumericDiffCostFunction 并添加参数 ceres::CENTRAL
    ceres::CostFunction* cost_function =
        new ceres::NumericDiffCostFunction<CostFunctor,ceres::CENTRAL, 1, 1>(new CostFunctor);
    // 添加到求解问题中
    problem.AddResidualBlock(cost_function, nullptr, &x);
    // 求解问题
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.minimizer_progress_to_stdout = true;
    ceres::Solve(options, &problem, &summary);
    // 输出调试信息
    
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << init_x << " -> " << x << "\n";
    return 0;
}

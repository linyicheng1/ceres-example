#include "ceres/ceres.h"

// e = 10 - x
// 需要继承这样一个类
// 修改点1 --- 不再是一个结构体，而是继承类，并定义雅克比矩阵
class QuadraticCostFunction:public ceres::SizedCostFunction<1,1>
{
public:
    ~QuadraticCostFunction(){}
    virtual bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians)const
    {
        double x = parameters[0][0];
        residuals[0] = 10 - x;
        if (jacobians != NULL && jacobians[0] != NULL)
        {
            jacobians[0][0] = -1;
        }
        return true;
    }
};

int main(int argc, char** argv)
{
    double x = 8;
    const double init_x = x;
    // 定义问题
    ceres::Problem problem;
    // 构造误差函数
    // 修改点二 --- 这个直接new，就比较简单了
    ceres::CostFunction* cost_function = new QuadraticCostFunction;

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

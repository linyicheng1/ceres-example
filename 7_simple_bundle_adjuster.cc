#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

// 读取BA问题的一个数据集
class BALProblem
{
public:
    ~BALProblem()
    {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }
    // 获取观测点数量
    int num_observations() const { return num_observations_; }
    // 获取观测值
    const double *observations() const { return observations_; }
    // 获得相机参数数量
    double *mutable_cameras() { return parameters_; }
    // 获得地图点+相机参数总数量
    double *mutable_points() { return parameters_ + 9 * num_cameras_; }

    double *mutable_camera_for_observation(int i)
    {
        return mutable_cameras() + camera_index_[i] * 9;
    }
    double *mutable_point_for_observation(int i)
    {
        return mutable_points() + point_index_[i] * 3;
    }
    bool LoadFile(const char *filename)
    {
        FILE *fptr = fopen(filename, "r");
        if (fptr == NULL)
        {
            return false;
        };
        FscanfOrDie(fptr, "%d", &num_cameras_);
        FscanfOrDie(fptr, "%d", &num_points_);
        FscanfOrDie(fptr, "%d", &num_observations_);
        point_index_ = new int[num_observations_];
        camera_index_ = new int[num_observations_];
        observations_ = new double[2 * num_observations_];

        num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
        parameters_ = new double[num_parameters_];
        for (int i = 0; i < num_observations_; ++i)
        {
            FscanfOrDie(fptr, "%d", camera_index_ + i);
            FscanfOrDie(fptr, "%d", point_index_ + i);
            for (int j = 0; j < 2; ++j)
            {
                FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
            }
        }
        for (int i = 0; i < num_parameters_; ++i)
        {
            FscanfOrDie(fptr, "%lf", parameters_ + i);
        }
        return true;
    }

private:
    template <typename T>
    void FscanfOrDie(FILE *fptr, const char *format, T *value)
    {
        int num_scanned = fscanf(fptr, format, value);
        if (num_scanned != 1)
        {
            std::cerr << "Invalid UW data file.";
        }
    }
    int num_cameras_;      // 相机位姿个数
    int num_points_;       // 地图点个数
    int num_observations_; // 观测特征点个数
    int num_parameters_;   // 参数个数

    int *point_index_;     // 地图点
    int *camera_index_;    // 相机位姿
    double *observations_; // 观测特征
    double *parameters_;   // 参数
};

// 使用针孔模型参数
// 相机建模使用9个参数 3个旋转 3个平移 1个焦距长度 2个畸变参数 ，假设图像中心就是cx,cy
struct SnavelyReprojectionError
{
    // 构造函数
    SnavelyReprojectionError(double obs_x, double obs_y)
        : observed_x(obs_x), observed_y(obs_y)
    {
    }
    template <typename T>
    bool operator()(const T *const camera, const T *const point, T *residuals) const
    {
        T p[3];
        // 将point进行旋转，旋转后得到p位置
        ceres::AngleAxisRotatePoint(camera, point, p);
        // 将point进行平移
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        // 归一化坐标
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // 畸变系数
        const T &l1 = camera[7];
        const T &l2 = camera[8];
        // 当前点的半径
        T r2 = xp * xp + yp * yp;
        // 畸变系数
        T distortion = 1.0 + r2 * (l1 + l2 * r2);

        // 相机焦距
        const T &focal = camera[6];
        // 像素坐标系下位置
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        // 残差，观测值和实际值的差值
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(observed_x, observed_y)));
    }
    double observed_x;
    double observed_y;
};

int main(int argc, char **argv)
{
    std::string path = "/home/lyc/slam/code/ceres-example/problem-93-61203-pre.txt";
    BALProblem bal_problem;
    // 读取文件信息
    if (!bal_problem.LoadFile(path.c_str()))
    {
        std::cerr << "ERROR: unable to open file " << path << std::endl;
    }
    // 观测值的指针
    const double *observations = bal_problem.observations();
    // 开始构造最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); i++)
    {
        //
        ceres::CostFunction *cost_function = SnavelyReprojectionError::Create(
            observations[2 * i + 0], observations[2 * i + 1]);
        ;
        //
        problem.AddResidualBlock(
            cost_function,
            NULL,
            bal_problem.mutable_camera_for_observation(i),
            bal_problem.mutable_point_for_observation(i));
    }
    // 求解问题
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; // 使用的稠密方法求解
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    return 0;
}

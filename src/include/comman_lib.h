#ifndef COMMAN_LIB_H        //加此宏定义 防止头文件多次include造成redefinition...的错误——————防止头文件重复包含和编译
#define COMMAN_LIB_H        //这个文件！！！ main中包含了他，main还包含了imuprocess.h ，imuprocess也包含了comman.h 这就导致 main->imuprocess->comman.h
#include <deque>            //   #ifndef 与 #pragma once 没有太大区别 前者受c++支持，后者受编译器支持 前者应该是更好的                                                                                      ->comman.h    这就重复包含了这个文件 宏定义防止重复编译
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include "so3_math.h"
using namespace std;

#define INIT_COV (0.0001)

typedef pcl::PointXYZINormal PointNomalType;        //pcl::PointXYZINormal结构包含点的坐标x、y、z，及点所在曲面上的法线向量normal[0],nomal[1],nomal[2]，及曲率、及强度
typedef pcl::PointCloud<PointNomalType> PointCloudXYZI;

static const Eigen::Vector3d Lidar_offset_to_IMU(0.04165, 0.02326, -0.0284); // Avia。坐在imu坐标系看雷达坐标系（0,0,0)imu + (0.04165,0.02326,-0.0284) = (0,0,0)lidar

struct Pose6D
{
    double offset_time;
    double acc[3];    //boost::array<double, 3> 数组
    double gyr[3];
    double vel[3];
    double pos[3];
    double rot[9];
};

template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g, const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
    }
    // Eigen::Map<Eigen::Matrix3d>(rot_kp.rot, 3,3) = R;
    return std::move(rot_kp);
}

struct MeasureGroup     // Lidar data and imu dates for the curent process
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     MeasureGroup()      //构造函数，创建lidar_beg_time
     {
         lidar_beg_time = 0.0;
         this->lidar.reset(new PointCloudXYZI());
     };
    //成员属性
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZI::Ptr lidar;
    deque<sensor_msgs::Imu::ConstPtr> imu;  //imu指针容器
};

struct StateGroup
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3d Rot;                    //[0-2]
    Eigen::Vector3d Pos;                    //[3-5]
    Eigen::Vector3d Vel;                    //[6-8]
    Eigen::Vector3d gyr_bias;               //[9-11]
    Eigen::Vector3d acc_bias;               //[12-14]
    Eigen::Vector3d gravity;                //[15-17]
    Eigen::Matrix<double, 18, 18> cov;      //状态的协方差
    double last_update_time = 0;
    StateGroup();
    ~StateGroup();

    //打印函数（静态成员函数 如果声明写在里面，实现写在外面，会导致 未定义的引用）
    static void display(const StateGroup &state, std::string str = std::string("State: "))  
    {
        Eigen::Matrix<double, 3, 1> angle_axis = SO3_LOG(state.Rot) * 57.3;
        printf("%s |", str.c_str());
        printf("[%.5f] | ", state.last_update_time);
        printf("(%.3f, %.3f, %.3f) | ", angle_axis(0), angle_axis(1), angle_axis(2));
        printf("(%.3f, %.3f, %.3f) | ", state.Pos(0), state.Pos(1), state.Pos(2));
        printf("(%.3f, %.3f, %.3f) | ", state.Vel(0), state.Vel(1), state.Vel(2));
        printf("(%.3f, %.3f, %.3f) | ", state.gyr_bias(0), state.gyr_bias(1), state.gyr_bias(2));
        printf("(%.3f, %.3f, %.3f) \r\n", state.acc_bias(0), state.acc_bias(1), state.acc_bias(2));
    }

};

StateGroup::StateGroup()                    //构造函数
{
    Rot = Eigen::Matrix3d::Identity();
    Pos = Eigen::Vector3d::Zero();          //初始位移
    Vel = Eigen::Vector3d::Zero();          //初始速度
    gyr_bias = Eigen::Vector3d::Zero();
    acc_bias = Eigen::Vector3d::Zero();
    gravity = Eigen::Vector3d(0.0, 0.0, 9.805);
    cov = Eigen::Matrix<double, 18, 18>::Identity() * INIT_COV;
}
StateGroup::~StateGroup() {};               //析构函数

#endif 
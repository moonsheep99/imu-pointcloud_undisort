#include <omp.h>    //并行环境变量
#include <mutex>    //线程锁
#include <math.h>
#include <thread>   //多线程
#include <fstream>  //文件输出
#include <csignal>  //进程信号处理，如ctrl+c
#include <unistd.h> //unix的std库。许多在Linux下开发的C程序都需要头文件unistd.h，但VC中没有个头文件， 所以用VC编译总是报错。

#include <ros/ros.h>
#include <Eigen/Core> 
#include <livox_ros_driver/CustomMsg.h>  

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "imuprocess.h"
#include "preprocess.h"
//#include "comman_lib.h"


#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
using namespace std;

mutex mtx_buffer;   //定义互斥量
condition_variable sig_buffer;

bool time_sync_en = false;  //是否需要用代码时间同步 标志
bool lidar_pushed = false;

std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;      //存20ms的所有帧imu数据
std::deque<PointCloudXYZI::Ptr>        lidar_buffer;    //存20ms的点云数据（pcl格式）
std::deque<double>                     time_buffer;     //存每20ms中的点云的最后时间
double last_timestamp_imu = -0.01, last_timestamp_lidar = 0;
double lidar_end_time = 0, lidar_mean_scantime = 0;
int scan_num = 0;

MeasureGroup measurement;                   //创建对象 全局变量 -- 测量量（imu）
StateGroup g_state;                         //创建状态 全局变量 -- 系统状态（imu）//构造函数里自动赋了初置
shared_ptr<ImuProcess> p_imu(new ImuProcess());
shared_ptr<PreProcess> p_pre(new PreProcess());

PointCloudXYZI::Ptr pointcloud_undistort(new PointCloudXYZI());  //点云指针类型

//---ros pub/sub
nav_msgs::Path IMU_path;                    //IMU path 用于发布
ros::Publisher pub_pre_odometry;            //imu位姿先验的发布者
ros::Publisher pub_pre_path;                //
ros::Publisher pub_pre_pointcloud;
ros::Publisher pub_pointcloud;
double timediff_lidar_and_imu = 0.0;        //imu和雷达偏移了多少,回调中处理ros调整时间
bool timediff_set_flg = false;              //是否需时间同步---imu雷达回调配合使用

void velodyne_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    if(msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        cout << "velodyne雷达时间戳不对---clear buffer of lidar" << endl;
        lidar_buffer.clear();
    }
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


//多个话题的回调是按顺序串行执行的
void livox_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();           //-----------------------lock
    if(msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        cout << "livox雷达时间戳不对---clear buffer of lidar" << endl;
        lidar_buffer.clear();       //pcl格式
    }
    last_timestamp_lidar = msg->header.stamp.toSec();   //首个点云的时间    //更新时间戳
    if(!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())  //缓存器有数据,但时间不同步 且时间偏移大于10
    {
        cout << "imu and lidar have big gap on time, imu header time:" << last_timestamp_imu << ", lidar header time:" << last_timestamp_lidar << endl;
    }
    if(time_sync_en && !timediff_set_flg && abs(last_timestamp_imu - last_timestamp_lidar) > 1 && !imu_buffer.empty())  //需要时间同步且有偏移,这里面只会走一次
    {   
        timediff_set_flg = true;
        timediff_lidar_and_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;   //计算两传感器偏移时间
        cout << "自动对齐时间,时间偏移" << timediff_lidar_and_imu <<endl;
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());  //定义一个新的点云，用ptr指向他
    p_pre->process(msg, ptr);    //参数1：待转换的livox点云， 参数2：转换后pcl点云格式 的容器
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar); 

    mtx_buffer.unlock();         //-----------------------unlock
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));   //堆区开辟空间给msg_in，用msg指向他（简单来说就是拷贝，因为用const接收）
    //先判断是否需要纠正时间的偏移
    if(time_sync_en && abs(timediff_lidar_and_imu) > 0.1)   //需要时间同步且时间偏移大于0.1,调整时间戳
    {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_and_imu + msg_in->header.stamp.toSec());    //将浮点型格式 转化为 时间戳格式
    }

    double imu_timestamp = msg->header.stamp.toSec();   //20ms中imu第一帧的时间
    mtx_buffer.lock();          //-----------------------lock
    if(imu_timestamp < last_timestamp_imu)
    {
        cout << "imu时间戳不对---clear buffer of imu" << endl;
        imu_buffer.clear();
    }
    last_timestamp_imu = imu_timestamp; //时间戳传递
    //cout << "in cbk: msg = \n" << msg->linear_acceleration << endl;   //可以用来确定imu坐标系
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();        //-----------------------unlock
    sig_buffer.notify_all();
}

bool buffer_to_meas(MeasureGroup &meas) //把buffer数据拿到meas中
{
    if(imu_buffer.empty() || lidar_buffer.empty())
    {
        //cout << "imu_buffer = 0" << endl;
        return false;
    }

    if(lidar_pushed == false)   //目的就是确定雷达的开始时间和结束时间
    {
        meas.lidar = lidar_buffer.front();  //把缓存器中第一个20ms的点云给到meas
        meas.lidar_beg_time = time_buffer.front();  //把时间也给了                  //lidar 开始时间赋值完毕------------
        
        //设定20ms中的最后点时间（需要分情况，不是最后点是多少时间就是多少）    //目的就是求 lidar_end_time
        if(meas.lidar->size() <= 1)
        {
            cout << "meas has too little points" << endl;
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;   //结束时间 = 这个点+雷达的平均扫描时间
        }
        else if(meas.lidar->points.back().curvature / double(1000) < 0.5*lidar_mean_scantime)    //meas由buffer给的，buffer由preprocess给的，preprocess中做的格式转换
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;   //curvature是该点距离20ms首点的偏移时间
        }
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);  //最后时间和偏移时间要搞清楚
            lidar_mean_scantime = lidar_mean_scantime + (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime)/scan_num;//更新平均扫描时间
        }
        meas.lidar_end_time = lidar_end_time;                                   //lidar 结束时间赋值完毕------------
        lidar_pushed = true;
    }

    if(last_timestamp_imu < lidar_end_time) //一定是imu时间包住雷达时间，因为还要做反向传播呢   //内部imu频率200hz
    {
        return false;
    }

    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while (!imu_buffer.empty() && imu_time < lidar_end_time) //若imu_buffer不为空，则把buffer中的东西转到meas中
    {
        //cout << "imu_buffer != 0" << endl;
        //if(imu_time > lidar_end_time) break;        //imu_time是 20ms中imu的最开始时间，last_timestamp_imu是20ms中imu的最后时间
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front(); //deque容器只能对头尾进行操作
    }

    //更新缓存器
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void publish_pre_imu(const StateGroup &state)
{
    //cout << "Pos:\n" << state.Pos << endl;
    //cout << "Rot:\n" << state.Rot << endl; 
    Eigen::Quaterniond q = Eigen::Quaterniond(state.Rot);
    
    nav_msgs::Odometry imu_pre_odometry;
    imu_pre_odometry.header.frame_id = "base_link";
    imu_pre_odometry.child_frame_id = "/body";
    imu_pre_odometry.header.stamp = ros::Time().now();
    imu_pre_odometry.pose.pose.position.x = state.Pos(0);
    imu_pre_odometry.pose.pose.position.y = state.Pos(1);
    imu_pre_odometry.pose.pose.position.z = state.Pos(2);
    imu_pre_odometry.pose.pose.orientation.w = q.w();
    imu_pre_odometry.pose.pose.orientation.x = q.x();
    imu_pre_odometry.pose.pose.orientation.y = q.y();
    imu_pre_odometry.pose.pose.orientation.z = q.z();
    pub_pre_odometry.publish(imu_pre_odometry); //odometry是一个有方向的箭头（pose在header.frame_id坐标系下，twist再child_frame_id坐标系下）

    geometry_msgs::PoseStamped imu_pre_path;
    imu_pre_path.header.stamp = ros::Time().now();
    imu_pre_path.header.frame_id = "base_link";
    imu_pre_path.pose.position.x = state.Pos(0);
    imu_pre_path.pose.position.y = state.Pos(1);
    imu_pre_path.pose.position.z = state.Pos(2);
    imu_pre_path.pose.orientation.x = q.x();
    imu_pre_path.pose.orientation.y = q.y();
    imu_pre_path.pose.orientation.z = q.z();
    imu_pre_path.pose.orientation.w = q.w();
    IMU_path.header.frame_id = "base_link";
    IMU_path.poses.push_back(imu_pre_path);
    pub_pre_path.publish(IMU_path);             //path是一条连续的路径
}

//主函数
int main(int argc, char** argv)
{
    ros::init(argc,argv,"imuprocess");
    ros::NodeHandle nh;
    ros::Subscriber sub_imu = nh.subscribe( "/handsfree/imu",200000,imu_cbk);
    ros::Subscriber sub_pcl = nh.subscribe( "/velodyne_points", 200000, velodyne_cbk);

    pub_pre_odometry = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pub_pre_path = nh.advertise<nav_msgs::Path>("/Path_IMU", 100000);
    pub_pre_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/pre_pointcloud", 100000);
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/my_practice_pointcloud", 100000);

    ros::Rate rate(5000);       //5000hz = 0.02s = 20 ms
    ros::Rate rate2(1);
    bool status = ros::ok();
    while(status)
    {
        ros::spinOnce();
        if(buffer_to_meas(measurement))    //把imu信息从缓存器转到meas变量中 （注意！ 是地址传递，虽然动形参，但是等于动实参）
        {
            PointCloudXYZI pcl_pre = *(measurement.lidar);
            sensor_msgs::PointCloud2 pcl_pre_ros;
            pcl::toROSMsg(pcl_pre, pcl_pre_ros);
            pcl_pre_ros.header.frame_id = "base_link";
            pub_pre_pointcloud.publish(pcl_pre_ros);


            p_imu->Process(measurement, g_state, pointcloud_undistort);


            sensor_msgs::PointCloud2 pcl_ros;
            pcl::toROSMsg(*pointcloud_undistort, pcl_ros);
            pcl_ros.header.frame_id = "base_link";
            pub_pointcloud.publish(pcl_ros);

            publish_pre_imu(g_state);
        }
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
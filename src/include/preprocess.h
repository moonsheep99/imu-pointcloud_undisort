#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

typedef pcl::PointXYZINormal PointNomalType;        //pcl::PointXYZINormal结构包含点的坐标x、y、z，及点所在曲面上的法线向量normal[0],nomal[1],nomal[2]，及曲率、及强度
typedef pcl::PointCloud<PointNomalType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELO16, OUST64}; //{1, 2, 3} //表示支持的da雷达类型 枚举123

// velodyne数据结构
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;            //4d点坐标类型
      float intensity;            //强度
      float time;                 //时间
      uint16_t ring;              //点所属圈数
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW //使用内存对齐
  };
}  // namespace velodyne_ros

//注册velodyne_ros的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

class PreProcess
{
  public:
    PreProcess();
    ~PreProcess();
    void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);   //这里很好的解释了c和C++的不同之————多态
    void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

    PointCloudXYZI pointcloud_full_point, pointcloud_corn_point, pointcloud_surf_point; //点云（点的容器）//全部点 边缘点 平面点（不使用特征是代表有效点）
    PointCloudXYZI point_cloud_buffer[128];
    int lidar_type;
    int point_filter_num;
    int N_SCANS, SCAN_RATE;
    double blind;           //最小距离值（也就是盲区）
    bool feature_enabled, given_offset_time;//是否特征提取、是否时间偏移
    double time_unit_scale;     //用于不同雷达的时间单元

  private:
    void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
};
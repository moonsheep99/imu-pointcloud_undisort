#include "preprocess.h"
using namespace std;

PreProcess::PreProcess() : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)    //注：构造函数与析构函数无返回值
{
    N_SCANS = 6;
    SCAN_RATE = 10; 
}

PreProcess::~PreProcess() {}

void PreProcess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)    //引用比指针更安全，不存在空引用、野引用 //引用用. 指针用->
{
    avia_handler(msg);                  //最终将 msg中的有效点——转到pointcloud_surf_point中了
    *pcl_out = pointcloud_surf_point;   //令pcl_out地址中的内容改变 为有效点
}

void PreProcess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)//构造函数 ———— velondyne雷达走这里
{
    time_unit_scale = 1.e-3f;   //时间单位 ：微秒
    velodyne_handler(msg);
    *pcl_out = pointcloud_surf_point;
}

void PreProcess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    pointcloud_full_point.clear();
    pointcloud_surf_point.clear();
    pointcloud_corn_point.clear();

    int point_size = msg->point_num;
    pointcloud_full_point.resize(point_size);
    pointcloud_corn_point.resize(point_size);
    pointcloud_surf_point.resize(point_size);

    for(int i = 0; i < N_SCANS; i++)
    {
        point_cloud_buffer[i].clear();
        point_cloud_buffer[i].resize(point_size);
    }
    uint valid_num = 0;

    for (uint i = 1; i < point_size; i++)
    {                                                 //也就是avia tag的第四位和第五味表示这是第几个回波    //知乎有文章  
        //avia有最大四次回波    //只对N_SCAN中的线，且回波=0/1的点做处理 //110000  010000                      110000   000000 
        if( msg->points[i].line < N_SCANS && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00) ) //&: 按位进行 与运算    //若这个点是前两个回波的，就要了
        {
            valid_num++;    //有效点+1
            pointcloud_full_point[i].x = msg->points[i].x;
            pointcloud_full_point[i].y = msg->points[i].y;
            pointcloud_full_point[i].z = msg->points[i].z;
            pointcloud_full_point[i].intensity = msg->points[i].reflectivity;
            pointcloud_full_point[i].curvature = msg->points[i].offset_time/float(1000000); //offset_time是这一点离第一个点的时间
            if(abs(pointcloud_full_point[i].x - pointcloud_full_point[i-1].x) > 1e-7 
               || abs(pointcloud_full_point[i].y - pointcloud_full_point[i-1].y) > 1e-7 
               || abs(pointcloud_full_point[i].z - pointcloud_full_point[i-1].z) > 1e-7 )   
            {
                pointcloud_surf_point.push_back(pointcloud_full_point[i]);
            }
        }
    }
}

void PreProcess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pointcloud_full_point.clear();
    pointcloud_surf_point.clear();
    pointcloud_corn_point.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);                 //把ros的msg格式转成pcl::PointCloud<velodyne_ros::Point>格式
    int point_size = pl_orig.points.size();
    if(point_size == 0) return;
    pointcloud_surf_point.reserve(point_size);      //分配内存

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);     // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);    // last offset time

    if(pl_orig.points[point_size-1].time > 0)      //点带有时间戳（livox雷达的每个点都带时间戳）
    {
        given_offset_time = true;
    }
    else                                           //点不带有时间戳
    {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;  //寻找终止yaw
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = point_size - 1; i > 0; i--)
        {
            if(pl_orig.points[i].ring == layer_first)
            {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for(int i = 0; i < point_size; i++)
    {
        PointNomalType added_point;             //added_point 格式与 pl_orig不同，最后用added_point追加到有效点容器，因为他们格式相同   //added_point是一个点哦
        added_point.normal_x = 0;               //normal: 点所在曲面的法线向量
        added_point.normal_y = 0;
        added_point.normal_z = 0;
        added_point.x = pl_orig.points[i].x;    //把点给到added_point
        added_point.y = pl_orig.points[i].y;
        added_point.z = pl_orig.points[i].z;
        added_point.curvature = pl_orig.points[i].time*time_unit_scale; //赋值时间戳(每20ms都清0，重新计时)

        if(given_offset_time == false)
        {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_point.y, added_point.x) * 57.2957;
            if(is_first[layer])
            {
                // printf("layer: %d; is first: %d", layer, is_first[layer]);
                yaw_fp[layer] = yaw_angle;
                is_first[layer]=false;
                added_point.curvature = 0.0;
                yaw_last[layer]=yaw_angle;
                time_last[layer]=added_point.curvature;
                continue;
            }
            // compute offset time
            if (yaw_angle <= yaw_fp[layer])
            {
                added_point.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
            }
            else
            {
                added_point.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
            }
            if (added_point.curvature < time_last[layer])  added_point.curvature+=360.0/omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer]=added_point.curvature;
        }

        //如果点上自带时间戳。直接走这里
        if(added_point.x*added_point.x + added_point.y*added_point.y + added_point.z*added_point.z > (blind*blind)) //blind选取为0.01————根据雷达使用经验定义（多近的点定义为无效点）
        {
            pointcloud_surf_point.push_back(added_point);
        }
    }
}
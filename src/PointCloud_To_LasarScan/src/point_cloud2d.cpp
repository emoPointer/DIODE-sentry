#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;

sensor_msgs::LaserScan PointCloudToLaserscan(pcl::PointCloud<pcl::PointXYZ>& _pointcloud)
{
    float angle_min, angle_max, range_min, range_max, angle_increment;
    
    //需要自行调整的参数
    angle_min = -3.14159;
    angle_max =  3.14159;
    range_min = 0.5;
    range_max = 20;
    //角度分辨率，分辨率越小，转换后的误差越小
    angle_increment = 0.005;
    
    //计算扫描点个数
    unsigned int beam_size = ceil((angle_max - angle_min) / angle_increment);
    
    sensor_msgs::LaserScan output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "livox_frame";
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    output.time_increment = 0.0;
    output.scan_time = 0.0;
 
    //先将所有数据用nan填充
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    
    for (auto point : _pointcloud.points)
    {
        float range = hypot(point.x, point.y);
        float angle = atan2(point.y, point.x);
        int index = (int)((angle - output.angle_min) / output.angle_increment);
        if (index >= 0 && index < beam_size)
        {
        //如果当前内容为nan，则直接赋值
        if (isnan(output.ranges[index]))
        {
            output.ranges[index] = range;
        }
        //否则，只有距离小于当前值时，才可以重新赋值
        else
        {
            if (range < output.ranges[index])
            {
            output.ranges[index] = range;
            }
        }
        output.intensities[index] = 255;
        }
    }
    return output;
}

void pclCallback(const sensor_msgs::PointCloud2& msg)
{
    //将PointCloud2消息转换成PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *pcl_cloud);

    //对点云进行映射操作，将三维点映射到二位平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    //设置模型系数
    pcl::ModelCoefficients::Ptr cofficients(new pcl::ModelCoefficients());
    cofficients->values.resize(4);
    cofficients->values[0] = 0;
    cofficients->values[1] = 0;
    cofficients->values[2] = 1;
    cofficients->values[3] = 0;

    //创建滤波对象
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(pcl_cloud);
    proj.setModelCoefficients(cofficients);
    proj.filter(*proj_cloud);

    sensor_msgs::LaserScan laserscan = PointCloudToLaserscan(*proj_cloud);
    laserscan.header = msg.header;
    pub.publish(laserscan);
    //将pcl消息转换成sensor_msgs::PointCloud2
    // sensor_msgs::PointCloud2 pcl_2d;
    // pcl::toROSMsg(*proj_cloud, pcl_2d);
    // pcl_2d.header = msg.header;
    // pub.publish(pcl_2d);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud2d");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 2);

    ros::Subscriber sub = nh.subscribe("/points_no_ground", 2, pclCallback);

    ros::spin();

    return 0;
}
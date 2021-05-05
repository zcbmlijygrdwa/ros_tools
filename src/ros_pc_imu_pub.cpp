//#include "utility.h"

#include "ros/ros.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <string>

#include <random>

#include "/home/zhenyu/cpp_util/cpp_util.hpp"


ros::Publisher pubLidarRaw;
ros::Publisher pubIMURaw;

struct ImuRaw
{

    double gx = 0;
    double gy = 0;
    double gz = 0;
    double ax = 0;
    double ay = 0;
    double az = 0;

};

void readTimeStamp(std::vector<double>& time_stamp, std::string timestamp_filename)
{
    std::vector<std::string> lines = readLinesFromTxt(timestamp_filename);
    double t = 0.0;
    for(std::string l : lines)
    {
        std::vector<std::string> data = splitString(l, " ");
        //time_stamp.push_back(std::stod(data[1]));
        time_stamp.push_back(t); //fake
        t += 0.1;
    }
}

void readPcData(pcl::PointCloud<pcl::PointXYZI>& cloud, std::string filename)
{
    //printv(filename);

    static const size_t BufferSize = 4*sizeof(float);
    unsigned char buffer[BufferSize];
    FILE *file_ptr;

    file_ptr = fopen(filename.c_str(),"rb");  // r for read, b for binary
    const size_t fileSize = fread(buffer, sizeof(unsigned char), BufferSize, file_ptr);

    //printf("File size = %d bytes\n", (int)(fileSize));
    //printf("Size of each item in bytes = %d\n", (int)(sizeof(unsigned char)));

    int count = 0;
    while(fread(buffer,sizeof(buffer),1,file_ptr)==1)
    {
        //for(int i = 0; i<4; i++)
        //    printf("%f ", ((float*)buffer)[i]); // prints a series of bytes
        pcl::PointXYZI point;
        point.x = ((float*)buffer)[0];
        point.y = ((float*)buffer)[1];
        point.z = ((float*)buffer)[2];
        point.intensity = 0.01;
        //point.ring = count%16;
        cloud.push_back(point);
        count++    ;
    }
    //printv(count);
    fclose(file_ptr);
}

void readImuRawData(ImuRaw& imu_data, std::string filename)
{
    //printv(filename);

    std::vector<std::string> lines = readLinesFromTxt(filename);
    //printv(lines[0]);
    std::vector<std::string> nums = splitString(lines[0], " ");
    float RAD_TO_DEG = 57.29577951308233f;
    std::cout<<"read heading = "<<(stod(nums[5])*RAD_TO_DEG-78.+180.0)<<std::endl;

    imu_data.ax = stod(nums[11]);
    imu_data.ay = stod(nums[12]);
    imu_data.az = stod(nums[13]);

    imu_data.gx = stod(nums[17]);
    imu_data.gy = stod(nums[18]);
    imu_data.gz = stod(nums[19]);

}

double random_walk = 0;
std::default_random_engine generator;
std::normal_distribution<double> distribution(2.0,0.5);
void readImuData(sensor_msgs::Imu& imu_data, std::string filename)
{
    printv(filename);

    std::vector<std::string> lines = readLinesFromTxt(filename);
    //printv(lines[0]);
    std::vector<std::string> nums = splitString(lines[0], " ");
    //std::cout<<"read heading = "<<(stod(nums[5])*RAD_TO_DEG-78.+180.0)<<std::endl;

    double roll = stod(nums[3]);
    double pitch = stod(nums[4]);
    double yaw = stod(nums[5]);

    double random_n = distribution(generator);
    random_walk += random_n;
    printv(random_n);
    printv(random_walk);
    //add noise
    //yaw = random_n;

    imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);


    imu_data.linear_acceleration.x = stod(nums[11]);
    imu_data.linear_acceleration.y = stod(nums[12]);
    imu_data.linear_acceleration.z = stod(nums[13]);

    imu_data.angular_velocity.x = stod(nums[17]);
    imu_data.angular_velocity.y = stod(nums[18]);
    imu_data.angular_velocity.z = stod(nums[19]);

    for(int i = 0 ; i < 9 ; i ++)
    {
        imu_data.orientation_covariance[i] = 0;
        imu_data.angular_velocity_covariance[i] = 0;
        imu_data.linear_acceleration_covariance[i] = 0;
    }
}
void publishImuData(sensor_msgs::Imu imu_data)
{
    imu_data.header.stamp = ros::Time().now();
    //cloudMsgTemp.header.frame_id = "/velodyne";
    pubIMURaw.publish(imu_data);
    printv(imu_data.header.frame_id);
}

void publishPointCloudData(pcl::PointCloud<pcl::PointXYZI> laserCloudIn)
{
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(laserCloudIn, cloudMsgTemp);
    //printv(pc_total_count);
    //double t = time_stamp[pc_count];
    //printv(t);
    //cloudMsgTemp.header.stamp = ros::Time().fromSec(t);
    cloudMsgTemp.header.stamp = ros::Time().now();
    cloudMsgTemp.header.frame_id = "/velodyne";
    pubLidarRaw.publish(cloudMsgTemp);
}

int main(int argc, char** argv)
{

    int mode  = 1;
    //Mode = 0 : KITTI RAW data
    //Mode = 1 : KITTI odometry data


    std::string data_path = "";
    if(mode==0)
        data_path = "/home/zhenyu/datasets/kitti/2011_09_26_drive_0117_sync/2011_09_26/2011_09_26_drive_0117_sync/";
    else
        data_path = "/home/zhenyu/datasets/kitti/data_odometry_velodyne/dateset/sequences/00/";

    int pc_total_count = -1;

    //std::vector<double> time_stamp;
    //readTimeStamp(time_stamp, data_path+ "velodyne_points/timestamps.txt");
    ////printvP(time_stamp.size());
    //if(time_stamp.size()<=0)
    //{
    //    std::cout<<"Error : empty time_stamp data.."<<std::endl;
    //    return 0;
    //}


    ros::init(argc, argv, "pc_pub_node");
    ros::NodeHandle nh;

    int pc_count = 0;

    ROS_INFO("\033[1;32m---->\033[0m Lidar Raw publishing Started.");

    ros::Rate rate(2);
    std::string pc_path = "";
    if(mode==0)
        pc_path = data_path + "velodyne_points/data/";
    else
        pc_path = data_path + "velodyne/";
    std::string imu_path = data_path + "oxts/data/";
    std::string file_name = "";
    std::string postfix = ".bin";
    pubLidarRaw = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    pubIMURaw = nh.advertise<sensor_msgs::Imu>("/imu/data_noised", 2);

    std::vector<std::string> file_list = getFileListAtDir(pc_path);
    pc_total_count = file_list.size();
    printvec(file_list);
    printv(pc_total_count);

    while (ros::ok())
    {
        ros::spinOnce();
        printv(pc_count);

        int digis = -1;
        if(mode==0)
            digis = 10;
        else
            digis = 6;

        file_name = std::to_string(pc_count);
        int zeros = digis-file_name.size();
        for(int i = 0 ; i < zeros ; i++)
            file_name = "0"+file_name;

        // try to publish IMU data
        postfix = ".txt";
        float deltat = 0.1;
        if (mode==0 && pc_count<pc_total_count)
        {
            //ImuRaw imu_raw_data;
            sensor_msgs::Imu imu_data;
            readImuData(imu_data, imu_path+file_name+postfix);

            publishImuData(imu_data);
        }   

        // try to publish point cloud
        postfix = ".bin";
        if (pc_count<pc_total_count)
        {
            //printv(pc_count);
            pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
            readPcData(laserCloudIn, pc_path+file_name+postfix);
            publishPointCloudData(laserCloudIn);
        }   

        pc_count++;

        if(pc_count>=pc_total_count)
        {
            break;
        }

        printv(ros::Time().now());
        rate.sleep();
    }

    return 0;
}

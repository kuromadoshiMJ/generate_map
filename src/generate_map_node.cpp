#include<ros/ros.h>
#include<fstream>
#include<iostream>
#include<string>
#include <bits/stdc++.h>
// #include <open3d/Open3D.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>

#define dbgLine std::cerr<<"LINE:"<<__LINE__<<"\n"
#define dbg(x) std::cerr<<(#x)<<" is "<<x<<"\n"

using namespace std;
std::string mapDir = "/tmp/dump/";
struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(float, roll,
                                                       roll)(float, pitch,
                                                             pitch)(float, yaw,
                                                                    yaw)(double,
                                                                         time,
                                                                         time))

typedef PointXYZIRPYT PointTypePose;

int num_frames=10;
// pcl::PointCloud<PointTypePose>::Ptr globalPose6D;

Eigen::Matrix4f ConvertToTransformMatrix(float d[3], float q[3]) {
    Eigen::Matrix3f rotMat;

    rotMat = Eigen::AngleAxisf(q[0], Eigen::Vector3f::UnitX()) *
             Eigen::AngleAxisf(q[1], Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(q[2], Eigen::Vector3f::UnitZ());

    Eigen::Vector3f transMat;
    transMat << d[0], d[1], d[2];

    Eigen::Vector4f lastRow;
    lastRow << 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4f transformMat;

    transformMat.block<3, 3>(0, 0) = rotMat;
    transformMat.block<3, 1>(0, 3) = transMat;
    transformMat.block<1, 4>(3, 0) = lastRow;

    return transformMat;
  }
pcl::PointCloud<PointTypePose>::Ptr globalPose6D;
pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud(int index) {
    // pcl::PointCloud<PointTypePose>::Ptr globalPose6D;
    // pcl::io::loadPCDFile<PointTypePose>(mapDir + "cloudKeyPoses6D.pcd", *globalPose6D);
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << index;
    std::string cloudDir = mapDir + ss.str() + "/cloud.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>(cloudDir, *tempCloud);
    float q[3], d[3];
    
    q[0] = globalPose6D->points[index].yaw;
    q[1] = globalPose6D->points[index].roll;
    q[2] = globalPose6D->points[index].pitch;

    d[0] = globalPose6D->points[index].z;  // z
    d[1] = globalPose6D->points[index].x;  // x
    d[2] = globalPose6D->points[index].y;  // y

    // cout<<q[0]<<" "<<q[1]<<" "<<q[2]<<"\n";
    // cout<<d[1]<<" "<<d[2]<<" "<<d[0]<<"\n";

    Eigen::Matrix4f transform = ConvertToTransformMatrix(d, q);

    pcl::transformPointCloud(*tempCloud, *output, transform);
    return output;
  }

void SaveFullCloud(){
        dbgLine;
      pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud(new pcl::PointCloud<pcl::PointXYZI>);
      dbgLine;
      pcl::io::loadPCDFile<PointTypePose>(mapDir + "cloudKeyPoses6D.pcd", *globalPose6D);
      dbgLine;
      for(int i=0;i<=num_frames;i++){
        *fullCloud+=*getPointCloud(i);
      }

      pcl::io::savePCDFileASCII (mapDir +"Complete_Cloud.pcd", *fullCloud);
  }



int main(int argc, char** argv) {
  ros::init(argc, argv, "localise");

  

  SaveFullCloud();
}
//  struct myArray{
//     std::string arrayName;
//     int array[3][4];

//  };
// int main(int argc,char* argv[])
// {
//     ros::init(argc, argv, "map_generate_node");
//      myArray marr;
//     // std::string s = "Hello! World";

//     // ROS_INFO_STREAM(s);
    

//     ros::spin();
//     return 0;
// }
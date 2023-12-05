#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <cmath>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <functional>
#include <exception>
#include <iostream>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr scan;
ros::Publisher scan_pub;
ros::Publisher cluster_pub;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
int iterations = 0;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
  if (!input_cloud) {
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  if (!scan) {
    scan.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  if (!transformed_cloud) {
    transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  std::vector<double> points_x, points_y;
  for (int i = 0; i < data->ranges.size(); i++) {
    points_x.push_back(data->ranges[i] * cos(data->angle_min + i * data->angle_increment));
    points_y.push_back(data->ranges[i] * sin(data->angle_min + i * data->angle_increment));
  }

  input_cloud->is_dense = true;
  input_cloud->width = data->ranges.size();
  input_cloud->height = 1;
  input_cloud->points.resize(data->ranges.size());
  for (int i = 0; i < data->ranges.size(); i++) {
    if (!(data->ranges[i] > 0.5)) {
        input_cloud->points[i].x = points_x[i];
        input_cloud->points[i].y = points_y[i];
        input_cloud->points[i].z = 0.0;
    }
  }


  Eigen::Affine3f transform;
  geometry_msgs::TransformStamped transform_msg;
  try{
  transform_msg  = tf_buffer_->lookupTransform("odom", data->header.frame_id, data->header.stamp, ros::Duration(0.25));
  }catch(tf2::ExtrapolationException& exception){
      // transform_msg  = tf_buffer_->lookupTransform("odom", data->header.frame_id, ros::Time::now());
      return;
  }catch(tf2::LookupException& exception){
      return;
  }
  geometry_msgs::Vector3& point_msg = transform_msg.transform.translation;
  geometry_msgs::Quaternion& quat_msg = transform_msg.transform.rotation;
  Eigen::Vector3f transform_p(point_msg.x, point_msg.y, point_msg.z);
  Eigen::Quaternionf transform_q(quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z);
  transform.fromPositionOrientationScale(transform_p, transform_q, Eigen::Vector3f::Ones());
  pcl::transformPointCloud(*input_cloud, *transformed_cloud, transform);


  
  scan->is_dense = true;
  scan->width += data->ranges.size();
  scan->height = 1;
  for (int i = 0; i < data->ranges.size(); i++) {
    if (!(data->ranges[i] > 0.5)) {
        scan->push_back((*transformed_cloud)[i]);
    }
  }
  // if (iterations % 50 == 0) {
    pcl::io::savePCDFileASCII("car.pcd", *scan);
  // }
  std::cout << "Scan Size:" << scan->size() << std::endl;
  iterations += 1;

  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg<pcl::PointXYZ> (*scan, output_msg);
  output_msg.header = data->header;
  cluster_pub.publish(output_msg);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_objects");
  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/scan", 10, lidarCallback);
  scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_clusters", 10, true);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_clusters", 10, true);
  ros::spin();
}
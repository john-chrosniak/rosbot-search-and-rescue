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
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <cmath>
#include <pcl/registration/icp.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr left_scan;
pcl::PointCloud<pcl::PointXYZ>::Ptr right_scan;
pcl::PointCloud<pcl::PointXYZ>::Ptr divot_scan;
pcl::VoxelGrid<pcl::PointXYZ> sor;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
ros::Publisher cluster_pub;
ros::Publisher object_pub;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
  // std::cout << "Lidar Callback" << std::endl;
  if (!cloud) {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  cloud->clear();
  if (!tree) {
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>());
  }
  std::vector<double> points_x, points_y;
  for (int i = 0; i < data->ranges.size(); i++) {
    points_x.push_back(data->ranges[i] * cos(data->angle_min + i * data->angle_increment));
    points_y.push_back(data->ranges[i] * sin(data->angle_min + i * data->angle_increment));
  }
  cloud->is_dense = true;
  cloud->width = data->ranges.size();
  cloud->height = 1;
  // cloud->points.resize(data->ranges.size());
  for (int i = 0; i < data->ranges.size(); i++) {
    if (!(data->ranges[i] > 1.0) && !(std::isinf(data->ranges[i]))) {
        pcl::PointXYZ point;
        point.x = points_x[i];
        point.y = points_y[i];
        point.z = 0.0;
        // cloud->points[i].x = points_x[i];
        // cloud->points[i].y = points_y[i];
        // cloud->points[i].z = 0.5;
        cloud->push_back(point);
    }
  }
  
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.08);
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (200);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud); 
  ec.extract (cluster_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_clusters->height = 0;
  int cluster_count = 0;
  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {

    float cluster_mean_x = 0.0, cluster_mean_y = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : it->indices) {
        cloud_cluster->push_back ((*cloud)[idx]);
        cloud_clusters->push_back ((*cloud)[idx]);
        cluster_mean_x += (*cloud)[idx].x;
        cluster_mean_y += (*cloud)[idx].y;
        // std::cout << "Cluster Mean: (" << cluster_mean_x << ", " << cluster_mean_y << ")\n";

    }
    // std::cout << "Cluster Size: " << cloud_cluster->size() << std::endl;
    cluster_mean_x /= cloud_cluster->size();
    cluster_mean_y /= cloud_cluster->size();
    cluster_count++;
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*cloud_cluster, min, max);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    int threshold;
    for (int i =0; i < 3; i++) {
      icp.setInputSource(cloud_cluster);
      if (i == 0) {
        icp.setInputTarget(left_scan);
        threshold = 0.1;
      }
      else if (i == 1) {
        icp.setInputTarget(right_scan);
        threshold = 0.1;
      }
      else if (i == 2) {
        icp.setInputTarget(divot_scan);
        threshold = 0.5;
      }
      icp.setMaximumIterations(50);
      icp.setMaxCorrespondenceDistance(0.05);
      icp.setRANSACOutlierRejectionThreshold(0.05);
      pcl::PointCloud<pcl::PointXYZ> transformed_cloud_cluster;
      icp.align(transformed_cloud_cluster);
      // std::cout << "Cluster Mean: (" << cluster_mean_x << ", " << cluster_mean_y << ")\n";
      // std::cout << "ICP Score: " << icp.getFitnessScore() << std::endl;
      if (icp.getFitnessScore() < 0.12) {
        geometry_msgs::PointStamped detected_object;
        detected_object.header = data->header;
        detected_object.point.x = cluster_mean_x;
        detected_object.point.y = cluster_mean_y;
        detected_object.point.z = 0.0;
        object_pub.publish(detected_object);
        std::cout << "Object Detected!" << std::endl;
        break;
      }
    }
    

  }
  std::cout << "Got " << cluster_count << " clusters" << std::endl;
  
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg<pcl::PointXYZ> (*cloud_clusters, output_msg);
  output_msg.header = data->header;
  cluster_pub.publish(output_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_objects");

  left_scan.reset(new pcl::PointCloud<pcl::PointXYZ>());
  right_scan.reset(new pcl::PointCloud<pcl::PointXYZ>());
  divot_scan.reset(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile("/home/robot3/catkin_ws/src/amr_final_project/robot_scan_left.pcd", *left_scan);
  pcl::io::loadPCDFile("/home/robot3/catkin_ws/src/amr_final_project/robot_scan_right.pcd", *right_scan);
  pcl::io::loadPCDFile("/home/robot3/catkin_ws/src/amr_final_project/robot_scan_divot.pcd", *divot_scan);
  sor.setLeafSize(0.05, 0.05, 0.05);
  sor.setInputCloud(left_scan);
  sor.filter(*left_scan);
  sor.setInputCloud(right_scan);
  sor.filter(*right_scan);
  sor.setInputCloud(divot_scan);
  sor.filter(*divot_scan);
  ros::NodeHandle nh;
  object_pub = nh.advertise<geometry_msgs::PointStamped>("/detected_objects", 10, true);
  ros::Subscriber sub = nh.subscribe("/scan", 10, lidarCallback);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_clusters", 10, true);
  ros::spin();
}
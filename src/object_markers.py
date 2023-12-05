#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped 
from visualization_msgs.msg import MarkerArray, Marker
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation

running_rate = 10
marker_arr = MarkerArray()
tf_buffer = None
tf_listener = None

def add_marker(msg):
    global markers_arr, tf_buffer, tf_listener
    
    try:
        transform = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs))
    except:
        # transform = tf_buffer.lookup_transform("odom", "base_link", rospy.Time())
        return
    rotation_matrix = np.eye(4, dtype=np.float32)
    yaw_angle = Rotation.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]).as_euler('zxy')[0]
    rotation_matrix[0:3,0:3] = Rotation.from_euler('z', yaw_angle).as_dcm().astype(np.float32)
    rotation_matrix[0:3,3] = np.asarray([transform.transform.translation.x, transform.transform.translation.y, 0.0])
    rotation_matrix.transpose(0,1)
    detection_lidar_frame = np.array([-msg.point.x, -msg.point.y, 0.0, 1.0])
    detection_map_frame = np.matmul(rotation_matrix, detection_lidar_frame)[:2].T
    print("Translation:", transform.transform.translation.x, transform.transform.translation.y)

    visualization_msg = Marker()
    visualization_msg.header.frame_id = "odom"

    visualization_msg.type = Marker.SPHERE
    # visualization_msg.action = Marker.ADD
    visualization_msg.id = len(marker_arr.markers)

    

    visualization_msg.pose.position.x = detection_map_frame[0]
    visualization_msg.pose.position.y = detection_map_frame[1]

    visualization_msg.scale.x = 0.05
    visualization_msg.scale.y = 0.05
    visualization_msg.scale.z = 0.05

    visualization_msg.color.a = 1.0
    visualization_msg.color.r = 1.0

    marker_arr.markers.append(visualization_msg)


def main():
    global running_rate
    global marker_arr
    global tf_buffer
    global tf_listener
    rospy.init_node("marker_list")
    rate = rospy.Rate(running_rate)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    detected_objects_sub = rospy.Subscriber("/detected_objects", PointStamped, add_marker)
    marker_array_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=100)
    while not rospy.is_shutdown():
        # print(markers.markers)
        marker_array_pub.publish(marker_arr)
        rate.sleep()
    return

if __name__ == "__main__":
    main()
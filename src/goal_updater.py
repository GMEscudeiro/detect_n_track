#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node

import rclpy.time
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs

class GoalUpdater(Node):

    def __init__(self):
        super().__init__('goal_updater')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter('id', 0)
        
        self._inference_subscriber = self.create_subscription(
            Detection2DArray,
            '/detection_array',
            self.sub_callback,
            10
        )
    
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_update', 1)
        self.goal_marker = self.create_publisher(Marker, '/goal_marker', 1)

    def sub_callback(self, data):
        id_param = self.get_parameter('id').get_parameter_value().integer_value
        for i in data.detections:
            for j in i.results:
                if j.hypothesis.class_id == 'person' and int(i.id) == id_param:
                    try:
                        if j.pose.pose.position.x != 0:
                            goal_pose = PoseStamped()
                            goal_pose.header.stamp = self.get_clock().now().to_msg()
                            goal_pose.header.frame_id = "openni_cam_link"
                            goal_pose.pose.orientation.x = 0.0
                            goal_pose.pose.orientation.y = 0.0
                            goal_pose.pose.orientation.z = 0.0
                            goal_pose.pose.orientation.w = 1.0
                            goal_pose.pose.position.x = j.pose.pose.position.x
                            goal_pose.pose.position.y = j.pose.pose.position.y
                            goal_pose.pose.position.z = j.pose.pose.position.z
                            # It is important to wait for the listener to start listening. (rclpy.Duration)
                            output_pose_stamped = self.tf_buffer.transform(goal_pose, "base_link", rclpy.duration.Duration(seconds=0.01))
                            marker = Marker()
                            marker.header.frame_id = "base_link"
                            marker.header.stamp = self.get_clock().now().to_msg()
                            marker.ns = "person"
                            marker.id = int(i.id)
                            marker.type = 3
                            marker.action = 0
                            marker.scale.x = 0.5
                            marker.scale.y = 0.5
                            marker.scale.z = 0.5
                            marker.color.a = 1.0
                            marker.color.r = 0.0
                            marker.color.g = 1.0
                            marker.color.b = 0.0
                            marker.pose.position.x = output_pose_stamped.pose.position.x
                            marker.pose.position.y = output_pose_stamped.pose.position.y
                            marker.pose.position.z = output_pose_stamped.pose.position.z
                            marker.pose.orientation.x = output_pose_stamped.pose.orientation.x
                            marker.pose.orientation.y = output_pose_stamped.pose.orientation.y
                            marker.pose.orientation.z = output_pose_stamped.pose.orientation.z
                            marker.pose.orientation.w = output_pose_stamped.pose.orientation.w
                            
                            self.goal_marker.publish(marker)
                            self.goal_publisher.publish(output_pose_stamped)

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        raise

def main(args=None):
    rclpy.init(args=args)

    goal_updater = GoalUpdater()

    rclpy.spin(goal_updater)


if __name__ == '__main__':
    main()
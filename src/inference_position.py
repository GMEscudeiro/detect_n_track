#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped


bridge = CvBridge()

class InferencePositionCalc(Node):

    def __init__(self):
        super().__init__('inference_position')
        
        # camera intrinsics values from camera_info topic
        self.intrinsics_width = 640
        self.intrinsics_height = 480
        self.intrinsics_ppx = 319.5
        self.intrinsics_ppy = 239.5
        self.intrinsics_fx = 570.3422241210938
        self.intrinsics_fy = 570.3422241210938

        self.inferences = []

        self._subscriber_inference_result = self.create_subscription(
            Detection2DArray,
            '/detection_array',
            self.inference_result_callback,
            10)
        self._subscriber_inference_result # avoid unused variable warning

        self._subscriber_depth_camera_raw = self.create_subscription(
            Image,
            'camera/depth/image_raw',
            self.image_depth_callback,
            10)
        self._subscriber_depth_camera_raw # avoid unused variable warning

        self.detection_pos_pub = self.create_publisher(Detection2DArray, "/detection_array", 1)

    def image_depth_callback(self, data):
        try:
            cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
            new_detection_array = Detection2DArray()
            if len(self.inferences) > 0:
                for i in self.inferences:
                    inference_center = (i.bbox.center.position.x, i.bbox.center.position.y)
                    dist = cv_image[int(inference_center[1]), int(inference_center[0])] * 0.001
                    i.results[0].pose.pose.position.y = -(dist*(i.bbox.center.position.x - self.intrinsics_ppx)/self.intrinsics_fx) 
                    i.results[0].pose.pose.position.z = -(dist*(i.bbox.center.position.y - self.intrinsics_ppy)/self.intrinsics_fy)
                    i.results[0].pose.pose.position.x = float(dist)

                    new_detection_array.detections.append(i)
            self.detection_pos_pub.publish(new_detection_array)

                    

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return
        
    # gets inference center    
    def inference_result_callback(self, data): 
        self.inferences.clear()
        for i in data.detections:
            self.inferences.append(i)
        

if __name__ == '__main__':
    rclpy.init(args=None)
    inference_position = InferencePositionCalc()
    rclpy.spin(inference_position)
    rclpy.shutdown()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from track_n_follow.msg import Yolov8Inference, InferencePosition
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
            Yolov8Inference,
            '/Yolov8_Inference',
            self.inference_result_callback,
            10)
        self._subscriber_inference_result # avoid unused variable warning

        self._subscriber_depth_camera_raw = self.create_subscription(
            Image,
            'camera/depth/image_raw',
            self.image_depth_callback,
            10)
        self._subscriber_depth_camera_raw # avoid unused variable warning

        self.inference_pos_pub = self.create_publisher(InferencePosition, "/inference_position", 1)

    def image_depth_callback(self, data):
        try:
            cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
            if len(self.inferences) > 0:
                for i in self.inferences:
                    inference_pos = InferencePosition()
                    inference_center = (i.cx, i.cy)
                    dist = cv_image[int(inference_center[1]), int(inference_center[0])]
                    inference_pos.name = i.class_name
                    inference_pos.id = i.id
                    inference_pos.y = -(dist*(i.x1 - self.intrinsics_ppx)/self.intrinsics_fx) 
                    inference_pos.z = -(dist*(i.y1 - self.intrinsics_ppy)/self.intrinsics_fy)
                    inference_pos.x = float(dist)

                    self.inference_pos_pub.publish(inference_pos)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return
        
    # gets inference center    
    def inference_result_callback(self, data): 
        self.inferences.clear()
        for i in data.yolov8_inference:
            self.inferences.append(i)
        

if __name__ == '__main__':
    rclpy.init(args=None)
    inference_position = InferencePositionCalc()
    rclpy.spin(inference_position)
    rclpy.shutdown()

#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
import cv2 as cv
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from track_n_follow.msg import Yolov8Inference, InferenceResult
bridge = CvBridge()

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('yolov8n.pt')

        self.yolov8_inference = Yolov8Inference()

        self._subscriber_color_camera_raw = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.camera_callback,
            10)
        self._subscriber_color_camera_raw # avoid unused variable warning

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model.track(img, persist=True)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                id = box.id
                cx, cy = int((int(b[0]) + int(b[2]))/2), int((int(b[1]) + int(b[3]))/2) 
                inference_result.class_name = self.model.names[int(c)]
                try:
                    inference_result.id = int(id)
                except TypeError as e:
                    print(e)
                inference_result.x1 = int(b[0])
                inference_result.y1 = int(b[1])
                inference_result.x2 = int(b[2])
                inference_result.y2 = int(b[3])
                inference_result.cx = int(cx)
                inference_result.cy = int(cy)
                self.yolov8_inference.yolov8_inference.append(inference_result)
                cv.circle(img, (cx, cy), 2, (0,0,255), 10)
                

        annotated_frame = results[0].plot()

        img_msg = bridge.cv2_to_imgmsg(annotated_frame)

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

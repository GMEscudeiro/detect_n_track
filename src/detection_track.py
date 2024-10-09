#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
import cv2 as cv
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

bridge = CvBridge()

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('yolov8n.pt')

        self.detection_array = Detection2DArray()

        self._subscriber_color_camera_raw = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.camera_callback,
            10)
        self._subscriber_color_camera_raw # avoid unused variable warning

        self.detection_array_pub = self.create_publisher(Detection2DArray, "/detection_array", 1)
        self.img_pub = self.create_publisher(Image, "/detector_image", 1)

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model.track(img, persist=True)

        self.detection_array.header.frame_id = "Detections"
        self.detection_array.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                inference_result = Detection2D()
                inference_result.header.frame_id = "Detection"
                inference_result.header.stamp = self.get_clock().now().to_msg()

                b = box.xywh[0].to('cpu').detach().numpy().copy()
                class_id = box.cls
                conf = box.conf
                id = int(box.id[0].to('cpu').detach().numpy().copy())

                prob_result = ObjectHypothesisWithPose()
                prob_result.hypothesis.score = float(conf)

                try:
                    prob_result.hypothesis.class_id = self.model.names[int(class_id)]
                except TypeError as e:
                    print(e)

                inference_result.bbox.center.position.x = float(b[0])
                inference_result.bbox.center.position.y = float(b[1])
                inference_result.bbox.size_x = float(b[2])
                inference_result.bbox.size_y = float(b[3])
                inference_result.results.append(prob_result)
                inference_result.id = str(id)

                self.detection_array.detections.append(inference_result)
                cv.circle(img, (int(b[0]), int(b[1])), 2, (0,0,255), 10)
                

        annotated_frame = results[0].plot()

        img_msg = bridge.cv2_to_imgmsg(annotated_frame)

        self.img_pub.publish(img_msg)
        self.detection_array_pub.publish(self.detection_array)
        self.detection_array.detections.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

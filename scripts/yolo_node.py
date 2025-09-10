#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('./yolov8n.pt')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/cam/image_raw',
            self.camera_callback,
            10
        )

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):

        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        results = self.model(img)


        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

        annotated_frame = results[0].plot()
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "inference"
        img_msg.height = annotated_frame.shape[0]
        img_msg.width = annotated_frame.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.step = annotated_frame.shape[1] * 3
        img_msg.data = annotated_frame.tobytes()

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

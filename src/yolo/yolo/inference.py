#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix
import cv2
from cv_bridge import CvBridge
from stereo.msg import Arrow, Cone  # Use the proper naming conventions in ROS 2 (PascalCase for custom messages)
from ultralytics import YOLO
from PIL import Image as PILImage
from math import sqrt

from sensors.msg import ImuData  # Ensure this matches your ROS 2 message name and import path


class YOLOObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('inference')
        self.bridge = CvBridge()
        self.model = YOLO("/home/mrm/IRC2025/ircWS/src/yolo/arrowcone2.pt")
        self.image = None
        self.imu = None
        self.gps = None

        # Initialize publishers and subscribers
        self.image_pub = self.create_publisher(Image, '/img', 10)
        self.arrow_pub = self.create_publisher(Arrow, '/arrow_detect', 10)
        self.cone_pub = self.create_publisher(Cone, '/cone_detect', 10)

       # self.imu_sub = self.create_subscription(ImuData, "/imu_data", self.imu_callback, 10)
        #self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)
        self.image_sub = self.create_subscription(Image, '/zed/zed_node/left_raw/image_raw_color', self.img_callback, 10)

        self.timer = self.create_timer(0.005, self.run)  # Update at 200 Hz (1/200)

    def imu_callback(self, imu_msg):
        self.imu = imu_msg

    def gps_callback(self, fix):
        self.gps = fix

    def img_callback(self, image_msg):
        self.image = image_msg

    def detect_objects(self, frame):
        if frame is not None:
            height, width, _ = frame.shape
            img_center_x = width / 2
            pil_frame = PILImage.fromarray(frame)
            pred = self.model.predict(pil_frame)
            pred = pred[0]

            arrow_msg = Arrow()
            cone_msg = Cone()
            dist = []
            values = []

            for value in pred.boxes:
                box = value.xyxy
                box = (list(list(box.cpu())[0]))
                clss = float(list(value.cls.cpu())[0])

                # Define object classes
                wp = 'Left' if clss == 0 else 'Right' if clss == 1 else 'Cone'
                prob = float(list(value.conf.cpu())[0])
                box_area = float((box[2] - box[0]) * (box[3] - box[1]))
                obj_x = (7/3) + (-1/14250) * box_area

                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (200, 0, 0), 1)
                cv2.putText(frame, f'Class: {wp}  Prob: {prob:.3f}', (int(box[0])-20, int(box[1])-20),
                            cv2.FONT_HERSHEY_DUPLEX, 0.5, (200, 0, 0), 2)

                if self.gps:
                    gps_info = f"GPS Lat: {self.gps.latitude}"
                    gps_info1 = f"GPS Lon: {self.gps.longitude}"
                    cv2.putText(frame, gps_info, (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.5, (200, 0, 0), 2)
                    cv2.putText(frame, gps_info1, (10, 60), cv2.FONT_HERSHEY_DUPLEX, 0.5, (200, 0, 0), 2)

                # Check class and update messages
                if clss == 0:
                    is_found = True
                    is_right = False
                    is_cone = False
                elif clss == 1:
                    is_found = True
                    is_right = True
                    is_cone = False
                elif clss == 2:
                    is_found = True
                    is_cone = True
                    is_right = False

                box_center_x = int(box[0]) + (int(box[2] - box[0])) / 2
                obj_y = (1/558) * (box_center_x - img_center_x)
                dist.append(sqrt(obj_x**2 + obj_y**2))
                values.append([is_cone, is_found, is_right, obj_x, obj_y])

            if dist:
                ind = dist.index(min(dist))
                if values[ind][0]:
                    cone_msg.is_found = values[ind][1]
                    cone_msg.x = values[ind][3]
                    cone_msg.y = values[ind][4]
                else:
                    arrow_msg.is_found = values[ind][1]
                    arrow_msg.is_right = values[ind][2]
                    arrow_msg.x = values[ind][3]
                    arrow_msg.y = values[ind][4]

            self.arrow_pub.publish(arrow_msg)
            self.cone_pub.publish(cone_msg)
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(image_msg)

    def run(self):
        if self.image is not None:
            frame = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            self.detect_objects(frame)


def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLOObjectDetectionNode()

    try:
        rclpy.spin(yolo_node)
    except KeyboardInterrupt:
        pass

    yolo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()



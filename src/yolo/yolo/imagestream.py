#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix, CameraInfo
from stereo.msg import Arrow, Cone
from ultralytics import YOLO
from PIL import Image as PILImage
import cv2
from cv_bridge import CvBridge
import numpy as np
from flask import Flask, Response
import threading


class YOLOObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('inference')
        self.bridge = CvBridge()
        
        self.model = YOLO("/home/robomanipal/IRC2025/ircWS/src/yolo/arrowcone2old.engine", task='detect')
        self.image = None
        self.imu = None
        self.gps = None
        self.depth_image = None
        
        # Initialize ROS2 Publishers and Subscribers
        self.depth_sub = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.depth_img_callback, 10)
        self.arrow_pub = self.create_publisher(Arrow, '/arrow_detect', 10)
        self.cone_pub = self.create_publisher(Cone, '/cone_detect', 10)
        self.imu_sub = self.create_subscription(Imu, "/imu_data", self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)
        self.image_sub = self.create_subscription(Image, '/zed/zed_node/left_raw/image_raw_color', self.img_callback, 10)

        self.timer = self.create_timer(0.005, self.run)  # Update at 200 Hz (1/200)
        self.frame = None



    def depth_img_callback(self, depth_img_msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding='passthrough')

    def imu_callback(self, imu_msg):
        self.imu = imu_msg

    def gps_callback(self, fix):
        self.gps = fix

    def img_callback(self, image_msg):
        self.image = image_msg

    def calculate_distance(self, box, center_x, center_y):
        if self.depth_image is None:
            return None

        distances = self.depth_image[center_y - 5:center_y + 5, center_x - 5:center_x + 5]
        if distances.size > 0:         
            return np.mean(distances)
        return None

    def detect_objects(self, frame):
        if frame is not None:
            height, width, _ = frame.shape
            img_center_x = width / 2
            pred = self.model.predict(frame, device='cuda', verbose=False)
            pred = pred[0]

            arrow_msg = Arrow()
            cone_msg = Cone()
            for value in pred.boxes:
                box = value.xyxy
                box = (list(list(box.cpu())[0]))
                clss = float(list(value.cls.cpu())[0])
                wp = 'Left' if clss == 0 else 'Right' if clss == 1 else 'Cone'
                prob = float(list(value.conf.cpu())[0])
                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (200, 0, 0), 1)
                cv2.putText(frame, f'Class: {wp}  Prob: {prob:.3f}', (int(box[0]) - 20, int(box[1]) - 20),
                            cv2.FONT_HERSHEY_DUPLEX, 0.5, (200, 0, 0), 2)
                

                x = int(int(box[0]) + (int(box[2] - box[0])) / 2)
                y = int(int(box[1]) + (int(box[3] - box[1])) / 2)
                obj_x = self.calculate_distance(box, x, y)
                obj_y = (1 / 558) * (x - img_center_x)

                if clss == 0:
                    arrow_msg.is_found = True
                    arrow_msg.is_right = False
                elif clss == 1:
                    arrow_msg.is_found = True
                    arrow_msg.is_right = True
                elif clss == 2:
                    cone_msg.is_found = True

            self.arrow_pub.publish(arrow_msg)
            self.cone_pub.publish(cone_msg)
            
            self.frame = frame

    def run(self):
        if self.image is not None:
            frame = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            self.detect_objects(frame)


# Flask Web Server
app = Flask(__name__)
node_instance = None


def generate_frames():
    while True:
        if node_instance and node_instance.frame is not None:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 0]  # Adjust quality (0-100), lower = more compression
            _, buffer = cv2.imencode('.jpg', node_instance.frame, encode_param)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return "YOLO Detection Stream is running. Visit /video for the live stream."


@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


def flask_thread():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


def main(args=None):
    global node_instance
    rclpy.init(args=args)
    node_instance = YOLOObjectDetectionNode()

    flask_thread_obj = threading.Thread(target=flask_thread)
    flask_thread_obj.start()

    try:
        rclpy.spin(node_instance)
    except KeyboardInterrupt:
        pass

    node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

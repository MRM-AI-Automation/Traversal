#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix, CompressedImage, CameraInfo
import cv2
from cv_bridge import CvBridge
from stereo.msg import Arrow, Cone  # Use the proper naming conventions in ROS 2 (PascalCase for custom messages)
from ultralytics import YOLO
from PIL import Image as PILImage
from math import sqrt
import numpy as np

from sensors.msg import ImuData  # Ensure this matches your ROS 2 message name and import path


class YOLOObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('inference')
        self.bridge = CvBridge()
        self.model = YOLO("/home/mrm/IRC2025/ircWS/src/yolo/arrowcone2old.engine",task='detect')
        self.image = None
        self.imu = None
        self.gps = None
        self.depth_image=None
        # Initialize publishers and subscribers
        self.image_cf_pub=self.create_publisher( CameraInfo,'/img/camera_info',10)
        self.image_pub = self.create_publisher(CompressedImage, '/img/compressed', 10)
        self.depth_sub=self.create_subscription(Image,'/zed/zed_node/depth/depth_registered',self.depth_img_callback,10)
        self.arrow_pub = self.create_publisher(Arrow, '/arrow_detect', 10)
        self.cone_pub = self.create_publisher(Cone, '/cone_detect', 10)
        self.cf_sub=self.create_subscription(CameraInfo,'/zed/zed_node/left_raw/camera_info',self.camerainfo_callback,10)

        self.imu_sub = self.create_subscription(ImuData, "/imu_data", self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)
        self.image_sub = self.create_subscription(Image, '/zed/zed_node/left_raw/image_raw_color', self.img_callback, 10)

        self.timer = self.create_timer(0.001, self.run)  # Update at 200 Hz (1/200)

    def camerainfo_callback(self,cf_msg):
        cf_msg.header.frame_id = "zed_left_camera_optical_frame"
        self.image_cf_pub.publish(cf_msg)

    def depth_img_callback(self,depth_img_msg):
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

        if (distances.size>=0):         
            return np.mean(distances)
        else:
            return None

    def detect_objects(self, frame):
        if frame is not None:
            color = (0,0,255)
            height, width, _ = frame.shape
            img_center_x = width / 2
            pil_frame = PILImage.fromarray(frame)
            pred = self.model.predict(pil_frame,device='cuda',half=True,verbose=True)
            pred = pred[0]

            arrow_msg = Arrow()
            cone_msg = Cone()
            dist = 0
            values = []

            for value in pred.boxes:
                box = value.xyxy
                box = (list(list(box.cpu())[0]))
                clss = float(list(value.cls.cpu())[0])

                # Define object classes
                wp = 'Left' if clss == 0 else 'Right' if clss == 1 else 'Cone'
                prob = float(list(value.conf.cpu())[0])
                

                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (200, 0, 0), 1)
                cv2.putText(frame, f'Class: {wp}  Prob: {prob:.3f}', (int(box[0])-20, int(box[1])-20),
                            cv2.FONT_HERSHEY_DUPLEX, 0.5, (200, 0, 0), 2)
                if self.gps:
                    gps_info = f"GPS Lat: {self.gps.latitude}"
                    gps_info1 = f"GPS Lon: {self.gps.longitude}"
                    cv2.putText(frame, gps_info, (int(width/3)-50, 30), cv2.FONT_HERSHEY_DUPLEX, 0.7, (200, 0, 0), 2)
                    cv2.putText(frame, gps_info1, (int(width/3)-50, 60), cv2.FONT_HERSHEY_DUPLEX, 0.7, (200, 0, 0), 2)
                '''
                if self.imu:
                    self.angle=self.imu.orientation.x 
                    roll_imu_info = f"IMU Roll: {self.imu.orientation.z}"
                    pitch_imu_info = f"IMU Pitch: {self.imu.orientation.y}"
                    yaw_imu_info = f"IMU Yaw: {self.imu.orientation.x}"
                    cv2.putText(frame, roll_imu_info, (int(width - height/2), 30), cv2.FONT_HERSHEY_DUPLEX, 0.7, (200,0,0), 2)
                    cv2.putText(frame, pitch_imu_info, (int(width - height/2), 60), cv2.FONT_HERSHEY_DUPLEX, 0.7, (200,0,0), 2)
                    cv2.putText(frame, yaw_imu_info, (int(width - height/2), 90), cv2.FONT_HERSHEY_DUPLEX, 0.7, (200,0,0), 2)
                    print("mrm")
                cv2.putText(frame, 'N', (102, 45), cv2.FONT_HERSHEY_TRIPLEX,
                1.5, color, 1, cv2.LINE_AA)
                cv2.putText(frame, 'W', (20, 125), cv2.FONT_HERSHEY_TRIPLEX,
                1.5, color, 1, cv2.LINE_AA)
                cv2.putText(frame, 'E', (180, 125), cv2.FONT_HERSHEY_TRIPLEX,
                1.5, color, 1, cv2.LINE_AA)
                cv2.putText(frame, 'S', (102, 205), cv2.FONT_HERSHEY_TRIPLEX,
                1.5, color, 1, cv2.LINE_AA)

                
                # angle=40
                if(clss==0):
                    self.angle=self.angle+90
                elif(clss==1):
                    self.angle=self.angle-90    

                overlay_frame = cv2.imread('/home/mrm/needle2.png', cv2.IMREAD_UNCHANGED)
                overlay_frame = cv2.resize(overlay_frame, (130, 130))
                
                row, col, _ = overlay_frame.shape
                center = tuple(np.array([row, col]) / 2)
                rot_mat = cv2.getRotationMatrix2D(center, (-1.0) * self.angle, 1.0)
                overlay_frame = cv2.warpAffine(overlay_frame, rot_mat, (col, row))

                x_pos = 52
                y_pos = 43

                # needle_frame = frame.copy()

                for y in range(overlay_frame.shape[0]):
                    for x in range(overlay_frame.shape[1]):
                        if overlay_frame[y, x, 3] > 0:
                            frame[y + y_pos, x + x_pos, 0] = overlay_frame[y, x, 0]
                            frame[y + y_pos, x + x_pos, 1] = overlay_frame[y, x, 1]
                            frame[y + y_pos, x + x_pos, 2] = overlay_frame[y, x, 2]
                # Check class and update messages
                '''
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

                x = int(int(box[0]) + (int(box[2] - box[0])) / 2)
                y =int(int(box[1]) + (int(box[3]- box[1]))/2)
                box_=[int(box[0]), int(box[1]),int(box[2]), int(box[3])]
                if self.depth_image is not None:    
                    if x < self.depth_image.shape[1] and y < self.depth_image.shape[0]:
                        dist = self.depth_image[y, x]
                        self.get_logger().info(f"Distance at pixel ({x}, {y}): {dist:.2f} meters")
                    else:
                        self.get_logger().warn("Target pixel is out of bounds")
                obj_x=self.calculate_distance(box_,x,y)
                obj_y = (1/558) * (x - img_center_x)
                #print(obj_y)

                values=[is_cone, is_found, is_right, obj_x, obj_y]

            try:
                if dist:
                    if values[0]:
                        cone_msg.is_found = values[1]
                        cone_msg.x = float(values[3])
                        cone_msg.y=float(values[4])
                    else:
                        arrow_msg.is_found = values[1]
                        arrow_msg.is_right = values[2]
                        arrow_msg.x = float(values[3])
                        arrow_msg.y=float(values[4])
            except Exception as e:
                print(e)

            self.arrow_pub.publish(arrow_msg)
            self.cone_pub.publish(cone_msg)
            compressed_img_msg = CompressedImage()
            compressed_img_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_img_msg.header.frame_id = "zed_left_camera_optical_frame"
            compressed_img_msg.format = "jpeg"
            compressed_img_msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
            self.image_pub.publish(compressed_img_msg)

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






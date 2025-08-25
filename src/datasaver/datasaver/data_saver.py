#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, CompressedImage
from sensors.srv import Data
from sensors.msg import ImuData
from cv_bridge import CvBridge
import csv
import cv2


class DataSaver(Node):
    count = 1

    def __init__(self):
        super().__init__('data_receiver_node')
        self.bridge = CvBridge()
        self.image = None
        self.imu_data = None
        self.gps_data = None
        self.csv_filename = 'data.csv'
        self.saveDirectory = 'savedData'  # Directory to save images
        
        # Create the directory if it does not exist
        os.makedirs(self.saveDirectory, exist_ok=True)

        # Subscribers
        self.imu_sub = self.create_subscription(ImuData, '/imu/data_raw', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.img_sub = self.create_subscription(CompressedImage, '/img/compressed', self.img_callback, 10)

        # Service
        self.data_saver_service = self.create_service(Data, 'data_saver', self.saver_callback)

    def img_callback(self, img):
        self.image = img
    
    def imu_callback(self, imu_msg):
        self.imu_data = imu_msg    

    def gps_callback(self, fix):
        self.gps_data = fix

    def saver_callback(self, request, response):
        data = request.status

        if self.image is not None:
            # Use compressed_imgmsg_to_cv2 for compressed images
            image = self.bridge.compressed_imgmsg_to_cv2(self.image, "bgr8")
            # Construct the full path for the image file
            image_filename = os.path.join(self.saveDirectory, f"img{DataSaver.count}.png")
            cv2.imwrite(image_filename, image)
            DataSaver.count += 1
            self.get_logger().info(f"Screenshot saved as {image_filename}")

            # Write GPS and IMU data to CSV
            with open(self.csv_filename, 'a') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([
                    self.gps_data.latitude, self.gps_data.longitude,
                    self.imu_data.acceleration.x, self.imu_data.acceleration.y, self.imu_data.acceleration.z,
                    self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z
                ])
            self.get_logger().info("GPS and IMU data written to CSV")
            response.success = True
        else:
            self.get_logger().warn("No image received to capture a screenshot.")
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    data_saver = DataSaver()
    rclpy.spin(data_saver)
    data_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

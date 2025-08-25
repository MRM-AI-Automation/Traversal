import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from serial import Serial
from pyubx2 import UBXReader
import sys
import threading

class UbxParser(Node):
    def __init__(self):
        super().__init__('ubx_parser')
        self.gps_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.stream = Serial('/dev/ttyTHS0', 38400)

    def receiver_gps(self):
        ubr = UBXReader(self.stream)
        while rclpy.ok():
            (raw_data, parsed_data) = ubr.read()
            try:
                if parsed_data.identity == "NAV-PVT":
                    fixType, lat, lon, alt = parsed_data.fixType, parsed_data.lat, parsed_data.lon, parsed_data.hMSL
                    self.get_logger().info(f"FixType={fixType}, lat = {lat}, lon = {lon}, alt = {alt / 1000} m")
                    msg = NavSatFix()
                    msg.latitude = lat / 1e7  # Convert from UBX format to standard GPS format
                    msg.longitude = lon / 1e7
                    msg.altitude = alt / 1000  # Convert mm to meters
                    self.gps_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error parsing GPS data: {e}")

def main(args=None):
    rclpy.init(args=args)
    ubx = UbxParser()

    try:
        t = threading.Thread(target=ubx.receiver_gps)
        t.start()
        t.join()
    except KeyboardInterrupt:
        ubx.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    main()


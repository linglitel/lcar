import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import pynmea2

class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)

        # 串口初始化 (根据你的设备调整 ttyUSB0 和波特率)
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)

        # 定时器 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                msg = pynmea2.parse(line)

                fix = NavSatFix()
                fix.header.stamp = self.get_clock().now().to_msg()
                fix.header.frame_id = "gps_link"

                # NEO6M 给的单位就是度
                fix.latitude = msg.latitude
                fix.longitude = msg.longitude
                fix.altitude = msg.altitude

                # GPS 状态
                fix.status.status = NavSatStatus.STATUS_FIX if int(msg.gps_qual) > 0 else NavSatStatus.STATUS_NO_FIX
                fix.status.service = NavSatStatus.SERVICE_GPS

                # 协方差先填0
                fix.position_covariance = [0.0] * 9
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.publisher_.publish(fix)
                self.get_logger().info(f"Published GPS: lat={fix.latitude}, lon={fix.longitude}, alt={fix.altitude}")

        except Exception as e:
            self.get_logger().warn(f"Failed to parse line: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
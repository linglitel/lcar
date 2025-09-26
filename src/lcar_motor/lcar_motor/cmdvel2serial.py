import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToSerial(Node):
    last_cmd = None
    def __init__(self):
        super().__init__('cmdvel_to_serial')

        # 串口配置（根据实际修改）
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # 订阅 cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

        # 参数：最大速度对应PWM
        self.max_speed = 1.5   # m/s
        self.max_pwm   = 200    # 范围 0~20
        self.wheel_base = 0.3  # 两轮间距 (米)

    def listener_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # 差速计算
        v_l = v - w * self.wheel_base / 2.0
        v_r = v + w * self.wheel_base / 2.0

        # 转成PWM（映射并取绝对值）
        pwm_l = int((max(min(v_l / self.max_speed * self.max_pwm, self.max_pwm), -self.max_pwm)))
        pwm_r = int((max(min(v_r / self.max_speed * self.max_pwm, self.max_pwm), -self.max_pwm)))
        pwm = (pwm_l,pwm_r)
        
        if pwm == self.last_cmd:
            return
        self.last_cmd = pwm
        # 格式化字符串协议
        data = f"#${pwm_l},{pwm_r}\n"
        self.get_logger().info(f"Send: {data.strip()}")

        # 串口发送
        self.ser.write(data.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

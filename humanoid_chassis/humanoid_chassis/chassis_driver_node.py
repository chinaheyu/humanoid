import rclpy
import rclpy.qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import serial.tools.list_ports
import math
from crcmod import crcmod
from binascii import unhexlify
import threading
import time


def dec_to_hex(dec10):
    hex16 = "{:04X}".format(dec10 & 0xFFFF)
    return hex16


def hex_to_dec(hex16):
    dec10 =int(hex16, 16)
    if dec10 > 4096:
        dec10 = dec10 - 65536
        return dec10
    return dec10


def crc16_add(read):
    crc16 = crcmod.mkCrcFun(0x18005,rev=True,initCrc=0xFFFF,xorOut=0x0000)
    data = read.replace(" ","")
    readcrcout=hex(crc16(unhexlify(data))).upper()
    str_list = list(readcrcout)
    if len(str_list) == 5:
        str_list.insert(2,'0') 
    crc_data = "".join(str_list)
    read = read.strip() + crc_data[4:] + crc_data[2:4]

    return read


class ChassisDriverNode(Node):
    def __init__(self):
        super().__init__('chassis_driver')
        self.wheel_separation = 0.49
        self.wheel_diameter = 0.169

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.serial_device = serial.Serial(self.detect_serial_device(), 115200)
        self.initialize()

        self.publish_feedback_thread = threading.Thread(target=self.publish_feedback)
        self.publish_feedback_thread.start()

    def initialize(self):
        self.serial_device.write(bytes.fromhex('01442318331800000000DD0C'))
        time.sleep(0.1)
        len_return_data = self.serial_device.inWaiting()
        if len_return_data:
            self.serial_device.read(len_return_data)
        self.serial_device.write(bytes.fromhex('014421003100000100017534'))
        time.sleep(0.1)
        len_return_data = self.serial_device.inWaiting()
        if len_return_data:
            self.serial_device.read(len_return_data)

    def publish_feedback(self):
        if self.serial_device.is_open:
            self.serial_device.write(bytes.fromhex('0143500051006895'))
            len_return_data = self.serial_device.inWaiting()
            if len_return_data:
                return_data = self.serial_device.read(len_return_data)
                str_return_data = str(return_data.hex())
                if '014350005100' in str_return_data:
                    str_return_data = str_return_data[str_return_data.index('014350005100')+12:-1]
                    motor1 = hex_to_dec(str_return_data[0:4])
                    motor2 = hex_to_dec(str_return_data[4:8])
                    linear_x, angular_z = self.get_velocities(-motor1, motor2)
                    msg = Odometry()
                    msg.twist.twist.linear.x = linear_x
                    msg.twist.twist.angular.z = angular_z
                    self.odom_publisher.publish(msg)
                time.sleep(1.0 / 25.0)
            time.sleep(0.005)

    def detect_serial_device(self) -> str:
        for port_info in serial.tools.list_ports.comports():
            if port_info.vid == 0x1a86 and port_info.pid == 0x7523 and port_info.product == 'USB Serial':
                return port_info.device
        raise RuntimeError(f'Could not find chassis serial device!')

    def get_rpm(self, linear_x, linear_y, angular_z):
        linear_vel_x_mins_ = linear_x * 60.0
        linear_vel_y_mins_ = linear_y * 60.0

        angular_vel_z_mins_ = angular_z * 60.0

        tangential_vel_ = angular_vel_z_mins_ * self.wheel_separation / 2.0

        x_rpm_ = linear_vel_x_mins_ / (self.wheel_diameter * math.pi)
        y_rpm_ = linear_vel_y_mins_ / (self.wheel_diameter * math.pi)
        tan_rpm_ = tangential_vel_ / (self.wheel_diameter * math.pi)

        rpm_motor1 = x_rpm_ + y_rpm_ + tan_rpm_
        rpm_motor2 = x_rpm_ - y_rpm_ - tan_rpm_

        return rpm_motor1, rpm_motor2

    def get_velocities(self, motor1, motor2):
        average_rpm_x = (motor1 + motor2) / 2.0
        average_rps_x = average_rpm_x / 60.0
        linear_x = (average_rps_x * (self.wheel_diameter * math.pi))
        average_rpm_a = motor1 - motor2
        average_rps_a = average_rpm_a / 60.0
        angular_z =  (average_rps_a * (self.wheel_diameter * math.pi)) / self.wheel_separation

        return linear_x, angular_z

    def cmd_vel_callback(self, msg: Twist) -> None:
        rpm_motor1, rpm_motor2 = self.get_rpm(msg.linear.x, msg.linear.y, msg.angular.z)
        set_speed = '014423183318' + dec_to_hex(int(-rpm_motor1)) + dec_to_hex(int(rpm_motor2))
        set_speed = crc16_add(set_speed)
        if self.serial_device.is_open:
            send_data = set_speed
            send_data = bytes.fromhex(send_data)
            self.serial_device.write(send_data)

    def clean_up_and_exit(self) -> None:
        if self.serial_device.is_open:
            self.serial_device.write(bytes.fromhex('01442100310000000000E534'))
            time.sleep(0.01)
            len_return_data = self.serial_device.inWaiting()
            if len_return_data:
                self.serial_device.read(len_return_data)
            self.serial_device.close()
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    chassis_driver_node = ChassisDriverNode()
    rclpy.spin(chassis_driver_node)


if __name__ == '__main__':
    main()

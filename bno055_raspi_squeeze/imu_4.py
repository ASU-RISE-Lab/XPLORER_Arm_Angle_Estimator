import board
import busio
import adafruit_bno055
import adafruit_tca9548a
import time
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
from bno_055_arduino_ros2_serial.msg import ImuData  # Custome Message with 4 float values
from px4_msgs.msg import VehicleMagnetometer
import numpy as np
import time
from scipy.spatial.transform import Rotation
import math
from px4_msgs.msg import VehicleOdometry


class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')

        self.heading = 0.0
        self.inital_yaw = 0.0
        self.initial_yaw_set = False
        self.actual_yaw = 0.0

        self.i2c = busio.I2C(board.SCL, board.SDA)
        tca = adafruit_tca9548a.TCA9548A(self.i2c)
        self.sensor1 = adafruit_bno055.BNO055_I2C(tca[2])  # Arm Left-Front
        self.sensor2 = adafruit_bno055.BNO055_I2C(tca[3])  # Arm Right-Front
        self.sensor3 = adafruit_bno055.BNO055_I2C(tca[4])  # Arm Right-Rear
        self.sensor4 = adafruit_bno055.BNO055_I2C(tca[5])  # Arm Left-Rear

        self.publisher_ = self.create_publisher(ImuData, 'IMU_Data', 1)
        timer_period = .02 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.odom_sub = self.create_subscription(VehicleOdometry ,"/fmu/vehicle_odometry/out",self.odom_callback,10)

        self.q = np.zeros(4)

    def odom_callback(self, msg):

        self.timestamp = msg.timestamp
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

        self.q[3] = self.qw = msg.q[0]
        self.q[0] = self.qx = msg.q[1]
        self.q[1] = self.qy = msg.q[2]
        self.q[2] = self.qz = msg.q[3]

        self.rot = Rotation.from_quat(self.q)
        self.rot_euler = self.rot.as_euler('xyz', degrees=True)  # [roll, pitch, yaw]
        #self.yaw_curr = self.rot_euler[2] * math.pi / 180.0
        #print(self.rot_euler[2])
        if(self.initial_yaw_set == False):
            self.inital_yaw = self.rot_euler[2]
            self.initial_yaw_set = True

        self.actual_yaw = self.rot_euler[2] - self.inital_yaw

    def publish(self, values):

        self.get_logger().info('Imu1:"%f" Imu2:"%f" Imu3:"%f" Imu4:"%f"' % (values.imu1, values.imu2, values.imu3, values.imu4))
        self.publisher_.publish(values)

    def timer_callback(self):

        values = ImuData()

        imu1 = round(float(self.sensor1.euler[0]),4)
        imu2 = round(float(self.sensor2.euler[0]),4)
        imu3 = round(float(self.sensor3.euler[0]),4)
        imu4 = round(float(self.sensor4.euler[0]),4)

        if(abs(imu1) > 180.0):
            imu1 = imu1 - imu1/abs(imu1)*360

        if(abs(imu2) > 180.0):
            imu2 = imu2 - imu2/abs(imu2)*360

        if(abs(imu3) > 180.0):
            imu3 = imu3 - imu3/abs(imu3)*360

        if(abs(imu4) > 180.0):
            imu4 = imu4 - imu4/abs(imu4)*360

        imu1 = imu1 - 45.0
        imu2 = imu2 + 45.0
        imu3 = imu3 + 135.0
        imu4 = imu4 - 135.0

        # self.actual_yaw = 0.0

        values.imu1 = self.arm1_delta = imu1 - (self.actual_yaw)
        values.imu2 = self.arm2_delta = imu2 - (self.actual_yaw)
        values.imu3 = self.arm3_delta = imu3 - (self.actual_yaw)
        values.imu4 = self.arm4_delta = imu4 - (self.actual_yaw)
        values.timestamp = self.timestamp #= 0


        self.publish(values)


def main(args=None):
    rclpy.init(args=args)

    bno055 = BNO055_DATA()

    rclpy.spin(bno055)

    BNO055_DATA.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
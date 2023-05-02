import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
# from bno055_raspi_squeeze.msg import ImuData  # Custome Message with 4 float values
from squeeze_custom_msgs.msg import ImuData
from px4_msgs.msg import VehicleMagnetometer
import numpy as np
import time
from scipy.spatial.transform import Rotation
import math
from px4_msgs.msg import VehicleOdometry


class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')


        self.timestamp = 0
        self.heading = 0.0
        self.inital_yaw = 0.0
        self.initial_yaw_set = False
        self.actual_yaw = 0.0
        self.q = np.zeros(4)


        self.odom_sub = self.create_subscription(VehicleOdometry ,"/fmu/vehicle_odometry/out",self.odom_callback,1)


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
        #print("Odom Callback",self.timestamp)
        # self.timer_callback()
        if(self.actual_yaw == 90.0):
            print("Actual Yaw",self.actual_yaw)


def main(args=None):
    rclpy.init(args=args)

    bno055 = BNO055_DATA()

    rclpy.spin(bno055)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    BNO055_DATA.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

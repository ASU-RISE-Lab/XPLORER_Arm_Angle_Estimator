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

import board
import busio
import adafruit_bno055
import adafruit_tca9548a

class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')

        self.timestamp = 0
        self.heading = 0.0
        self.inital_yaw = 0.0
        self.initial_yaw_set = False
        self.actual_yaw = 0.0

        self.publisher_ = self.create_publisher(ImuData, 'IMU_Data', 1)
        timer_period = 80 # Hz
        self.timer = self.create_timer(1/timer_period, self.timer_callback)
        self.odom_sub = self.create_subscription(VehicleOdometry ,"/fmu/vehicle_odometry/out",self.odom_callback,1)
        
        self.sensor = None;
        self.sensor_cal = None;
        self.init_sensor();
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
        #print("Odom Callback",self.timestamp)

    def timer_callback(self):
        
        values = ImuData()
        msg = String()

        for i in range(len(self.sensor)):
            try: 
                imu = self.sensor[i].euler[0] - self.sensor_cal[i]
           
            
                if(abs(imu) > 180.0):
                    imu = imu - imu/abs(imu)*360
               
                #if (i % 2 == 0):
                #    imu = imu - 45
                #else:
                #    imu = imu + 45
                
                imu = imu - self.actual_yaw
                    
                if i == 0:
                    imu = imu + 135
                    values.imu1 = imu
                elif i == 1:
                    imu = imu + 45
                    values.imu2 = imu
                elif i == 2:
                    imu = imu - 45
                    values.imu3 = imu
                elif i == 3:
                    imu = imu - 135
                    values.imu4 = imu
                
                values.timestamp = self.timestamp

                print("imu{}: {}".format(i, round(imu,5)), end="\t", )
                if (i == (len(self.sensor) - 1)):
                    print("\n");
            except:
                pass
        self.publisher_.publish(values)
    
    def init_sensor(self):
        self.sensor = []
        self.sensor_cal = []
    
        i2c = busio.I2C(board.SCL, board.SDA)
        tca = adafruit_tca9548a.TCA9548A(i2c)
        
        for i in range(4):
            try:
                # add the plus one because of how sensors are connected
                self.sensor.append(adafruit_bno055.BNO055_I2C(tca[i+2]))
                time.sleep(0.5)
                self.sensor_cal.append(self.sensor[-1].euler[0]);
                print("sensor {} connected".format(i+2))
            except:
                print("sensor {} isnt connected".format(i+2))


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

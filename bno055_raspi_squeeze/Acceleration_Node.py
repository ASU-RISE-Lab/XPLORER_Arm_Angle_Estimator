import rclpy
from rclpy.node import Node
import numpy as np
import time
from scipy.spatial.transform import Rotation
from px4_msgs.msg import VehicleOdometry
"""
Libraries needed to read the IMU data from the BNO055 IMU.
"""
import board
import busio
import adafruit_bno055
import adafruit_tca9548a
"""
Custom Message to publish the Arm Anlges obtaind from IMUs on the arms of the drone.
Refer "https://github.com/ASU-RISE-Lab/squeeze_custom_msgs/blob/main/msg/ImuData.msg" for more info on the message type.
"""
from squeeze_custom_msgs.msg import ImuData
from squeeze_custom_msgs.msg import ImuAccData

class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')

        self.raw_publisher_ = self.create_publisher(ImuAccData, 'Arm_Acceleration_Raw', 1)
        self.fil_publisher_ = self.create_publisher(ImuAccData, 'Arm_Acceleration_Filtered', 1)

        acc = np.zeros(3)
        self.arm1 = np.zeros(3)
        self.arm2 = np.zeros(3)
        self.arm3 = np.zeros(3)
        self.arm4 = np.zeros(3)

        self.arm1_prev = np.zeros(3)
        self.arm2_prev = np.zeros(3)
        self.arm3_prev = np.zeros(3)
        self.arm4_prev = np.zeros(3)

        self.arm1_filtered = np.zeros(3)
        self.arm2_filtered = np.zeros(3)
        self.arm3_filtered = np.zeros(3)
        self.arm4_filtered = np.zeros(3)

        self.k = 0.1

        timer_period = 100 # Hz        
        self.sensor = None;
        self.init_sensor();
        self.timer = self.create_timer(1/timer_period, self.timer_callback)

    def timer_callback(self): # Sensor Callback to read Data via i2c
        
        self.timestamp = (int((time.time() * 10**6)))  # Timestamp converted to PX4 format

        raw_data = ImuAccData()
        filt_data = ImuAccData()

        for i in range(len(self.sensor)):
            try: 
                acc = np.array([self.sensor[i].acceleration[0], self.sensor[i].acceleration[1], self.sensor[i].acceleration[2]])

                if i == 0:
                    self.arm1 = acc.astype(np.float64)
                elif i == 1:
                    self.arm2 = acc.astype(np.float64)
                elif i == 2:
                    self.arm3 = acc.astype(np.float64)
                elif i == 3:
                    self.arm4 = acc.astype(np.float64)

                print("IMU{}: ACC_X:{},ACC_Y:{},ACC_Z:{}".format(i+1, round(acc[0],5), round(acc[1],5) , round(acc[2],5)))
            
                # print("imu{}: {}".format(i+1, round(imu,5)), end="\t", )
                if (i == (len(self.sensor) - 1)):
                    print("\n");
            except:
                pass
                # print("Failed to read sensor {} data".format(i+1))
        
        raw_data.timestamp = self.timestamp
        raw_data.imu1 = self.arm1
        raw_data.imu2 = self.arm2
        raw_data.imu3 = self.arm3
        raw_data.imu4 = self.arm4

        self.arm1_filtered = self.arm1 * self.k + self.arm1_prev * (1 - self.k)
        self.arm2_filtered = self.arm2 * self.k + self.arm2_prev * (1 - self.k)
        self.arm3_filtered = self.arm3 * self.k + self.arm3_prev * (1 - self.k)
        self.arm4_filtered = self.arm4 * self.k + self.arm4_prev * (1 - self.k) 

        filt_data.timestamp = self.timestamp
        filt_data.imu1 = self.arm1_filtered
        filt_data.imu2 = self.arm2_filtered
        filt_data.imu3 = self.arm3_filtered
        filt_data.imu4 = self.arm4_filtered

        self.raw_publisher_.publish(raw_data)
        self.fil_publisher_.publish(filt_data)
            
    def init_sensor(self): # Sensor Initialization
        self.sensor = []
        self.sensor_cal = []
    
        i2c = busio.I2C(board.SCL, board.SDA)
        tca = adafruit_tca9548a.TCA9548A(i2c)
        
        for i in range(4):
            try:
                # add the plus one because of how sensors are connected
                self.sensor.append(adafruit_bno055.BNO055_I2C(tca[i+2]))
                time.sleep(0.25)
                print("sensor {} connected".format(i+1))
            except:
                print("sensor {} isnt connected".format(i+1))


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

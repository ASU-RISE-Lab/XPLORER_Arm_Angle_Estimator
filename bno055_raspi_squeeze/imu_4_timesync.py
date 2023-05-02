import rclpy
from rclpy.node import Node
import numpy as np
import time
from scipy.spatial.transform import Rotation
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Float64
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

class BNO055_DATA(Node):

    def __init__(self):
        super().__init__('BNO055_DATA')

        self.MDLPF1 = Median_LPF()
        self.MDLPF2 = Median_LPF()
        self.MDLPF3 = Median_LPF()
        self.MDLPF4 = Median_LPF()

        self.timestamp = 0
        self.inital_yaw = 0.0
        self.initial_yaw_set = False
        self.actual_yaw = 0.0

        self.value_imu = [[0, 0, 0],
                          [0, 0, 0],
                          [0, 0, 0],
                          [0, 0, 0]]

        self.values_filtered = ImuData()
        self.values_filtered.imu1 = 0.0
        self.values_filtered.imu2 = 0.0
        self.values_filtered.imu3 = 0.0
        self.values_filtered.imu4 = 0.0

        self.raw_publisher_ = self.create_publisher(ImuData, 'Arm_Angles_Raw', 1)
        self.fil_publisher_ = self.create_publisher(ImuData, 'Arm_Angles_Filtered', 1)
        self.actual_yaw_publisher_ = self.create_publisher(Float64, '/IMU/Actual_Yaw',1)
        self.initial_yaw_publisher_ = self.create_publisher(Float64, '/IMU/Initial_Yaw',1)

        timer_period = 100 # Hz
        self.timer = self.create_timer(1/timer_period, self.timer_callback)
        self.odom_sub = self.create_subscription(VehicleOdometry ,"/fmu/vehicle_odometry/out",self.odom_callback,1)
        
        self.sensor = None;
        self.sensor_cal = None;
        self.init_sensor();
        self.q = np.zeros(4)

    def odom_callback(self, msg): # Odometry Callback to obtain the initial Yaw Angle to compensate the IMU

        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

        self.q[3] = self.qw = msg.q[0]
        self.q[0] = self.qx = msg.q[1]
        self.q[1] = self.qy = msg.q[2]
        self.q[2] = self.qz = msg.q[3]

        self.rot = Rotation.from_quat(self.q)
        self.rot_euler = self.rot.as_euler('xyz', degrees=True)  # [Roll, Pitch, Yaw]

        if(self.initial_yaw_set == False):
            self.inital_yaw = self.rot_euler[2]
            self.initial_yaw_set = True

        self.actual_yaw = self.rot_euler[2] - self.inital_yaw

        actual_yaw = Float64()
        inital_yaw = Float64()

        actual_yaw.data = float(self.actual_yaw)
        inital_yaw.data = float(self.inital_yaw)

        self.actual_yaw_publisher_.publish(actual_yaw)
        self.initial_yaw_publisher_.publish(inital_yaw)

    def timer_callback(self): # Sensor Callback to read Data via i2c
        
        self.timestamp = (int((time.time() * 10**6)))  # Timestamp converted to PX4 format

        values = ImuData()

        for i in range(len(self.sensor)):
            try: 
                imu = self.sensor[i].euler[0] - self.sensor_cal[i]
           
            
                if(abs(imu) > 180.0):
                    imu = imu - imu/abs(imu)*360
                               
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

                self.value_imu[i][2] = self.value_imu[i][1]
                self.value_imu[i][1] = self.value_imu[i][0]
                self.value_imu[i][0] = imu

                values.timestamp = self.timestamp
                
                # print(self.value_imu[i][0])

                print("imu{}: {}".format(i+1, round(imu,5)), end="\t", )
                if (i == (len(self.sensor) - 1)):
                    print("\n");
            except:
                pass
        
        self.values_filtered.timestamp = self.timestamp

        self.values_filtered.imu1 = self.MDLPF1.med_lp(self.value_imu[0], lp_alpha=0.8)
        self.values_filtered.imu2 = self.MDLPF2.med_lp(self.value_imu[1], lp_alpha=0.8)
        self.values_filtered.imu3 = self.MDLPF3.med_lp(self.value_imu[2], lp_alpha=0.8)
        self.values_filtered.imu4 = self.MDLPF4.med_lp(self.value_imu[3], lp_alpha=0.8)

        # Clipping the max and min values of the IMU to prevent freaking

        self.values_filtered.imu1 = np.clip(self.values_filtered.imu1, 100, 170)
        self.values_filtered.imu2 = np.clip(self.values_filtered.imu2, 10, 80)
        self.values_filtered.imu3 = np.clip(self.values_filtered.imu3, -80, -10)
        self.values_filtered.imu4 = np.clip(self.values_filtered.imu4, -170, -100)

        self.raw_publisher_.publish(values)
        self.fil_publisher_.publish(self.values_filtered)
    
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
                self.sensor_cal.append(self.sensor[-1].euler[0]);
                print("sensor {} connected".format(i+1))
            except:
                print("sensor {} isnt connected".format(i+1))

class Median_LPF:   # Median Low Pass Filter Class
    def __init__(self):
        self.lp_vase1 = [0, 0]
        self.lp_vase2 = [0, 0]

    def med_lp(self, v_window, lp_alpha):
        med_vase = []
        med_filter = []

        for j in range(len(v_window)):
            h = j + 1
            med_vase.append(v_window[len(v_window)-h])
        
        med_filter = np.median(med_vase)
        self.lp_vase1[0] = lp_alpha*med_filter + (1-lp_alpha)*self.lp_vase1[1]
        self.lp_vase2[0] = lp_alpha*self.lp_vase1[0] + (1-lp_alpha)*self.lp_vase2[1]
        lp_filter = self.lp_vase2[0]
        self.lp_vase1[1] = self.lp_vase1[0]
        self.lp_vase2[1] = self.lp_vase2[0]
        return lp_filter


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

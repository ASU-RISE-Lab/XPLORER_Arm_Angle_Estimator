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

        self.acc_raw_publisher_ = self.create_publisher(ImuAccData, 'Arm_Acceleration_Raw', 1)
        self.acc_fil_publisher_ = self.create_publisher(ImuAccData, 'Arm_Acceleration_Filtered', 1)

        self.angle_raw_publisher_ = self.create_publisher(ImuData, 'Arm_Angles_Raw', 1)
        self.angle_fil_publisher_ = self.create_publisher(ImuData, 'Arm_Angles_Filtered', 1)

        self.arm_angle_variables()
        self.accelerometer_variables()

        timer_period = 100 # Hz        
        self.sensor = None;
        self.init_sensor();
        self.timer = self.create_timer(1/timer_period, self.timer_callback)

    def arm_angle_variables(self):
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

    def accelerometer_variables(self):
        self.arm1_acc = np.zeros(3)
        self.arm2_acc = np.zeros(3)
        self.arm3_acc = np.zeros(3)
        self.arm4_acc = np.zeros(3)

        self.arm1_acc_prev = np.zeros(3)
        self.arm2_acc_prev = np.zeros(3)
        self.arm3_acc_prev = np.zeros(3)
        self.arm4_acc_prev = np.zeros(3)

        self.arm1_acc_filtered = np.zeros(3)
        self.arm2_acc_filtered = np.zeros(3)
        self.arm3_acc_filtered = np.zeros(3)
        self.arm4_acc_filtered = np.zeros(3)

        self.k = 0.1

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

    def timer_callback(self): # Sensor Callback to read Data via i2c
        
        self.timestamp = (int((time.time() * 10**6)))  # Timestamp converted to PX4 format
        values = ImuData()

        for i in range(len(self.sensor)):
            try: 
                imu = self.sensor[i].euler[0] - self.sensor_cal[i]
                acc = np.array([self.sensor[i].acceleration[0], self.sensor[i].acceleration[1], self.sensor[i].acceleration[2]])

                if(abs(imu) > 180.0):
                    imu = imu - imu/abs(imu)*360
                               
                imu = imu - self.actual_yaw

                if i == 0:
                    self.arm1_acc = acc.astype(np.float64)
                    imu = imu + 135
                    values.imu1 = imu
                elif i == 1:
                    self.arm2_acc = acc.astype(np.float64)
                    imu = imu + 45
                    values.imu2 = imu
                elif i == 2:
                    self.arm3_acc = acc.astype(np.float64)
                    imu = imu - 45
                    values.imu3 = imu
                elif i == 3:
                    self.arm4_acc = acc.astype(np.float64)
                    imu = imu - 135
                    values.imu4 = imu

                self.value_imu[i][2] = self.value_imu[i][1]
                self.value_imu[i][1] = self.value_imu[i][0]
                self.value_imu[i][0] = imu

                values.timestamp = self.timestamp

                print("IMU{}: ACC_X:{},ACC_Y:{},ACC_Z:{},Angle:{}".format(i+1, round(acc[0],5), round(acc[1],5) , round(acc[2],5), round(imu,5)))
                # print("IMU{}: {}".format(i+1, round(imu,5)), end="\t", )
                if (i == (len(self.sensor) - 1)):
                    print("\n");
            except:
                pass
                # print("Failed to read sensor {} data".format(i+1))

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

        self.angle_raw_publisher_.publish(values)
        self.angle_fil_publisher_.publish(self.values_filtered)

        self.publish_accelerometer_data()
        
    def publish_accelerometer_data(self):
        raw_data = ImuAccData()
        filt_data = ImuAccData()

        raw_data.timestamp = self.timestamp
        raw_data.imu1 = self.arm1_acc
        raw_data.imu2 = self.arm2_acc
        raw_data.imu3 = self.arm3_acc
        raw_data.imu4 = self.arm4_acc

        self.arm1_acc_filtered = (self.arm1_acc * self.k) + (self.arm1_acc_prev * (1 - self.k))
        self.arm2_acc_filtered = (self.arm2_acc * self.k) + (self.arm2_acc_prev * (1 - self.k))
        self.arm3_acc_filtered = (self.arm3_acc * self.k) + (self.arm3_acc_prev * (1 - self.k))
        self.arm4_acc_filtered = (self.arm4_acc * self.k) + (self.arm4_acc_prev * (1 - self.k))

        if (np.isnan(self.arm1_acc_filtered).any() or np.isnan(self.arm2_acc_filtered).any() or np.isnan(self.arm3_acc_filtered).any() or np.isnan(self.arm4_acc_filtered).any()):
            pass
        else:
            self.arm1_acc_prev = self.arm1_acc_filtered
            self.arm2_acc_prev = self.arm2_acc_filtered
            self.arm3_acc_prev = self.arm3_acc_filtered
            self.arm4_acc_prev = self.arm4_acc_filtered

        filt_data.timestamp = self.timestamp
        filt_data.imu1 = np.round(self.arm1_acc_filtered,4)
        filt_data.imu2 = np.round(self.arm2_acc_filtered,4)
        filt_data.imu3 = np.round(self.arm3_acc_filtered,4)
        filt_data.imu4 = np.round(self.arm4_acc_filtered,4)

        self.acc_raw_publisher_.publish(raw_data)
        self.acc_fil_publisher_.publish(filt_data)
            
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

        time.sleep(1.0)

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

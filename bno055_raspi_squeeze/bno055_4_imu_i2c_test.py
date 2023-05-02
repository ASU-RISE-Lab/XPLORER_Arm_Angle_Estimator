import board
import busio
import adafruit_bno055
import adafruit_tca9548a
import time

i2c = busio.I2C(board.SCL, board.SDA)
tca = adafruit_tca9548a.TCA9548A(i2c)
sensor1 = adafruit_bno055.BNO055_I2C(tca[2])
sensor2 = adafruit_bno055.BNO055_I2C(tca[3])
sensor3 = adafruit_bno055.BNO055_I2C(tca[4])
sensor4 = adafruit_bno055.BNO055_I2C(tca[5])


while True:
    print('Sensor 1: {}\tSensor 2: {}\tSensor 3: {}\tSensor 4: {}'.format(sensor1.euler[0], sensor2.euler[0], sensor3.euler[0],sensor4.euler[0]))
    time.sleep(0.005)

import smbus
import rospy
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import math

class Gy85:
    def __init__(self, bus_number=1):
        self.bus = smbus.SMBus(bus_number)
        
        # Accelerometer (ADXL345) registers and address
        self.ACCEL_ADDR = 0x53
        self.ACCL_PWR = 0x2D
        self.ACCL_DATA_FORMAT = 0x31
        self.ACCEL_XOUT_H = 0x32
        
        # Gyroscope (ITG3205) registers and address
        self.ITG_ADDR = 0x68
        self.ITG_SMPLRT_DIV = 0x15
        self.ITG_DLPF_FS = 0x16
        self.ITG_INT_CFG = 0x17
        self.ITG_XOUT_H = 0x1D
        
        # Magnetometer (HMC5883L) registers and address
        self.MAG_ADDR = 0x1E
        self.MAG_CONF_REG_A = 0x00
        self.MAG_CONF_REG_B = 0x01
        self.MAG_MODE_REG = 0x02
        self.MAG_XOUT_H = 0x03

    def init_accel(self):
        # Power on the accelerometer and set measurement mode
        self.bus.write_byte_data(self.ACCEL_ADDR, self.ACCL_PWR, 0x08)
        # Set data format (+/- 2g)
        self.bus.write_byte_data(self.ACCEL_ADDR, self.ACCL_DATA_FORMAT, 0x08)

    def init_gyro(self):
        # Set the gyroscope sample rate divider
        self.bus.write_byte_data(self.ITG_ADDR, self.ITG_SMPLRT_DIV, 0x07)
        # Configure gyroscope full scale and low-pass filter
        self.bus.write_byte_data(self.ITG_ADDR, self.ITG_DLPF_FS, 0x18)
        # Configure interrupt settings
        self.bus.write_byte_data(self.ITG_ADDR, self.ITG_INT_CFG, 0x00)

    def init_mag(self):
        # Initialize magnetometer (set configuration registers)
        self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CONF_REG_A, 0x70)  # 8-average, normal measurement
        self.bus.write_byte_data(self.MAG_ADDR, self.MAG_CONF_REG_B, 0xA0)  # Gain = 5
        self.bus.write_byte_data(self.MAG_ADDR, self.MAG_MODE_REG, 0x00)  # Continuous measurement mode

    def read_accel_data(self):
        # Read accelerometer data (6 bytes: XH XL, YH YL, ZH ZL)
        data = self.bus.read_i2c_block_data(self.ACCEL_ADDR, self.ACCEL_XOUT_H, 6)
        x = self._combine_bytes(data[1], data[0])
        y = self._combine_bytes(data[3], data[2])
        z = self._combine_bytes(data[5], data[4])
        return x, y, z

    def read_gyro_data(self):
        # Read gyroscope data (6 bytes: XH XL, YH YL, ZH ZL)
        data = self.bus.read_i2c_block_data(self.ITG_ADDR, self.ITG_XOUT_H, 6)
        x = self._combine_bytes(data[1], data[0])
        y = self._combine_bytes(data[3], data[2])
        z = self._combine_bytes(data[5], data[4])
        return x, y, z

    def read_mag_data(self):
        # Read magnetometer data (6 bytes: XH XL, YH YL, ZH ZL)
        data = self.bus.read_i2c_block_data(self.MAG_ADDR, self.MAG_XOUT_H, 6)
        x = self._combine_bytes(data[0], data[1])
        y = self._combine_bytes(data[4], data[5])
        z = self._combine_bytes(data[2], data[3])
        return x, y, z

    def _combine_bytes(self, msb, lsb):
        # Combine MSB and LSB into a signed 16-bit value
        value = (msb << 8) | lsb
        if value >= 0x8000:
            value -= 0x10000
        return value


def main():
    rospy.init_node('gy85_imu_node')
    pub_imu = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    pub_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    imu = Gy85()
    imu.init_accel()
    imu.init_gyro()
    imu.init_mag()

    rospy.loginfo("GY-85 IMU node initialized and publishing data.")

    while not rospy.is_shutdown():
        accel_x, accel_y, accel_z = imu.read_accel_data()
        gyro_x, gyro_y, gyro_z = imu.read_gyro_data()
        mag_x, mag_y, mag_z = imu.read_mag_data()

        # Convert raw accelerometer data to m/s²
        accel_x = accel_x / 256.0 * 9.81
        accel_y = accel_y / 256.0 * 9.81
        accel_z = accel_z / 256.0 * 9.81

        # Convert raw gyroscope data to rad/s
        gyro_x = gyro_x / 14.375 * math.pi / 180.0
        gyro_y = gyro_y / 14.375 * math.pi / 180.0
        gyro_z = gyro_z / 14.375 * math.pi / 180.0

        # Create IMU message
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.linear_acceleration_covariance = [0.0] * 9

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.angular_velocity_covariance = [0.0] * 9

        imu_msg.orientation_covariance[0] = -1

        pub_imu.publish(imu_msg)

        # Create MagneticField message
        mag_msg = MagneticField()
        mag_msg.header = Header()
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.header.frame_id = "imu_link"

        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z
        mag_msg.magnetic_field_covariance = [0.0] * 9

        pub_mag.publish(mag_msg)

        rospy.logdebug(f"Published IMU data: {imu_msg}")
        rospy.logdebug(f"Published Magnetometer data: {mag_msg}")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

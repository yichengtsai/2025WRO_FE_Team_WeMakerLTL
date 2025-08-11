# === 導入所需函式庫 ===
# 匯入GPIO控制(pigpio)、時間處理(time)、I2C通訊(smbus)、二進位資料處理(struct)、ROS雷射掃描訊息(sensor_msgs.msg.LaserScan)、ROS節點管理(rospy)、數學運算(math)模組
import pigpio
import time
import smbus
import struct
from sensor_msgs.msg import LaserScan
import rospy
import math

# === 硬體腳位設定與伺服角度校正參數 ===
Red_LED_pin = 17      # 紅色LED控制腳位
Green_LED_pin = 27    # 綠色LED控制腳位
Motor_IN1_pin = 10    # 直流馬達控制腳位IN1
Motor_IN2_pin = 22    # 直流馬達控制腳位IN2
Motor_PWM_pin = 5     # 直流馬達PWM調速腳位
Button_pin = 4        # 按鈕輸入腳位
Servo_pin = 21        # 伺服馬達控制腳位
offset = 43           # 伺服馬達角度校正值，用於調整伺服位置，確保伺服轉向中立（中間）位置

# 建立 pigpio 介面物件，用於控制 Raspberry Pi 的 GPIO
pi = pigpio.pi()

def mapping(x, in_min, in_max, out_min, out_max):
    # 將 x 從輸入範圍 [in_min, in_max] 線性映射到輸出範圍 [out_min, out_max]
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def constrain(x, out_min, out_max):
    """
    限制數值 x 在指定範圍內（out_min 到 out_max）。
    若 x 小於下限，回傳下限；若大於上限，回傳上限；否則回傳原值。
    """
    return out_min if x < out_min else out_max if x > out_max else x

# 取得I2C總線
bus = smbus.SMBus(1)

# 裝置的I2C位址
TCS34725_DEFAULT_ADDRESS = 0x29

# TCS34725 Register Set
TCS34725_COMMAND_BIT = 0x80
TCS34725_REG_ENABLE = 0x00 # Enables states and interrupts
TCS34725_REG_ATIME = 0x01 # RGBC integration time
TCS34725_REG_WTIME = 0x03 # Wait time
TCS34725_REG_CONFIG = 0x0D # Configuration register
TCS34725_REG_CONTROL = 0x0F # Control register
TCS34725_REG_CDATAL = 0x14 # Clear/IR channel low data register
TCS34725_REG_CDATAH = 0x15 # Clear/IR channel high data register
TCS34725_REG_RDATAL = 0x16 # Red ADC low data register
TCS34725_REG_RDATAH = 0x17 # Red ADC high data register
TCS34725_REG_GDATAL = 0x18 # Green ADC low data register
TCS34725_REG_GDATAH = 0x19 # Green ADC high data register
TCS34725_REG_BDATAL = 0x1A # Blue ADC low data register
TCS34725_REG_BDATAH = 0x1B # Blue ADC high data register

# TCS34725 Enable Register Configuration
TCS34725_REG_ENABLE_SAI = 0x40 # Sleep After Interrupt
TCS34725_REG_ENABLE_AIEN = 0x10 # ALS Interrupt Enable
TCS34725_REG_ENABLE_WEN = 0x08 # Wait Enable
TCS34725_REG_ENABLE_AEN = 0x02 # ADC Enable
TCS34725_REG_ENABLE_PON = 0x01 # Power ON

# TCS34725 Time Register Configuration
TCS34725_REG_ATIME_2_4 = 0xFF # Atime = 2.4 ms, Cycles = 1
TCS34725_REG_ATIME_24 = 0xF6 # Atime = 24 ms, Cycles = 10
TCS34725_REG_ATIME_101 = 0xDB # Atime = 101 ms, Cycles = 42
TCS34725_REG_ATIME_154 = 0xC0 # Atime = 154 ms, Cycles = 64
TCS34725_REG_ATIME_700 = 0x00 # Atime = 700 ms, Cycles = 256
TCS34725_REG_WTIME_2_4 = 0xFF # Wtime = 2.4 ms
TCS34725_REG_WTIME_204 = 0xAB # Wtime = 204 ms
TCS34725_REG_WTIME_614 = 0x00 # Wtime = 614 ms

# TCS34725 Gain Configuration
TCS34725_REG_CONTROL_AGAIN_1 = 0x00 # 1x Gain
TCS34725_REG_CONTROL_AGAIN_4 = 0x01 # 4x Gain
TCS34725_REG_CONTROL_AGAIN_16 = 0x02 # 16x Gain
TCS34725_REG_CONTROL_AGAIN_60 = 0x03 # 60x Gain

class TCS34725():
    def __init__(self):
        self.enable_selection()
        self.time_selection()
        self.gain_selection()

    def enable_selection(self):
        """Select the ENABLE register configuration from the given provided values"""
        ENABLE_CONFIGURATION = (TCS34725_REG_ENABLE_AEN | TCS34725_REG_ENABLE_PON)
        bus.write_byte_data(TCS34725_DEFAULT_ADDRESS, TCS34725_REG_ENABLE | TCS34725_COMMAND_BIT, ENABLE_CONFIGURATION)

    def time_selection(self):
        """Select the ATIME register configuration from the given provided values"""
        bus.write_byte_data(TCS34725_DEFAULT_ADDRESS, TCS34725_REG_ATIME | TCS34725_COMMAND_BIT, TCS34725_REG_ATIME_2_4)

        """Select the WTIME register configuration from the given provided values"""
        bus.write_byte_data(TCS34725_DEFAULT_ADDRESS, TCS34725_REG_WTIME | TCS34725_COMMAND_BIT, TCS34725_REG_WTIME_2_4)

    def gain_selection(self):
        """Select the gain register configuration from the given provided values"""
        bus.write_byte_data(TCS34725_DEFAULT_ADDRESS, TCS34725_REG_CONTROL | TCS34725_COMMAND_BIT, TCS34725_REG_CONTROL_AGAIN_1)

    def readluminance(self):
        """Read data back from TCS34725_REG_CDATAL(0x94), 8 bytes, with TCS34725_COMMAND_BIT, (0x80)
        cData LSB, cData MSB, Red LSB, Red MSB, Green LSB, Green MSB, Blue LSB, Blue MSB"""
        data = bus.read_i2c_block_data(TCS34725_DEFAULT_ADDRESS, TCS34725_REG_CDATAL | TCS34725_COMMAND_BIT, 8)

        # Convert the data
        cData = data[1] * 256 + data[0]
        red = data[3] * 256 + data[2]
        green = data[5] * 256 + data[4]
        blue = data[7] * 256 + data[6]

		# Calculate luminance
        luminance = (-0.32466 * red) + (1.57837 * green) + (-0.73191 * blue)

        return {'c' : cData, 'r' : red, 'g' : green, 'b' : blue, 'l' : luminance}

class external_control():
    def __init__(self):
        # 設定紅色LED腳位為輸出模式
        pi.set_mode(Red_LED_pin, pigpio.OUTPUT)
        # 設定綠色LED腳位為輸出模式
        pi.set_mode(Green_LED_pin, pigpio.OUTPUT)
        # 設定按鈕腳位為輸入模式
        pi.set_mode(Button_pin, pigpio.INPUT)
        # 啟用按鈕腳位的上拉電阻，避免腳位懸空雜訊
        pi.set_pull_up_down(Button_pin, pigpio.PUD_UP)
    def button_read(self):
        # 讀取按鈕狀態，按下時返回低電位(0)，未按下返回高電位(1)
        return pi.read(Button_pin)
    def Green_OFF(self):
        # 關閉綠色LED
        pi.write(Green_LED_pin, pigpio.LOW)
    def Green_ON(self):
        # 開啟綠色LED
        pi.write(Green_LED_pin, pigpio.HIGH)
    def Red_OFF(self):
        # 關閉紅色LED
        pi.write(Red_LED_pin, pigpio.LOW)
    def Red_ON(self):
        # 開啟紅色LED
        pi.write(Red_LED_pin, pigpio.HIGH)


class dc_motor():
    def __init__(self):
        # 設定直流馬達控制腳位為輸出模式
        pi.set_mode(Motor_IN1_pin, pigpio.OUTPUT)
        pi.set_mode(Motor_IN2_pin, pigpio.OUTPUT)
        pi.set_mode(Motor_PWM_pin, pigpio.OUTPUT)
        
    def DC(self, power):
        """
        控制直流馬達轉動方向與速度
        power: 馬達動力大小，範圍 -100 到 100，正值正轉，負值反轉，0為停止
        """
        if power == 0:
            # 馬達停止，IN1與IN2都設低電位，PWM輸出為0
            pi.write(Motor_IN1_pin, pigpio.LOW)
            pi.write(Motor_IN2_pin, pigpio.LOW)
            pi.set_PWM_dutycycle(Motor_PWM_pin, 0)
        elif power > 0:
            # 馬達正轉，IN1設高，IN2設低，PWM根據power比例設定
            pi.write(Motor_IN1_pin, pigpio.HIGH)
            pi.write(Motor_IN2_pin, pigpio.LOW)
            # power值映射到0~255的PWM佔空比並限制範圍
            pi.set_PWM_dutycycle(Motor_PWM_pin, constrain(mapping(power, 0, 100, 0, 255), 0, 255))
        else:
            # 馬達反轉，IN1設低，IN2設高，PWM根據power絕對值比例設定
            pi.write(Motor_IN1_pin, pigpio.LOW)
            pi.write(Motor_IN2_pin, pigpio.HIGH)
            pi.set_PWM_dutycycle(Motor_PWM_pin, constrain(mapping(abs(power), 0, 100, 0, 255), 0, 255))

class servo_motor():
    def __init__(self):
        # 設定伺服馬達控制腳位為輸出模式
        pi.set_mode(Servo_pin, pigpio.OUTPUT)
        
    def angle(self, turn_angle):
        """
        設定伺服馬達角度
        turn_angle: 伺服角度，範圍通常是 -90 到 +90 度（相對中心）
        """
        # 計算實際伺服角度：將輸入角度加上90度（中立點）和校正偏移值offset
        self.servoangle = turn_angle + 90 + offset
        
        # 將角度映射到PWM脈波寬度範圍 (500~2500微秒)
        duty = constrain(mapping(self.servoangle, 0, 180, 500, 2500), 500, 2500)
        
        # 設定伺服馬達的脈波寬度以控制角度
        pi.set_servo_pulsewidth(Servo_pin, duty)

class BNO055:
	BNO055_ADDRESS_A 				= 0x28
	BNO055_ADDRESS_B 				= 0x29
	BNO055_ID 		 			= 0xA0

	# Power mode settings
	POWER_MODE_NORMAL   				= 0X00
	POWER_MODE_LOWPOWER 				= 0X01
	POWER_MODE_SUSPEND  				= 0X02

	# Operation mode settings
	OPERATION_MODE_CONFIG 				= 0X00
	OPERATION_MODE_ACCONLY 				= 0X01
	OPERATION_MODE_MAGONLY 				= 0X02
	OPERATION_MODE_GYRONLY 				= 0X03
	OPERATION_MODE_ACCMAG 				= 0X04
	OPERATION_MODE_ACCGYRO 				= 0X05
	OPERATION_MODE_MAGGYRO 				= 0X06
	OPERATION_MODE_AMG 				= 0X07
	OPERATION_MODE_IMUPLUS 				= 0X08
	OPERATION_MODE_COMPASS 				= 0X09
	OPERATION_MODE_M4G 				= 0X0A
	OPERATION_MODE_NDOF_FMC_OFF 			= 0X0B
	OPERATION_MODE_NDOF 				= 0X0C

	# Output vector type
	VECTOR_ACCELEROMETER 				= 0x08
	VECTOR_MAGNETOMETER  				= 0x0E
	VECTOR_GYROSCOPE     				= 0x14
	VECTOR_EULER         				= 0x1A
	VECTOR_LINEARACCEL   				= 0x28
	VECTOR_GRAVITY       				= 0x2E

	# REGISTER DEFINITION START
	BNO055_PAGE_ID_ADDR 				= 0X07

	BNO055_CHIP_ID_ADDR 				= 0x00
	BNO055_ACCEL_REV_ID_ADDR 			= 0x01
	BNO055_MAG_REV_ID_ADDR 				= 0x02
	BNO055_GYRO_REV_ID_ADDR 			= 0x03
	BNO055_SW_REV_ID_LSB_ADDR 			= 0x04
	BNO055_SW_REV_ID_MSB_ADDR 			= 0x05
	BNO055_BL_REV_ID_ADDR 				= 0X06

	# Accel data register 
	BNO055_ACCEL_DATA_X_LSB_ADDR 			= 0X08
	BNO055_ACCEL_DATA_X_MSB_ADDR 			= 0X09
	BNO055_ACCEL_DATA_Y_LSB_ADDR 			= 0X0A
	BNO055_ACCEL_DATA_Y_MSB_ADDR 			= 0X0B
	BNO055_ACCEL_DATA_Z_LSB_ADDR 			= 0X0C
	BNO055_ACCEL_DATA_Z_MSB_ADDR 			= 0X0D

	# Mag data register 
	BNO055_MAG_DATA_X_LSB_ADDR 			= 0X0E
	BNO055_MAG_DATA_X_MSB_ADDR 			= 0X0F
	BNO055_MAG_DATA_Y_LSB_ADDR 			= 0X10
	BNO055_MAG_DATA_Y_MSB_ADDR 			= 0X11
	BNO055_MAG_DATA_Z_LSB_ADDR 			= 0X12
	BNO055_MAG_DATA_Z_MSB_ADDR			= 0X13

	# Gyro data registers 
	BNO055_GYRO_DATA_X_LSB_ADDR 			= 0X14
	BNO055_GYRO_DATA_X_MSB_ADDR 			= 0X15
	BNO055_GYRO_DATA_Y_LSB_ADDR 			= 0X16
	BNO055_GYRO_DATA_Y_MSB_ADDR 			= 0X17
	BNO055_GYRO_DATA_Z_LSB_ADDR 			= 0X18
	BNO055_GYRO_DATA_Z_MSB_ADDR 			= 0X19
	
	# Euler data registers 
	BNO055_EULER_H_LSB_ADDR 			= 0X1A
	BNO055_EULER_H_MSB_ADDR 			= 0X1B
	BNO055_EULER_R_LSB_ADDR 			= 0X1C
	BNO055_EULER_R_MSB_ADDR 			= 0X1D
	BNO055_EULER_P_LSB_ADDR 			= 0X1E
	BNO055_EULER_P_MSB_ADDR 			= 0X1F

	# Quaternion data registers 
	BNO055_QUATERNION_DATA_W_LSB_ADDR 		= 0X20
	BNO055_QUATERNION_DATA_W_MSB_ADDR 		= 0X21
	BNO055_QUATERNION_DATA_X_LSB_ADDR 		= 0X22
	BNO055_QUATERNION_DATA_X_MSB_ADDR 		= 0X23
	BNO055_QUATERNION_DATA_Y_LSB_ADDR 		= 0X24
	BNO055_QUATERNION_DATA_Y_MSB_ADDR 		= 0X25
	BNO055_QUATERNION_DATA_Z_LSB_ADDR 		= 0X26
	BNO055_QUATERNION_DATA_Z_MSB_ADDR 		= 0X27

	# Linear acceleration data registers 
	BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 		= 0X28
	BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 		= 0X29
	BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR	 	= 0X2A
	BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR		= 0X2B
	BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR		= 0X2C
	BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR		= 0X2D

	# Gravity data registers 
	BNO055_GRAVITY_DATA_X_LSB_ADDR 			= 0X2E
	BNO055_GRAVITY_DATA_X_MSB_ADDR	 		= 0X2F
	BNO055_GRAVITY_DATA_Y_LSB_ADDR 			= 0X30
	BNO055_GRAVITY_DATA_Y_MSB_ADDR 			= 0X31
	BNO055_GRAVITY_DATA_Z_LSB_ADDR 			= 0X32
	BNO055_GRAVITY_DATA_Z_MSB_ADDR 			= 0X33

	# Temperature data register 
	BNO055_TEMP_ADDR 				= 0X34

	# Status registers 
	BNO055_CALIB_STAT_ADDR 				= 0X35
	BNO055_SELFTEST_RESULT_ADDR	 		= 0X36
	BNO055_INTR_STAT_ADDR 				= 0X37

	BNO055_SYS_CLK_STAT_ADDR 			= 0X38
	BNO055_SYS_STAT_ADDR 				= 0X39
	BNO055_SYS_ERR_ADDR 				= 0X3A

	# Unit selection register 
	BNO055_UNIT_SEL_ADDR 				= 0X3B
	BNO055_DATA_SELECT_ADDR 			= 0X3C

	# Mode registers 
	BNO055_OPR_MODE_ADDR 				= 0X3D
	BNO055_PWR_MODE_ADDR 				= 0X3E

	BNO055_SYS_TRIGGER_ADDR 			= 0X3F
	BNO055_TEMP_SOURCE_ADDR 			= 0X40

	# Axis remap registers 
	BNO055_AXIS_MAP_CONFIG_ADDR 			= 0X41
	BNO055_AXIS_MAP_SIGN_ADDR 			= 0X42

	# SIC registers 
	BNO055_SIC_MATRIX_0_LSB_ADDR 			= 0X43
	BNO055_SIC_MATRIX_0_MSB_ADDR 			= 0X44
	BNO055_SIC_MATRIX_1_LSB_ADDR 			= 0X45
	BNO055_SIC_MATRIX_1_MSB_ADDR 			= 0X46
	BNO055_SIC_MATRIX_2_LSB_ADDR 			= 0X47
	BNO055_SIC_MATRIX_2_MSB_ADDR 			= 0X48
	BNO055_SIC_MATRIX_3_LSB_ADDR 			= 0X49
	BNO055_SIC_MATRIX_3_MSB_ADDR 			= 0X4A
	BNO055_SIC_MATRIX_4_LSB_ADDR 			= 0X4B
	BNO055_SIC_MATRIX_4_MSB_ADDR 			= 0X4C
	BNO055_SIC_MATRIX_5_LSB_ADDR 			= 0X4D
	BNO055_SIC_MATRIX_5_MSB_ADDR 			= 0X4E
	BNO055_SIC_MATRIX_6_LSB_ADDR 			= 0X4F
	BNO055_SIC_MATRIX_6_MSB_ADDR 			= 0X50
	BNO055_SIC_MATRIX_7_LSB_ADDR 			= 0X51
	BNO055_SIC_MATRIX_7_MSB_ADDR 			= 0X52
	BNO055_SIC_MATRIX_8_LSB_ADDR 			= 0X53
	BNO055_SIC_MATRIX_8_MSB_ADDR 			= 0X54
	
	# Accelerometer Offset registers	 
	ACCEL_OFFSET_X_LSB_ADDR 			= 0X55
	ACCEL_OFFSET_X_MSB_ADDR 			= 0X56
	ACCEL_OFFSET_Y_LSB_ADDR 			= 0X57
	ACCEL_OFFSET_Y_MSB_ADDR 			= 0X58
	ACCEL_OFFSET_Z_LSB_ADDR 			= 0X59
	ACCEL_OFFSET_Z_MSB_ADDR 			= 0X5A

	# Magnetometer Offset registers 
	MAG_OFFSET_X_LSB_ADDR 				= 0X5B
	MAG_OFFSET_X_MSB_ADDR 				= 0X5C
	MAG_OFFSET_Y_LSB_ADDR 				= 0X5D
	MAG_OFFSET_Y_MSB_ADDR 				= 0X5E
	MAG_OFFSET_Z_LSB_ADDR 				= 0X5F
	MAG_OFFSET_Z_MSB_ADDR 				= 0X60

	# Gyroscope Offset registers
	GYRO_OFFSET_X_LSB_ADDR 				= 0X61
	GYRO_OFFSET_X_MSB_ADDR 				= 0X62
	GYRO_OFFSET_Y_LSB_ADDR 				= 0X63
	GYRO_OFFSET_Y_MSB_ADDR 				= 0X64
	GYRO_OFFSET_Z_LSB_ADDR 				= 0X65
	GYRO_OFFSET_Z_MSB_ADDR 				= 0X66

	# Radius registers 
	ACCEL_RADIUS_LSB_ADDR 				= 0X67
	ACCEL_RADIUS_MSB_ADDR 				= 0X68
	MAG_RADIUS_LSB_ADDR 				= 0X69
	MAG_RADIUS_MSB_ADDR 				= 0X6A

	# REGISTER DEFINITION END

	def __init__(self, sensorId=-1, address=0x28):
		self._sensorId = sensorId
		self._address = address
		self._bus = smbus.SMBus(1)
		self._mode = BNO055.OPERATION_MODE_NDOF

	def begin(self, mode=None):
		if mode is None: mode = BNO055.OPERATION_MODE_NDOF
		# Open I2C bus
		self._bus = smbus.SMBus(1)

		# Make sure we have the right device
		if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
			time.sleep(1)	# Wait for the device to boot up
			if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
				return False

		# Switch to config mode
		self.setMode(BNO055.OPERATION_MODE_CONFIG)

		# Trigger a reset and wait for the device to boot up again
		self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x20])
		time.sleep(1)
		while self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
			time.sleep(0.01)
		time.sleep(0.05)

		# Set to normal power mode
		self.writeBytes(BNO055.BNO055_PWR_MODE_ADDR, [BNO055.POWER_MODE_NORMAL])
		time.sleep(0.01)

		self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
		self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0])
		time.sleep(0.01)

		# Set the requested mode
		self.setMode(mode)
		time.sleep(0.02)

		return True

	def setMode(self, mode):
		self._mode = mode
		self.writeBytes(BNO055.BNO055_OPR_MODE_ADDR, [self._mode])
		time.sleep(0.03)

	def setExternalCrystalUse(self, useExternalCrystal = True):
		prevMode = self._mode
		self.setMode(BNO055.OPERATION_MODE_CONFIG)
		time.sleep(0.025)
		self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
		self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x80] if useExternalCrystal else [0])
		time.sleep(0.01)
		self.setMode(prevMode)
		time.sleep(0.02)

	def getSystemStatus(self):
		self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
		(sys_stat, sys_err) = self.readBytes(BNO055.BNO055_SYS_STAT_ADDR, 2)
		self_test = self.readBytes(BNO055.BNO055_SELFTEST_RESULT_ADDR)[0]
		return (sys_stat, self_test, sys_err)

	def getRevInfo(self):
		(accel_rev, mag_rev, gyro_rev) = self.readBytes(BNO055.BNO055_ACCEL_REV_ID_ADDR, 3)
		sw_rev = self.readBytes(BNO055.BNO055_SW_REV_ID_LSB_ADDR, 2)
		sw_rev = sw_rev[0] | sw_rev[1] << 8
		bl_rev = self.readBytes(BNO055.BNO055_BL_REV_ID_ADDR)[0]
		return (accel_rev, mag_rev, gyro_rev, sw_rev, bl_rev)

	def getCalibration(self):
		calData = self.readBytes(BNO055.BNO055_CALIB_STAT_ADDR)[0]
		return (calData >> 6 & 0x03, calData >> 4 & 0x03, calData >> 2 & 0x03, calData & 0x03)

	def getTemp(self):
		return self.readBytes(BNO055.BNO055_TEMP_ADDR)[0]

	def getVector(self, vectorType):
		buf = self.readBytes(vectorType, 6)
		xyz = struct.unpack('hhh', struct.pack('BBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]))
		if vectorType == BNO055.VECTOR_MAGNETOMETER:	scalingFactor = 16.0
		elif vectorType == BNO055.VECTOR_GYROSCOPE:	scalingFactor = 900.0
		elif vectorType == BNO055.VECTOR_EULER: 		scalingFactor = 16.0
		elif vectorType == BNO055.VECTOR_GRAVITY:	scalingFactor = 100.0
		else:											scalingFactor = 1.0
		return tuple([i/scalingFactor for i in xyz])

	def getQuat(self):
		buf = self.readBytes(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
		wxyz = struct.unpack('hhhh', struct.pack('BBBBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]))
		return tuple([i * (1.0 / (1 << 14)) for i in wxyz])

	def readBytes(self, register, numBytes=1):
		return self._bus.read_i2c_block_data(self._address, register, numBytes)

	def writeBytes(self, register, byteVals):
		return self._bus.write_i2c_block_data(self._address, register, byteVals)
	
	def relative(self, direction):
		"""
		計算並回傳相對於指定方向（direction）的陀螺儀偏角（相對值）

		參數:
			direction (int): 目標方向的角度（0~359度）
		
		流程:
		1. 取得當前陀螺儀的Euler角度的yaw值（self.getVector(self.VECTOR_EULER)[0]），
		並將其轉成整數。
		2. 計算目標方向與當前方向的差值，加180度以方便後續模360的運算。
		3. 根據差值計算偏角，結果限制在-180度到180度之間，方便判斷左右偏轉。
		4. 稍作延遲以穩定讀值。
		5. 回傳計算出的相對偏角gyro。
		"""
		value = direction - int(self.getVector(self.VECTOR_EULER)[0]) + 180
		if value >= 0:
			gyro = (value % 360) - 180
		else:
			gyro = 359 - ((-1 - value) % 360) - 180
		time.sleep(0.01)
		return gyro

class D100Reader:
    def __init__(self):
        # 初始化 ROS 節點，命名為 'd100_reader'，anonymous=True 避免名稱衝突
        rospy.init_node('d100_reader', anonymous=True)
        # 訂閱 /scan 主題，接收 LaserScan 訊息並呼叫 callback 處理
        rospy.Subscriber("/scan", LaserScan, self.callback)

        # 初始化距離變數，分別代表前方、左側、右側與右側特定區域距離
        self.front_dist = None
        self.left_dist = None
        self.right_dist = None
        self.right_1 = None

    def callback(self, scan: LaserScan):
        # 接收到雷射掃描資料的 callback 函式
        self.ranges = scan.ranges                  # 距離陣列
        self.angle_min = scan.angle_min            # 雷射起始角度 (弧度)
        self.angle_increment = scan.angle_increment # 角度增量 (弧度)

        angle_distance_pairs = []
        # 遍歷所有距離值，過濾非數值(dist 非 NaN)後，將角度轉為度數並存成 (角度, 距離) 對
        for i, dist in enumerate(self.ranges):
            if not math.isnan(dist):
                angle = self.angle_min + i * self.angle_increment
                deg = math.degrees(angle) % 360
                angle_distance_pairs.append((deg, dist))

        # 分別從角度範圍內取出前方、左側、右側與右側特定區域的距離清單
        front = [d for a, d in angle_distance_pairs if a >= 350 or a <= 10]    # 前方約350~10度範圍
        left  = [d for a, d in angle_distance_pairs if 70 <= a <= 110]         # 左側約70~110度範圍
        right = [d for a, d in angle_distance_pairs if 250 <= a <= 290]        # 右側約250~290度範圍
        right_1 = [d for a, d in angle_distance_pairs if 265 <= a <= 290]      # 右側較窄區域，供特殊判斷使用

        # 取各方向距離的最短值作為障礙物距離，若沒有資料則回傳 -1 表示無效
        self.front_dist = min(front) if front else -1
        self.left_dist = min(left) if left else -1
        self.right_dist = min(right) if right else -1
        self.right_1 = min(right_1) if right_1 else -1
        
    def get_latest_distances(self):
        # 提供外部呼叫取得最新的距離與角度資料
        return self.ranges, self.angle_min, self.angle_increment, self.front_dist, self.left_dist, self.right_dist, self.right_1

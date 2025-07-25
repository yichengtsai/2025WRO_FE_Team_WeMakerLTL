import rospy
import math
import cv2
from sensor_msgs.msg import LaserScan
from function import *
import threading
from queue import Queue
from picamera2 import Picamera2

frame_queue = Queue(maxsize=2)
imcap = Picamera2()
config = imcap.create_preview_configuration(main={"size": (480, 360)})
imcap.configure(config)
imcap.start()
imcap.set_controls({"AeExposureMode": 0, "ExposureValue": 1})

reader = D100Reader()
servo = servo_motor()
Motor = dc_motor()
tcs34725 = TCS34725()
bno = BNO055()
pi = pigpio.pi()
dir_array = [0, 270, 180, 90]
run = True
line_count = 0
gyro_kp = 0.9
round_count = 0

lidar_enable = True  # 是否啟用光達置中
lidar_tolerance = 0.06  # 置中容忍誤差 (公尺)
lidar_kp = 1  # 光達置中比例係數
max_lidar_correction = 40  # 光達校正的最大角度
gyro = 0

# 陀螺儀啟用條件
gyro_enable_threshold = 0.06  # 當左右距離差小於此值時才啟用陀螺儀
gyro_priority_mode = False  # 陀螺儀優先模式（接近置中時切換）

def gyro_read():
    global gyro, gyro_angle, line_count
    
    print('gyro start')
    while run:
        gyro = bno.relative(dir_array[line_count])
        time.sleep(0.01)

def lidar_read():
    global front, left, right
    while run:
        front, left, right = reader.get_latest_distances()
        time.sleep(0.01)
        
def color_read():
    global lum
    tcs34725 = TCS34725()
    while run:
        lum = tcs34725.readluminance()['c']
        time.sleep(0.01)
        
def get_color():
    global gyro, dir_array
    color = 100
    while color > 30:
        color = tcs34725.readluminance()['c']
        time.sleep(0.01)
    color = 0
    get_color = 100
    while color < 30:
        color = tcs34725.readluminance()['c']
        if get_color > color:
            get_color = color
        time.sleep(0.01)
#     print(get_color)
    if get_color > 12:
        print('right turn')
        dir_array = [0, 90, 180, 270]
    else:
        print('left turn')
        dir_array = [0, 270, 180, 90]
        
def get_corrected_lidar_distances():
    """根據陀螺儀角度校正光達距離（相對值）"""
    global left, right, gyro
    if left is None or right is None:
        return left, right
    # 陀螺儀是相對值，範圍大約在-180到180度
    # 只有在陀螺儀值較小時才進行距離校正，避免過度校正
    if abs(gyro) < 30:  # 只有在偏角小於30度時才校正
        # 將陀螺儀角度轉換為弧度
        gyro_rad = math.radians(gyro)
        # 使用三角函數校正距離
        corrected_left = left * math.cos(gyro_rad)
        corrected_right = right * math.cos(gyro_rad)
        # 輕微的角度補償
        angle_compensation = abs(gyro) * 0.005  # 相對值用較小的係數
        if gyro > 0:  # 車身向右偏
            corrected_left -= angle_compensation
            corrected_right += angle_compensation
        elif gyro < 0:  # 車身向左偏
            corrected_left += angle_compensation
            corrected_right -= angle_compensation
        return corrected_left, corrected_right
    else:
        # 偏角太大時不校正，直接返回原始值
        return left, right
    
def get_control_mode():
    """決定使用哪種控制模式"""
    global left, right, gyro_priority_mode
    if left is None or right is None:
        return "gyro_only"
    # 獲取校正後的光達距離
    corrected_left, corrected_right = get_corrected_lidar_distances()
    distance_diff = abs(corrected_left - corrected_right)
    print("dis:", distance_diff)
    # 判斷控制模式
    if distance_diff > gyro_enable_threshold:
        # 距離差較大，優先使用光達校正
        gyro_priority_mode = False
        return "lidar_priority"
    else:
        # 距離差較小，切換到陀螺儀優先模式
        gyro_priority_mode = True
        return "gyro_priority"
    
def get_lidar_correction():
    """計算光達置中的校正角度"""
    global left, right, gyro
    if left is None or right is None:
        return 0
    # 獲取校正後的光達距離
    corrected_left, corrected_right = get_corrected_lidar_distances()
    # 計算校正後的左右距離差
    distance_diff = corrected_left - corrected_right
    # 如果在容忍範圍內，不需要校正
    if abs(distance_diff) <= lidar_tolerance:
        return 0
    # 計算校正角度
    correction = -distance_diff * lidar_kp * 70  # 30是放大係數
    # 限制校正角度範圍
    correction = constrain(correction, -max_lidar_correction, max_lidar_correction)
    return correction

def get_combined_servo_angle():
    """根據控制模式結合陀螺儀和光達的伺服角度"""
    global gyro
    control_mode = get_control_mode()
    gyro_angle = gyro * gyro_kp
    # 光達置中校正角度
    lidar_correction = 0
    if lidar_enable:
        lidar_correction = get_lidar_correction()
    
    # 根據控制模式決定如何結合
    if control_mode == "lidar_priority":
        # 光達優先模式：主要用光達校正，陀螺儀作為輔助
        combined_angle = lidar_correction + gyro_angle * 0.2  # 陀螺儀權重降低
        mode_info = "光達優先"
    elif control_mode == "gyro_priority":
        # 陀螺儀優先模式：主要用陀螺儀校正，光達作為輔助
        combined_angle = gyro_angle + lidar_correction * 0.2  # 光達權重降低
        mode_info = "陀螺儀優先"
    else:
        # 只有陀螺儀
        combined_angle = gyro_angle
        mode_info = "僅陀螺儀"
    
    # 限制總角度範圍
    combined_angle = constrain(combined_angle, -40, 40)
    
    return combined_angle, gyro_angle, lidar_correction, mode_info

def gogo():
    global line_count, gyro, round_count, front
    while round_count < 12:
        print("go")
#         while run:
        while front is None or front > 0.7:
#             servo_angle, gyro_correction, lidar_correction, mode_info = get_combined_servo_angle()
#             servo_angle = constrain(servo_angle, -40, 40)
#             servo.angle(servo_angle)
            servo_angle = gyro * 1.2
            servo_angle = constrain(servo_angle, -40, 40)
            servo.angle(servo_angle)
            time.sleep(0.01)
        if line_count + 1 ==4:
            line_count = 0
        else:
            line_count += 1
        round_count += 1
        if round_count != 12:
            start = time.time()
            while time.time() - start < 0.2 or gyro > 5 or gyro < -5:
                print("gyro:", gyro)
                servo_angle = gyro * 1.2
                servo_angle = constrain(servo_angle, -40, 40)
                servo.angle(servo_angle)
            start = time.time()
            while time.time() - start < 1.5:
                servo_angle, gyro_correction, lidar_correction, mode_info = get_combined_servo_angle()
                servo_angle = constrain(servo_angle, -40, 40)
                servo.angle(servo_angle)
    start = time.time()
    while time.time() - start < 0.4:
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -40, 40)
        servo.angle(servo_angle)
        
    while abs(gyro) > 5:
        servo_angle, gyro_correction, lidar_correction, mode_info = get_combined_servo_angle()
        servo_angle = constrain(servo_angle, -40, 40)
        servo.angle(servo_angle)
    start = time.time()
    while time.time() - start < 0.5:
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -40, 40)
        servo.angle(servo_angle)
    front=2
    while front > 1.6:
        print(front)
        servo_angle = gyro * 1.2
        servo_angle = constrain(servo_angle, -40, 40)
        servo.angle(servo_angle)
    Motor.DC(0)

if bno.begin() is not True:
    print("Error initializing device")
    exit()
time.sleep(1)
bno.setExternalCrystalUse(True)

thread_3 = threading.Thread(target = get_color)
thread_2 = threading.Thread(target = color_read)
thread_1 = threading.Thread(target = gyro_read)
thread_0 = threading.Thread(target = lidar_read)
thread_4 = threading.Thread(target = gogo)

thread_3.start()
thread_2.start()
thread_1.start()
thread_0.start()
thread_4.start()

pi.set_mode(17, pigpio.OUTPUT)
pi.set_mode(27, pigpio.OUTPUT)
pi.set_mode(9, pigpio.INPUT)
pi.set_pull_up_down(9, pigpio.PUD_UP)
pi.write(17, 1)
pi.write(27, 0)
button = 1
while button == 1:
    button = pi.read(9)
    time.sleep(0.01)
pi.write(17, 0)
pi.write(27, 1)
last=0
try:
    while run:
        frame = imcap.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # 先轉回 BGR
        cv2.imshow("Detect", frame)
        if last==0:
            Motor.DC(60)
        last=1
        if cv2.waitKey(10) & 0xFF == ord('q'): 
            imcap.stop()
            run = False
            cv2.destroyAllWindows()
            break
    
finally:
    run = False
    Motor.DC(0)
    servo.angle(0)
    thread_0.join()
    thread_1.join()
    thread_2.join()
    thread_3.join()
    pi.stop()
    print("end")
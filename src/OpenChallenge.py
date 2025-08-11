# === 導入所需函式庫 ===
# 自訂功能(function)、匯入多執行緒(threading)
from function import *
import threading

# 建立 pigpio 介面物件，用於控制 Raspberry Pi 的 GPIO
pi = pigpio.pi()

# PWM 驅動馬達時會在電源線上產生高頻脈衝，造成電壓波動與 EMI（電磁干擾）。
# 這段程式在感測器初始化前先將 PWM 腳位強制設為低電平，等感測器啟動完成後再恢復馬達控制。
pi.set_mode(11, pigpio.LOW)
pi.set_mode(21, pigpio.LOW)

# === 從自訂 function 模組匯入並初始化感測器物件 ===
reader = D100Reader()  # 超音波測距模組
tcs34725 = TCS34725()  # 顏色感測器
bno = BNO055()         # 陀螺儀(IMU) 

# === 計數與狀態控制 ===
line_count = 0                # 記錄當前通過的線數(0~3)，用來對應 dir_array 切換車頭目標方向
total_line_count = 0          # 總共完成的線數
dir_array = [0, 90, 180, 270] # 車頭朝向的方向角度
program_active = True         # 控制整體程式運行，直到全部跑完才結束

# === 光達置中控制參數 ===
lidar_enable = True        # 是否啟用光達置中
lidar_tolerance = 0.06     # 置中容忍誤差 (公尺)
lidar_kp = 1               # 光達置中比例係數

# === 陀螺儀啟用條件 ===
gyro_enable_threshold = 0.06  # 當左右距離差小於此值時才啟用陀螺儀
gyro_priority_mode = False  # 陀螺儀優先模式（接近置中時切換）

def gyro_read():
    """
    以固定時間間隔讀取 BNO055 陀螺儀的相對角度，
    並依照 line_count 對應的方向做修正。
    持續讀取直到 program_active 為 False。
    """
    global gyro, line_count, program_active
    while program_active:
        gyro = bno.relative(dir_array[line_count])
        time.sleep(0.01)

def lidar_read():
    """
    持續從光達（LiDAR）讀取最新距離資料
    並更新前方(front)、左側(left)、右側(right)距離變數
    持續讀取直到 program_active 為 False。
    """
    global front, left, right
    while program_active:
        ranges, angle_min, angle_increment, front, left, right, right_special = reader.get_latest_distances()
        time.sleep(0.01)
        
def color_read():
    """
    以固定時間間隔讀取 TCS34725 光感值，
    持續讀取直到 program_active 為 False。
    """
    global lum, program_active
    while program_active:
        lum = tcs34725.readluminance()['c']
        time.sleep(0.01)
        
def detect_line_color_direction():
    """
    利用光感判斷線條顏色，
    以決定車輛行駛方向。

    流程：
    1. 先等待亮度下降到低於30（偵測到線條開始）。
    2. 在亮度低於30期間持續讀取，記錄最低亮度值（代表線條顏色深淺）。
    3. 根據最低亮度判斷：
       - 亮度 > 12 判定為右轉方向，設定陀螺儀方向陣列為[0, 90, 180, 270]
       - 亮度 ≤ 12 判定為左轉方向，設定陀螺儀方向陣列為[0, 270, 180, 90]
    """
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
    if get_color > 12:
        print('right turn')
        dir_array = [0, 90, 180, 270]
    else:
        print('left turn')
        dir_array = [0, 270, 180, 90]
        
def get_corrected_lidar_distances():
    """
    使用陀螺儀角度對左右光達距離做校正，提升測距準確度。

    - 只有當車身偏角（gyro）小於30度時才進行校正，避免過度修正。
    - 校正方法：根據角度轉換距離，並做微調補償，使距離更貼合實際方向。
    - 偏角太大時，直接回傳原始距離，不做校正。
    """
    global left, right, gyro
    if left is None or right is None:
        return left, right
    if abs(gyro) < 30:
        gyro_rad = math.radians(gyro)
        corrected_left = left * math.cos(gyro_rad)
        corrected_right = right * math.cos(gyro_rad)
        angle_compensation = abs(gyro) * 0.005
        if gyro > 0:  # 車身向右偏
            corrected_left -= angle_compensation
            corrected_right += angle_compensation
        elif gyro < 0:  # 車身向左偏
            corrected_left += angle_compensation
            corrected_right -= angle_compensation
        return corrected_left, corrected_right
    else:
        return left, right
    
def get_control_mode():
    """
    根據左右光達距離差決定車輛控制模式：

    - 若任一距離為None，預設使用陀螺儀模式("gyro_only")。
    - 計算校正後左右光達距離差異。
    - 距離差大於閾值，優先使用光達校正模式("lidar_priority")。
    - 距離差較小，切換至陀螺儀優先模式("gyro_priority")。

    並根據判斷更新gyro_priority_mode旗標。
    """
    global left, right, gyro_priority_mode
    if left is None or right is None:
        return "gyro_only"
    
    corrected_left, corrected_right = get_corrected_lidar_distances()
    distance_diff = abs(corrected_left - corrected_right)

    if distance_diff > gyro_enable_threshold:
        gyro_priority_mode = False
        return "lidar_priority"
    else:
        gyro_priority_mode = True
        return "gyro_priority"

    
def get_lidar_correction():
    """
    計算車輛基於光達距離差的置中校正角度。

    - 先取得校正後的左右光達距離。
    - 計算左右距離差，若差異在容忍範圍內，則不需校正。
    - 超出容忍範圍時，依距離差計算校正角度（放大後乘比例係數）。
    - 將校正角度限制在[-40, 40]度範圍內避免過度調整。
    - 回傳最終校正角度值。
    """
    global left, right, gyro
    if left is None or right is None:
        return 0

    corrected_left, corrected_right = get_corrected_lidar_distances()
    distance_diff = corrected_left - corrected_right

    if abs(distance_diff) <= lidar_tolerance:
        return 0

    correction = -distance_diff * lidar_kp * 70
    correction = constrain(correction, -40, 40)
    return correction

def get_combined_servo_angle():
    """
    根據目前控制模式，結合陀螺儀和光達校正角度計算伺服馬達轉向角度。

    流程說明：
    1. 取得控制模式（光達優先、陀螺儀優先、僅陀螺儀）。
    2. 將陀螺儀角度乘以權重(0.9)。
    3. 若啟用光達校正，計算光達置中校正角度。
    4. 根據控制模式調整陀螺儀和光達角度權重後相加：
       - 光達優先模式：光達校正角度主導，陀螺儀權重較低。
       - 陀螺儀優先模式：陀螺儀校正主導，光達權重較低。
       - 其他情況僅用陀螺儀角度。
    5. 將結果限制在[-40, 40]度內避免過度轉向。
    6. 回傳最終結合角度，用於伺服馬達控制。
    """
    global gyro
    control_mode = get_control_mode()
    gyro_angle = gyro * 0.9
    lidar_correction = 0
    if lidar_enable:
        lidar_correction = get_lidar_correction()
    
    if control_mode == "lidar_priority":
        combined_angle = lidar_correction + gyro_angle * 0.2
    elif control_mode == "gyro_priority":
        combined_angle = gyro_angle + lidar_correction * 0.2
    else:
        combined_angle = gyro_angle

    combined_angle = constrain(combined_angle, -40, 40)
    return combined_angle

if bno.begin() is not True:  # 初始化BNO055陀螺儀，若失敗則顯示錯誤並結束程式
    print("Error initializing device")
    exit()

time.sleep(1)  # 稍作延遲等待硬體穩定
bno.setExternalCrystalUse(True)  # 啟用外部晶振，提高陀螺儀精度
time.sleep(1)

# 啟動多執行緒以非同步方式持續讀取
threading.Thread(target=detect_line_color_direction).start()
threading.Thread(target=color_read).start()
threading.Thread(target=gyro_read).start()
threading.Thread(target=lidar_read).start()

# === 從自訂 function 模組匯入並初始化馬達控制物件 ===
servo = servo_motor()  # 伺服馬達物件
Motor = dc_motor()     # 直流馬達物件

# === 主要繞圈與行駛控制迴圈 ===
"""
啟動直流馬達以固定速度前進。
以 total_line_count 計數，當少於12時，持續偵測前方距離與線數。
在前方無障礙且距離大於0.7公尺時，根據陀螺儀角度微調伺服方向保持直線行駛。
每偵測過一條線（line_count 增加），總線數也累加，模擬車輛通過場地的路線標記。
每次通過線後，根據車輛穩定狀態分別執行陀螺儀校正與結合光達校正的方向調整，確保轉向精準。
繞圈接近尾聲時，會針對陀螺儀角度做最後微調保持車身穩定。
最後，判斷前方距離小於1.6公尺時停止馬達，結束繞圈動作。
"""
Motor.DC(60)
while program_active:
    while total_line_count < 12:
        while front is None or front > 0.7:
            servo_angle = gyro * 1.2
            servo_angle = constrain(servo_angle, -40, 40)
            servo.angle(servo_angle)
            time.sleep(0.01)
        if line_count + 1 ==4:
            line_count = 0
        else:
            line_count += 1
        total_line_count += 1
        if total_line_count != 12:
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

# 停止伺服馬達與直流馬達，停止程式運行
program_active = False
Motor.DC(0)
servo.angle(0)
pi.stop()
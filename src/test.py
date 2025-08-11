from function import *  # 匯入所有自訂功能與類別

pi = pigpio.pi()  # 初始化 pigpio，控制樹莓派GPIO

def Servo_test():
    """伺服馬達測試：轉動伺服到40度，維持2秒後回中"""
    servo = servo_motor()
    servo.angle(40)  # 設定伺服角度為40度
    time.sleep(2)
    servo.angle(0)   # 回到中間位置(0度)

def Motor_test():
    """馬達測試：啟動直流馬達轉速50%，維持3秒後停止"""
    Motor = dc_motor()
    Motor.DC(50)    # 馬達正轉，功率50%
    time.sleep(3)
    Motor.DC(0)     # 停止馬達

def sensor_test():
    """
    感測器測試：
    初始化光感(TCS34725)、光達測距(D100Reader)、陀螺儀(BNO055)
    並持續印出陀螺儀相對角度、絕對角度及光照值，
    以及雷射掃描前、左、右距離資訊
    """
    tcs34725 = TCS34725()
    reader = D100Reader()
    bno = BNO055()
    if bno.begin() is not True:
        print("Error initializing device")  # 若陀螺儀初始化失敗，結束程式
        exit()
    time.sleep(1)
    bno.setExternalCrystalUse(True)  # 啟用外部晶振提高精度

    while True:
        # 印出陀螺儀相對角度、絕對角度與光照感測值
        print(f"rel:{bno.relative(0)} abs:{int(bno.getVector(bno.VECTOR_EULER)[0])} lum:{tcs34725.readluminance()['c']}")
        # 取得雷射距離資料，並印出各方向距離
        ranges, angle_min, angle_increment, front, left, right, right_1 = reader.get_latest_distances()
        print(f"front:{front:.2f} left:{left:.2f} right:{right:.2f}")
        time.sleep(0.01)

if __name__ == "__main__":
    Servo_test()   # 執行伺服馬達測試
    Motor_test()   # 執行直流馬達測試
    sensor_test()  # 執行感測器測試並持續輸出資訊

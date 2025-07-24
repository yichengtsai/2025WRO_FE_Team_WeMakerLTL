# Engineering Report

# 移動性管理

## 底盤

### 驅動系統
#### 後輪直流馬達

在後輪驅動部分使用一顆620RPM 的直流馬達（DC Motor）作為動力來源。直流馬達具有轉速穩定、控制方式簡單的特性。本系統透過金屬齒輪組將馬達輸出連接至變速器，以調整扭力與轉速的傳遞效率，使車輛具備良好的啟動能力與穩定的行進速度。金屬齒輪具有良好的強度與耐磨性，可提升整體傳動系統的壽命與可靠度。

- 優點
  - 控制簡單，可透過 PWM 輕鬆調整速度
  - 可提供穩定扭力，適合驅動車輪
  - 搭配金屬齒輪與變速機構，提升扭力輸出並延長使用壽命
  - 兼具穩定推進與反應速度

<div align="center">
<img width="140" height="150" src="../img/DC_motor.png">
</div>

### 轉向系統
#### Ackermann Steering Mechanism (阿克曼轉向機構介紹)

本設計採用了 **阿克曼轉向機構（Ackermann Steering Mechanism）** 作為前輪轉向方案。該機構透過幾何設計，使車輛在轉彎時前輪能指向同一圓心，內外輪轉向角度不同，符合理想轉彎軌跡。這種設計常見於真實汽車中，能有效提升轉彎穩定性與轉向效率。本車輛的阿克曼機構由光固化3D列印製作，搭配伺服馬達驅動，實現準確且穩定的轉向控制。
- 優點
  - 減少輪胎在轉彎時的滑移與磨損
  - 提高轉彎的穩定性與效率
  - 模擬真實車輛的轉向行為，提升真實感與控制精度
  - 可與伺服馬達整合，實現精準轉角控制

<div align="center">
<img width="300" src="../img/Ackerman_steering.png">
</div>
<div align="center">阿克曼轉向機構</div>

# 電源管理與感測

## 電源(電池、降壓、每個模組供電方式)

## 感測器
### 控制器
#### Raspbrry pi 4(8GB)
- 處理器 (CPU)
    - Broadcom BCM2711
    - 四核心 ARM Cortex-A72 (ARM v8) 64-bit 處理器
    - 時脈：1.5GHz
- 記憶體 (RAM)
    - 8GB LPDDR4 SDRAM
    - 記憶體與 SoC 整合在一個封裝上
- 圖形處理器 (GPU)
    - Broadcom VideoCore VI
    - 支援 OpenGL ES 3.0
- 輸出入埠 (I/O Ports)
    - 2 × USB 3.0
    - 2 × USB 2.0
    - 1 × Gigabit Ethernet (RJ45)
    - 40-pin GPIO header
    - CSI 相機介面
    - DSI 顯示介面
<div align="center">
<img width="300" height="200" src="../img/Raspberry_pi.png">
</div>

### 攝影鏡頭
#### Sony IMX477
- 感測器類型：CMOS（背照式，BSI）
- 解析度：12.3 Megapixels
- 感測器尺寸：1/2.3 英吋
- 像素大小：1.55μm × 1.55μm
- 有效畫素：4056 × 3040
- 輸出格式：
    - RAW Bayer（10-bit / 12-bit）
    - 支援 MIPI CSI-2 數位輸出
<div align="center">
<img width="150" height="120" src="../img/Sony_IMX477.png">
</div>

### 顏色感測器
#### TCS34725
- 感測元件類型RGB + Clear 光感測器
- 通訊介面I²C（最大 400 kHz）
- ADC 解析度16 位元（4 通道同步）
- 光感通道R（紅）、G（綠）、B（藍）、C（清光）
- 增益設定1x、4x、16x、60x
- 整合時間2.4 ms ~ 700 ms（可程式化）
- 紅外線濾光片內建，可提升色彩準確度
- 工作電壓2.7V ~ 3.6V（模組通常支援 3.3V / 5V）
- 輸出資料格式每通道 16 位元數值（R、G、B、C）
<div align="center">
<img width="120" height="120" src="../img/TCS34725.png">
</div>

### 光學雷達
#### LDROBOT D100 LiDAR
- 測距原理	Triangulation（CCD 三角測量）
- 掃描範圍	360° 全方位旋轉
- 測距範圍	0.15 ～ 8 公尺
- 角度解析度	約 1°
- 資料輸出頻率	約 2300 點/秒
- 掃描頻率	6 Hz（固定）
- 測距精度	±3 cm（典型值）
- 通訊介面	UART（3.3V TTL）
- 供電電壓	DC 5V（USB 或 UART）
<div align="center">
<img width="300" height="300" src="../img/D100.png">
</div>

### 降壓板
#### High Current 5A Constant Voltage Constant Current Buck Power Supply Module
- 輸入電壓	DC 4V ～ 38V（推薦 5V～36V）
- 輸出電壓	DC 1.25V ～ 36V（可調）
- 輸出電流	最大 5A（建議持續 4.5A 以下）
- 輸出功率	最大 75W（需加散熱片）
- 轉換效率	高達 96%（視輸入/輸出電壓與電流）
- 恆壓/恆流控制	可透過兩顆可調電位器分別設定輸出電壓與限流值
- 開關頻率	約 180kHz（視版本與晶片而定）
- 保護功能	輸出短路保護、過溫保護、過電流保護
<div align="center">
<img width="250" height="300" src="../img/5A_Buck_Converter.png">
</div>

### 陀螺儀
#### BNO055 IMU
- 感測器內容	三軸加速度計 + 三軸陀螺儀 + 三軸磁力計 + 32-bit Cortex-M0F FUSION MCU
- 輸出數據	Quaternion（四元數）、Euler 角（Yaw, Pitch, Roll）、加速度、角速度、磁場、線性加速度、重力向量等
- 加速度範圍	±2g / ±4g / ±8g / ±16g
- 陀螺儀範圍	±125 / ±250 / ±500 / ±1000 / ±2000 dps
- 磁力計範圍	±1300 μT（X/Y 軸）、±2500 μT（Z 軸）
- 取樣率（輸出率）	最多 100 Hz（視模式）
- 絕對方向誤差	約 ±2.5°（典型值）
- 電壓需求	3.0V ~ 3.6V（模組常含穩壓可支援 3.3V / 5V）
- 通訊介面	I²C（100kHz～400kHz）與 UART（最大460800 bps）
- 內建演算法	Bosch Sensor Fusion Algorithm（可直接輸出姿態角）
<div align="center">
<img width="160" height="120" src="../img/BNO055.png">
</div>

### 馬達控制板
#### L298N
- IC 型號	L298N 雙 H 橋馬達驅動 IC
- 驅動電壓	5V ~ 35V（邏輯電壓 5V）
- 邏輯電壓	5V（模組可內建穩壓器）
- 最大輸出電流	2A / 每通道（瞬間最大可達 3A）
- 驅動通道數	2 通道（可驅動兩個直流馬達或一個雙極步進馬達）
- 控制方式	數位訊號控制（IN1 ~ IN4）、PWM 調速
- 支援功能	正轉、反轉、剎車、停止、PWM 調速
- 功耗	依負載與驅動電壓而異
- 模組接腳說明：
    - 接腳	功能
    - IN1 / IN2	控制馬達A方向
    - IN3 / IN4	控制馬達B方向
    - ENA / ENB	啟用並調速（通常接 PWM）
    - OUT1 / OUT2	馬達A輸出
    - OUT3 / OUT4	馬達B輸出
    - VCC	馬達電源（5V ~ 35V）
    - GND	接地
    - 5V	模組內部邏輯電壓輸出/輸入（若使用板上穩壓器，插跳線帽以供應5V）
<div align="center">
<img width="200" height="200" src="../img/L298N.png">
</div>

### 伺服馬達
#### MG90S
- 工作電壓	4.8 V ～ 6.0 V
- 扭力	約 1.8 kg·cm（4.8 V）
約 2.2 kg·cm（6.0 V）
- 速度	約 0.1 秒 / 60°（4.8 V）
- 工作電流	空載約 100 mA，負載時更高
- 控制方式	PWM 控制，頻率約 50 Hz
- 齒輪材質	金屬齒輪（鋼齒）
- 軸承類型	雙軸承設計，提高穩定性
- 角度範圍	約 180°
<div align="center">
<img width="120" height="120" src="../img/MG90.png">
</div>

### 直流減速馬達
#### JGA25-370
- 工作電壓	3 V ～ 12 V（常用 6 V / 12 V）
- 空載轉速	約 370 RPM（12 V 時）
- 空載電流	約 90 mA ～ 120 mA
- 額定負載電流	約 300 mA ～ 500 mA
- 額定轉速	約 200 ～ 300 RPM（依負載與電壓而異）
- 扭力	約 1.5 ～ 3 kg·cm（依減速比不同）
- 減速比	常見有 1:48、1:64、1:100 等版本
<div align="center">
<img height="200" src="../img/DC_Motor.png">
</div>
感應器規格...
自製上板(上面的電路板)

# 障礙管理

## 資格賽

### 功能概述
我們透過整合陀螺儀與光達，實現車子在限定空間內的繞圈移動行為。整體動作流程分為以下步驟：

一、初始化階段
啟動系統後，進行感測器初始化與通訊設定，同步讀取陀螺儀、光達與光感的資料，並透過光達資料進行初始位置的置中校正。

二、前進與距離判斷
車子向前行駛，光達持續檢測前方牆壁距離，當偵測到接近指定距離時，觸發轉彎程序。

三、轉彎動作
根據當前角度與預設目標角度，控制車子執行陀螺儀角度旋轉。旋轉完成後，再次透過光達執行橫向置中校正。

四、重複流程
校正完成後進入下一段直行流程，如此循環構成完整繞圈動作。

<div align="center">
<img width="auto" height="300" src="../img/Open_Challenge_Flowchart.png">
</div>

### 光達置中校正運算

#### 補償原理:

一、角度轉換與投影修正

假設車身偏轉角度為 gyro（單位：度），轉換為弧度後，將原始距離乘上 cos(gyro)，修正光達讀值的投影誤差。
原因在於：當車身偏轉時，光達實際測得的是與牆面夾角方向的距離，其垂直距離需透過餘弦關係還原。

公式：
```
corrected_left  = left  × cos(gyro)
corrected_right = right × cos(gyro)
```
二、額外補償微調（角度漂移影響）

為進一步提升精度，對左右距離加入一項與角度成比例的補償量 angle_compensation = abs(gyro) × 0.005，模擬由於傾斜造成的視角偏移。

根據偏轉方向，左右距離分別加減該補償量：
```
if gyro > 0:
    corrected_left  -= angle_compensation
    corrected_right += angle_compensation
else if gyro < 0:
    corrected_left  += angle_compensation
    corrected_right -= angle_compensation
```

## 障礙挑戰賽

### 顏色遮罩設定與輪廓擷取

為了辨識場地中的紅色與綠色積木，程式採用了 HSV 色彩空間來進行顏色篩選。與 RGB 不同，HSV 更適合處理顏色辨識的問題，因為它將顏色（Hue）、飽和度（Saturation）與明亮度（Value）分離，能更穩定地適應不同光照條件。

使用 ``` cv2.inRange ``` 函式可以將在範圍內的像素轉為白色（255），不在範圍內的轉為黑色（0），形成二值遮罩。再透過 cv2.findContours 找出輪廓，即可擷取紅綠積木在影像中的位置。這些輪廓後續會用來判斷積木位置與大小，進一步進行避障判斷與行為控制。

根據設定好的色彩範圍，對畫面進行遮罩，找出特定顏色積木的區域
```
green_mask = cv2.inRange(hsv, green_lower, green_upper)
contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

### 計算閃避積木的角度

在辨識出紅色或綠色積木並取得其座標後，系統接著根據積木與畫面中心的相對位置，動態計算車子應該偏轉的角度以進行閃避。這段運算考量了以下幾個因素：

- 水平偏移量（offset）：透過積木中心與畫面中心的 X 軸差距，換算出一個比例值 offset_ratio，代表積木偏離中心的程度。

- 閃避補償角度（extra_angle）：當積木靠近畫面下半部，會根據當前陀螺儀角度（gyro）增加一個額外的閃避角度，使機器人能提早偏轉方向，避免碰撞。

- 總角度合成：將偏移比例轉換為角度，加上補償角，並透過 constrain 限制結果在合理範圍內，確保伺服馬達不會轉動過頭。

座標正規化與偏移量計算
```
frame_width = frame.shape[1]
frame_height = frame.shape[0]
norm_y = block_center_y / frame_height
norm_x = block_center_x / frame_height
frame_center_x = frame_width // 2
offset_ratio = (frame_center_x - block_center_x) / (frame_width // 2)
```
根據條件增加額外偏轉角度
```
if norm_y > 0.5 and norm_x < 0.8 and abs(gyro) > 30:
    extra_angle = 25
elif norm_y > 0.5 and norm_x < 0.8:
    extra_angle = 10
else:
    extra_angle = 0
```
最終角度計算與限制
```
total_angle = (52 * (offset_ratio + 0.8) / 1.6) + extra_angle
final_angle = constrain(total_angle, 0, 50)
```

# 團隊營運

## 工作分配
| 姓名  | 負責項目    | 貢獻說明                                        |
| ------ | ------- | ------------------------------------------- |
| 蔡宜成 | 軟體程式設計  | 撰寫 Raspberry Pi 控制程式，包括感測器資料讀取、影像處理、路徑規劃邏輯。 |
| 林仲斌 | 撰寫Github與硬體電路與整合 | 設計並焊接電路板（含感測器與電源模組接線），負責供電穩定與模組連裝。             |
| 賴孟承 | 撰寫Github與機構設計與製作 | 使用 Onshape 設計底盤與夾爪零件，負責3D列印與組接。          |
## 

# 未來發展

如何更深一步研究
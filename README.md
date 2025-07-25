# WRO 2025 未來工程師 - WeMaker-LTL

## 隊伍資訊

<div align="center"><img src="img/WeMaker - LOGO.png" width="200px"></div>

- **隊伍名稱**： WeMaker-LTL
- **隊員**： 蔡宜成、林仲斌、賴孟承
- **教練**： 楊智仁
- **學校/組織**： TCIVS WeMaker FabLab

## 評分對照表
為方便 WRO 2025 評審快速找到對應文件，以下為附件 C 的 7 項評分標準與相關檔案的對照表：
1. 移動性管理: [/docs/engineering_report.md](docs/engineering_report.md#移動性管理)
2. 電源和感測器:[/docs/engineering_report.md](docs/engineering_report.md#電源管理與感測)
3. 障礙物管理:[/docs/engineering_report.md](docs/engineering_report.md#障礙管理)
4. 圖片-團隊和汽車: [/media/README.md](media/README.md)
5. 完整的影片: [/media/README.md#demonstration-videos](media/README.md#demonstration-videos)
6. 使用GitHub: [/README.md](README.md)
7. 工程因素/技術條件: [/hardware/part_files](/hardware/part_files.md)
8. 評審總體印象: [/README.md/快速開始](README.md#快速開始)

## 快速開始
以下步驟幫助您快速設置並重現我們的專案成果，適用於其他隊伍、開發者或感興趣的使用者。

### 先決條件
- **硬體**：
  - Raspberry Pi 4B (8GB RAM)
  - JGA25-370 直流馬達 (x2) 與 L298N 馬達驅動模組
  - MG90 伺服馬達 (轉向)
  - TCS34725 顏色感測器
  - IMX477 攝影機模組
  - LDRobot D100 LiDAR
  - Adafruit BNO055 IMU
  - 底盤：雷射切割木板、3D 列印支架、LEGO 差速器與輪胎
- **工具**：雷射切割機、3D 印表機、螺絲起子、焊接工具
- **軟體**：Raspberry Pi OS、Python 3、Git、OpenCV、ROS

### 設置步驟
1. **組裝硬體**：
   - 參考 `hardware/circuit.md` 連接電路。
   - 使用 `docs/hardware` 中的 STL 檔案列印感測器與馬達支架。
   - 組裝底盤，確保阿克曼轉向系統（80% 阿克曼率）正確設置。
2. **設置軟體環境**：

   ```bash
   # Clone this repository
   git clone https://github.com/yichengtsai/2025WRO_FE_Team_WeMakerLTL.git
   cd WRO-2025-Future-Engineers-TeamName

   # Copy the image file to the SD card
   Use balenaEtcher copy the operating system to the SD card.
   Link: https://etcher.balena.io/
   ```
3. **配置參數**：
   - 檢查 `src/config/hardware_config.md` 中的硬體引腳與感測器設定，根據您的硬體調整。
4. **執行程式**：
   ```bash
   python src/test.py
   ```
   - 確保 Raspberry Pi 已連接到所有設備。
   - 將機器人放置於場地內，觀察其移動表現。
5. **驗證結果**：
   - 運行 `src/OpenChallenge.py` 與 `src/ObstacleChallenge.py` 檢查模組功能。
   - 參考 [media/README.md Demonstration Videos](media/README.md#demonstration-videos) 對比您的機器人表現。

### 預期成果
- 機器人應能自主在比賽場地內繞行三圈後停止，平均完成時間 60 秒。
- 機器人從停車區域出發，繞行三圈後自組移動到停車區域內，並且平行停進停車格內，平均完成時間 120 秒。

如需詳細說明，請參閱 `docs/engineering_report.md`。

## 硬體架構
- **主控制器**：Raspberry Pi 4B (8GB RAM)，負責處理感測器數據與執行導航演算法。
- **驅動馬達**：JGA25-370 直流馬達，提供差速驅動動力。
- **馬達驅動模組**：L298N，控制 JGA25-370 馬達的速度與方向。
- **轉向馬達**：MG90 伺服馬達，實現精確的阿克曼轉向控制（80% 阿克曼率）。
- **顏色感測器**：TCS34725，用於檢測賽道標記與顏色變化。
- **影像模組**：IMX477 高解析度攝影機，支援視覺導航與障礙辨識。
- **光達**：LDRobot D100 LiDAR，提供 360° 環境掃描與距離測量。
- **慣性測量單元 (IMU)**：Adafruit BNO055，追蹤機器人姿態與方向。
- **底盤**：雷射切割木板搭配 3D 列印支架，結合 LEGO 差速器與輪胎，確保結構穩固與靈活移動。
- **轉向系統**：阿克曼轉向系統（80% 阿克曼率），優化賽道轉彎性能。

詳細硬體設計請參閱 `/hardware/` 中的電路圖、照片與 3D 模型。

## 軟體架構
- **程式語言**：Python 3.8+，運行於 Raspberry Pi 4B。
- **模組化設計**：
  - **感測器與執行器模組** (`src/function.py`)：處理 TCS34725、LDRobot D100、BNO055的數據以及控制JGA25-370 馬達、L298N 驅動器 與 MG90 伺服馬達。
- **核心演算法**：
  - **路徑規劃**：在閃避積木的應用中，是根據積木在影像中的位置，計算出適當的轉向角度，並傳送至伺服馬達控制避開動作。該策略不使用傳統意義上的全區域規劃演算法，而是根據即時畫面中的積木位置做出本地性的轉向調整。
  - **障礙迴避**：為了讓車輛能順利繞過紅色與綠色積木，我們設計了一套簡單有效的閃避方法。當攝影機偵測到畫面中出現積木時，系統會判斷它的位置和面積，確認是否需要進行閃避動作。
  然後會根據積木的偏移程度和遠近調整轉向角度，並將這個角度轉換成伺服馬達的控制指令，讓車子能及時且順暢地避開障礙物。
  - **賽道辨識**：在場地中，地面設有藍色與橘色的標線作為定位依據。由於這兩條線間距接近，當顏色感測器偵測到任何一條時，通常代表車輛已進入彎道區域。
  為確保車輛能順利通過彎道，除了顏色判斷外，我們同步結合陀螺儀的角度資訊，控制車輛轉向的程度與穩定性。當進入轉彎時，系統會根據目前的角度進行角度補償與動作調整，並控制車輛以預定角度平滑轉彎，避免偏離路徑。
- **設定檔**：`src/config/` 包含硬體與導航參數的 YAML 設定檔，方便調整與維護。

詳見 `src/` 中的程式碼與模組說明。

## 儲存庫結構
- **docs/**：
  - `engineering_report.md`:完整的工程報告，涵蓋設計、開發與測試。
  - `testing.md`:測試計劃、結果與影片，展示機器人性能。
- **hardware/**：
  - `part_files.md`:硬體圖檔說明。
  - `circuit.md`:電路圖說明。
- **src/**：
  - `ObstacleChallenge.py`:障礙挑戰賽程式碼，執行避障、影像辨識、...。
  - `OpenChallenge.py`:資格賽程式碼，執行光達車輛置中，...。
  - `test.py`:單元測試程式碼，執行車輛周邊設備掃描與驗證設備正常工作。
- **requirements.txt**: Python依賴項清單。
- **LICENSE**:專案授權(MIT)。
- **CHANGELOG.md**:記錄專案更新的變更日誌。

## 安裝與執行
### 環境設置
1. **硬體準備**：
   - 確保所有硬體（Raspberry Pi 4B、JGA25-370、L298N 等）正確連接，參考 `hardware/Circuit.md`。
   - 安裝 3D 列印零件，參考 `hardware/`。
2. **軟體環境**：
   ```bash
   # 克隆儲存庫
   git clone https://github.com/[您的GitHub用戶名]/W
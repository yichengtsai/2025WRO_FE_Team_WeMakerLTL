## docs
## DXF 檔案說明

本專題機構底盤由三層雷射切割板構成，皆使用 3mm 厚椴木板製作，透過螺絲柱組合而成。
###  分層結構板件
| 層級 | 檔案名稱 | 功能說明 |
|------|-----------|----------|
| 上層 | `Plate_3.DXF` | 作為機構上蓋，用來固定電路板以及鏡頭 |
| 中層 | `Plate_2.DXF` | **專門用來固定 Raspberry Pi**，具備對應螺絲孔與線材穿孔設計 |
| 下層 | `Plate.DXF` | 為底部主結構，用來安裝馬達模組與支撐整體結構 |
### 獨立結構零件
| 檔案名稱 | 功能說明 | 材料建議 |
|----------|----------|-----------|
| `Braking_Arm.DXF` | 差速器固定片，用於卡住後輪差速器的一側 | 3mm 椴木板 |

> 所有 DXF 檔可直接匯入至雷射切割軟體（如 LightBurn、RDWorks）進行輸出，切割前請確認雷射機設定單位為 mm。

### SLDPRT檔案說明
## SolidWorks 零件檔案（.SLDPRT）

本資料夾中包含 1 個使用 SolidWorks 建立的 3D 零件檔（副檔名 `.SLDPRT`），此零件為後輪差速器專用固定片，用於防止差速器過度旋轉或滑動。

| 檔案名稱 | 功能說明 | 開啟方式 | 材料建議 |
|----------|-----------|-----------|-----------|
| `diff_lock_plate.SLDPRT` | 差速器固定片，安裝於後輪系統中限制一側旋轉 | 使用 SolidWorks 2020 或以上版本開啟 | 建議使用 3mm 壓克力或木板雷射切割後加工 |

> 此檔案可依需求轉出為 `.STL`、`.STEP` 或 `.DXF` 格式使用，若需列印或雷切可轉檔後處理。
## SolidWorks 零件檔案（.SLDPRT）

本資料夾 SLDPRT 使用 SolidWorks 建立的 3D 零件檔（副檔名 `.SLDPRT`）

| 檔案名稱 | 元件名稱 | 用途說明 |
|----------|----------|----------|
| `3S Battery.SLDPRT` | Lipo 3S 11.1V 1300mAh 70C | 作作為整體系統的電力來源 |
| `12-0024.SLDPRT` |  |  |
| `Bearing Holder_1.SLDPRT` |  |  |
| `jga25_motor.SLDPRT` |  |  |
| `Braking Arm.SLDPRT` |  |  |
| `D100 Lidar.SLDPRT` |  |  |
| `DC motor.SLDPRT` |  |  |
| `Front wheel.SLDPRT` |  |  |
| `Game Feiled Wall Holder - Inside.SLDPRT` |  |  |

> 此檔案可依需求轉出為 `.STL`、`.STEP` 或 `.DXF` 格式使用，若需列印或雷切可轉檔後處理。


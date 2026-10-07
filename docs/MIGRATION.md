# 遷移記錄

目標：Ubuntu 24.04 / ROS 2 Jazzy。來源 commit：`17eea2d1dd673643332537238bbde6fe2b1c0a71`。

## 選用版本與格式

`launch/hello.launch` 明確啟動 `real_time_6_5.py`，因此以它為主。`real_time_new.py` 使用相同特徵與模型，但使用相對路徑、不同追蹤窗口（15/10 對比 20/12）與部分配對閾值，不再建立第二個容易混淆的可執行版本。

根目錄維持 `launch/`、`src/` 與 CMake 格式，使用 `ament_cmake` 安裝 Python scripts。不是 `ament_python` 套件，因此不需要 setup.py、setup.cfg 或 resource 空檔。不要將舊 ROS 2 空殼套件的這些設定混入本 repo。

| 原始檔 | ROS 2 對應 | 處理 |
|---|---|---|
| launch/hello.launch | launch/hello.launch.py | ROS 2 launch API、可選 RViz / LiDAR / recorder |
| src/real_time_6_5.py | src/real_time_6_5.py + src/tracking_core.py | ROS 與演算法分離，保留模型/特徵並修正追蹤錯誤 |
| src/getdata.py | src/getdata.py | 單次訂閱、任意點數、時間與幀界線、正常 flush/close |
| src/adaboost_trained_data_mess_430.txt | src/ 同名檔 | 原始 bytes 不變 |
| src/multi_tracking_rviz_5_28.rviz | rviz/ 同名檔 | ROS 2 plugins、topic QoS、移除 ROS 1 視窗狀態 |
| CMakeLists.txt / package.xml | 根目錄同名檔 | catkin → ament_cmake，宣告 runtime/test 依賴 |

## 修正內容

- `rospy` → `rclpy.Node`，publisher/subscriber 在建構時建立一次。
- LaserScan 使用 sensor-data QoS，可接收 best-effort LiDAR；RViz 也使用 Best Effort。
- 讀取實際 ranges 長度，移除固定 360 點假設。排除非有限值、零、負值與超出 range_min/max 的回波。
- 無效 ray 中斷群集；完整 360 度掃描可合併跨首尾的群集。空掃描仍會更新追蹤遺失狀態。
- 單群集不再存取不存在的 `Seg[i+1]`；重複點、共線點避免除零、acos 超界與 Heron 根號負值。
- 模型 13 特徵的定義與數值尺度保留（含掃描順序的中位點，以及 circularity 原本 NumPy broadcast 引入的 n 倍尺度）。因模型未重新訓練，不能直接更改特徵尺度；測試與原始純數值函式比對。
- AdaBoost 的特徵索引、權重、閾值、正負方向與嚴格大小比較保持一致。
- 使用 SciPy 的線性指派實作取代原手寫 Hungarian。加上 unmatched 欄位，禁止無效配對；零成本是有效結果，不再被 `ans_mat != 0` 排除。
- 刪除候選時不再讓 list index 與原本迴圈 index 脫節；太遠的腳不會因配對失敗而被丟棄。
- Kalman 轉移矩陣和過程雜訊都依實際訊息時間差計算，不再混用 0.1 / 0.2 秒；去除每幀無條件施加 `[1,1]` 加速度造成的漂移。
- 配對使用當幀預測狀態，漏偵測時繼續預測；Joseph covariance update 保持數值穩定。
- 保留 20 幀窗口 / 至少 12 次命中，以及 5 次雙腳配對確認的設定。每個 track 的 history 正常老化，避免原先重置窗口導致目標永遠不刪除。
- 目標有穩定 ID；刪除時發布 DELETE；Marker 欄位轉成 Python float，使用真實 scan frame/stamp，並設定 lifetime。
- frame 改變、時間倒退或超過 reset_gap 時清除追蹤狀態；相同時間戳的重複幀不增加命中數。
- `stationary_simple_bencnmark.txt` 原本只被轉成未使用的 XYT，已解除 runtime 依賴，原檔保留作為測試資料。
- 安裝後 launch 關閉測試發現 process group 與 launch 重複送出 SIGINT，可能中斷 destroy_node。兩個 entry point 改由 Python 接收 Ctrl+C，清理期間忽略重複 SIGINT，先關檔/銷毀節點，再 shutdown context。
- 追蹤節點以 0.2 秒上限等待訊息，確保沒有掃描資料時也能處理 Ctrl+C。launch 測試要求兩個節點正常退出，禁止以升級訊號強制終止來掩蓋關閉問題。
- recorder 預設存新時間戳檔案，以 exclusive create 防止覆寫；從第一幀起計時，Ctrl+C/到時均 flush/close，移除 `os._exit(0)`。

## 測試與限制

- 本機 Python 3.12 / NumPy 2.3.5 / SciPy 1.17.0：21 個演算法測試通過。涵蓋全部 1,265 幀 benchmark、無效值、任意點數、單群集、特徵與分類分數回歸、靜止與運動目標、配對、漏偵測刪除。
- 非 ROS 環境下，ROS 節點 / launch 測試會明確 skip，不宣稱是 ROS runtime 通過。
- GitHub Actions 第一輪已通過 Jazzy 建置及 30 個 pytest 案例（colcon 含 3 個測試群組，合計回報 33，0 errors / failures / skipped）。[執行記錄](https://github.com/jian960810-hub/multi_tracking/actions/runs/36685601680)。後續另加入安裝後 executable 的 headless launch 啟動測試，最新結果以 Actions 為準。
- GitHub Actions 使用 ROS 2 Jazzy 真實建置與 DDS 測試，驗證 Best Effort LaserScan、Marker 收發、recorder、安裝後的 launch/RViz 資源。硬體驅動選擇測試只檢查 launch 結構，沒有連接實際 USB。
- 這裡沒有 TB3 實機、LiDAR、RViz 桌面，也沒有帶 ground truth 的標註資料；不能以「無例外跑完」推論實際辨識精準度。演算法修正後需要在實際裝置上確認配對與閾值。
- 追蹤以 scan 座標為準，未新增移動平台的 TF/里程計補償。
- 原始 package.xml 的 license 為 TODO；保留此資訊，未擅自替原程式指定授權。

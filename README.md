# multi_tracking — ROS 2 Jazzy

TurtleBot3 2D LiDAR 人腳辨識、多目標追蹤與掃描資料記錄。根目錄是 **ROS 2 Jazzy / Ubuntu 24.04** 套件；`turtlebot3_sample/` 與 `.tar.gz` 保留原始 ROS 1 程式。舊的 Notion 匯出 `old_code/` 已移除。

## 檔案位置

| 路徑 | 用途 |
|---|---|
| `launch/hello.launch.py` | 啟動追蹤、RViz，可選擇同時收資料或啟動本機 LiDAR |
| `launch/lidar.launch.py` | 依 LDS-01 / LDS-02 / LDS-03 選擇官方 ROS 2 驅動 |
| `multi_tracking/real_time_6_5.py` | ROS 2 訂閱 `/scan`，發布追蹤 Marker |
| `multi_tracking/tracking_core.py` | 從主程式分出的分群、13 個特徵、AdaBoost、配對、Kalman 追蹤 |
| `multi_tracking/getdata.py` | 將每幀掃描寫入兩欄 angle/range 文字檔 |
| `resource/adaboost_trained_data_mess_430.txt` | 原始模型，內容未修改 |
| `rviz/multi_tracking_rviz_5_28.rviz` | RViz2 的 LaserScan 與 Marker 顯示 |
| `resource/tracking.yaml` | 分群與追蹤參數 |
| `setup.py`、`setup.cfg`、`package.xml` | `ament_python` 建置、依賴與安裝規則 |
| `test/` | 數值回歸、原始資料重播、真實 ROS 訊息與 launch 測試 |
| `docs/FILE_AUDIT.md` | 每個原始檔案的保留／未複製原因 |
| `docs/MIGRATION.md` | 遷移與錯誤修正細節、測試範圍 |

原始資料夾有 `COLCON_IGNORE`，不會被當成另一個 ROS 2 套件建置。原始程式本身有已知錯誤，請執行根目錄安裝出的 ROS 2 節點。

## Python 套件結構

主要套件依 `ament_python` 格式整理：`launch/`、`multi_tracking/`、`resource/`、`rviz/`、`test/`、`package.xml`、`setup.cfg`、`setup.py`。
`multi_tracking/__init__.py` 讓程式能作為 Python 模組匯入；`resource/multi_tracking` 是 ROS 套件索引標記。
參數與模型也存於 `resource/`。原始備份、文件與自動測試設定仍保留在 repo 中。

從之前的 `ament_cmake` 版本更新時，請先關閉節點、開新終端機，再清除**本套件**的舊建置結果（預設 isolated install）：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/colon_ws
rm -rf build/multi_tracking install/multi_tracking
```

接著依下方步驟重新建置。新執行檔名稱為 `real_time_6_5` 與 `getdata`，不再帶 `.py`；launch 指令不變。

## 安裝與建置

先確認是 Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
```

在工作區的 `src` 下放入這個 repo，例如 `~/colon_ws/src/multi_tracking`。若尚未下載：

```bash
mkdir -p ~/colon_ws/src
cd ~/colon_ws/src
git clone https://github.com/jian960810-hub/multi_tracking.git
```

已有 clone 時，在該 repo 裡 `git pull`，不需要再 clone 一次。建置前安裝依賴：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
# 只有從未初始化 rosdep 時才執行 sudo rosdep init
rosdep update
cd ~/colon_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --packages-select multi_tracking
source install/setup.bash
```

每個新終端機都需要 source Jazzy 與工作區的 `install/setup.bash`。不要在同一個終端機混用 ROS 1 與 ROS 2 環境。

## 開啟 LiDAR

如果 TB3 已經用官方 bringup 發布 `/scan`，直接跳到下一節，不要再啟動第二個驅動。

若整台 TB3 由板載電腦控制，在 **TB3 板載電腦** 依真實型號設定 `TURTLEBOT3_MODEL`、`LDS_MODEL`，執行官方命令：

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

若 LiDAR USB 直接連到這台 Ubuntu（VirtualBox 必須先把 USB 裝置交給 Ubuntu），可只啟動 LiDAR：

```bash
# 範例為 LDS-01；請依硬體改成 LDS-02 或 LDS-03
ros2 launch multi_tracking lidar.launch.py lds_model:=LDS-01 port:=/dev/ttyUSB0
```

必須先安裝對應 ROS 2 驅動：LDS-01 用 `hls_lfcd_lds_driver`；LDS-02 用 `ld08_driver`；LDS-03 用 `coin_d4_driver`。也可依 ROBOTIS 官方 Jazzy 安裝流程安裝 `turtlebot3_bringup` 及其依賴。驅動是依硬體選用的額外套件，不隨本 repo 提供。

## 辨識與追蹤

```bash
ros2 launch multi_tracking hello.launch.py
```

- 預設讀取 `/scan`，發布 `/visualization_marker`（`visualization_msgs/msg/Marker`）。
- 方塊：尚待確認的人員候選；同一目標累積 5 次雙腳配對後改為圓柱。
- Marker 使用掃描訊息的 `frame_id`，RViz 預設固定座標為 `base_scan`。
- 追蹤器只處理感測資料，不發布行走指令。

其他使用方式：

```bash
# 不開圖形介面
ros2 launch multi_tracking hello.launch.py rviz:=false

# 其他 topic / frame；fixed_frame 是 RViz 座標設定，不會偽造 TF
ros2 launch multi_tracking hello.launch.py scan_topic:=/my_scan fixed_frame:=laser

# USB LiDAR 就在本機時，可一起啟動（不要和已啟動的驅動重複）
ros2 launch multi_tracking hello.launch.py start_lidar:=true lds_model:=LDS-01

# 播放有 /clock 的 rosbag 時
ros2 launch multi_tracking hello.launch.py use_sim_time:=true
```

追蹤在 LiDAR 座標下運作；沒有額外的機器人移動補償。現有模型的實際辨識率仍取決於 LiDAR、高度、環境與原訓練資料，需要實機確認。

## 收資料

```bash
# 同時顯示追蹤並記錄；預設第一幀收到後記錄 360 秒
ros2 launch multi_tracking hello.launch.py record_scan:=true

# 只收資料，不執行追蹤；0.0 表示直到 Ctrl+C
ros2 run multi_tracking getdata --ros-args -p duration_seconds:=0.0

# 指定新檔案位置；若檔案已存在會拒絕覆寫
ros2 run multi_tracking getdata --ros-args -p output_file:=/tmp/laser_trial_01.txt -p duration_seconds:=60.0
```

預設存至 `~/multi_tracking_data/laser_日期時間.txt`。每一幀都有 `# scan ...` 註解，記錄時間、frame、點數與量測範圍；下面仍是原本的「角度（弧度）、距離（公尺）」兩欄。`numpy.loadtxt()` 可忽略註解讀取。保留原始 `inf` / `nan` 距離，以免扭曲收集資料；追蹤器則會排除這些無效回波。需要完整 LaserScan、intensities、TF 或 rosbag 重播時，請另用 `ros2 bag record /scan /tf /tf_static`。

## 沒有畫面時

```bash
ros2 topic list
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once --field header
```

若沒有 `/scan`，先排查 TB3 / LiDAR 驅動、USB 與 ROS 網路。若有 `/scan` 但 RViz 顯示 TF 錯誤，將 `fixed_frame` 設成實際掃描 frame，或提供正確的 TF。跨機器時確認 ROS_DOMAIN_ID、DDS 與網路可互通。

## 測試

```bash
cd ~/colon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select multi_tracking --event-handlers console_direct+
colcon test-result --verbose
```

GitHub Actions 使用 `ros:jazzy-ros-base` 真實 ROS 2 環境進行建置與測試；最新結果見 repo 的 **Actions → ROS 2 Jazzy**。演算法測試也能在安裝 NumPy、SciPy、pytest 的 Python 環境執行：`python3 -m pytest test/test_tracking_core.py`。原始 benchmark 只用於回歸測試，不是即時節點的必要檔案。

參考：[ROBOTIS bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)、[Jazzy 官方 bringup 原始碼](https://github.com/ROBOTIS-GIT/turtlebot3/blob/jazzy/turtlebot3_bringup/launch/robot.launch.py)、[ROS 2 QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)。

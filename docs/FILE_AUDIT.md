# 原始檔案盤點

已逐檔讀取資料夾的 47 個檔案，检查 Python 語法、launch 入口、檔案引用與資料格式。以下「未複製」不表示已刪除；所有檔案仍在 turtlebot3_sample/。

| 原始相對路徑 | 決定與原因 |
|---|---|
| `CMakeLists.txt` | 建立根目錄 ament_cmake 設定 |
| `launch/hello.launch` | 複製並轉成 launch/hello.launch.py |
| `launch/sample.launch` | 只列印 LaserScan 的 C++ 示範；收資料需求由 getdata.py 負責 |
| `launch/test.launch` | chatter 示範，不處理人腳追蹤 |
| `package.xml` | 建立根目錄 ROS 2 依賴設定 |
| `src/Ball.txt` | 歷史 angle/range 掃描紀錄，不是即時依賴 |
| `src/Box.txt` | 歷史 angle/range 掃描紀錄，不是即時依賴 |
| `src/New_trainball.txt` | main*/final* 的其他訓練模型；不是 real_time_6_5.py 的模型 |
| `src/New_trainbox.txt` | main*/final* 的其他訓練模型；不是 real_time_6_5.py 的模型 |
| `src/ada_demo.txt` | main*/final* 的其他訓練模型；不是 real_time_6_5.py 的模型 |
| `src/adaboost_trained_data_mess_430.txt` | 目前主入口模型；逐 byte 複製至 src/ |
| `src/adae.txt` | main*/final* 的其他訓練模型；不是 real_time_6_5.py 的模型 |
| `src/ball.txt` | main*/final* 的其他訓練模型；不是 real_time_6_5.py 的模型 |
| `src/final.py` | 先前左右腳濾波/顯示實驗；未被目前主入口引用；原始第 233 行有 TabError |
| `src/final_corss2.txt` | 歷史 angle/range 掃描紀錄，不是即時依賴 |
| `src/final_leg.py` | 先前左右腳濾波/顯示實驗；未被目前主入口引用 |
| `src/final_new_label.py` | 先前左右腳濾波/顯示實驗；未被目前主入口引用 |
| `src/final_rviz.py` | 先前左右腳濾波/顯示實驗；未被目前主入口引用 |
| `src/final_rviz_1.py` | 先前左右腳濾波/顯示實驗；未被目前主入口引用 |
| `src/final_rviz_3.py` | 先前左右腳濾波/顯示實驗；未被目前主入口引用 |
| `src/getdata.py` | 複製並轉 ROS 2 收資料節點 |
| `src/laserball.txt` | 歷史 angle/range 掃描紀錄，不是即時依賴；第 39776 資料列欄位不完整，未用於測試 |
| `src/laserbox.txt` | 歷史 angle/range 掃描紀錄，不是即時依賴 |
| `src/listener.py` | chatter 示範，不處理人腳追蹤 |
| `src/main.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/main1.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/main2.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/main3.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/main4.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/main_ball.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/main_ball_new.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/main_box.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/main_box_new.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/multi_tracking_rviz.rviz` | 其他 RViz1 設定；未被 hello.launch 指定 |
| `src/multi_tracking_rviz_5_28.rviz` | hello.launch 指定；轉 RViz2 後放 rviz/ |
| `src/mutli_tracking_rviz.rviz` | 其他 RViz1 設定；未被 hello.launch 指定 |
| `src/mutli_tracking_rviz2.rviz` | 其他 RViz1 設定；未被 hello.launch 指定 |
| `src/mybasic_shape.py` | 固定幾何 Marker 顯示示範 |
| `src/r_main_ball.py` | 球/箱辨識與濾波實驗；未被 hello.launch 或主入口引用 |
| `src/real_time_6_5.py` | 主入口；ROS 2 節點＋獨立 tracking_core.py |
| `src/real_time_new.py` | 相同模型/演算法的另一版（15 幀窗口與不同閾值）；原 hello.launch 未啟動 |
| `src/sample.cpp` | 只列印 LaserScan 的 C++ 示範；收資料需求由 getdata.py 負責 |
| `src/stationary_simple_bencnmark.txt` | 離線 benchmark；原主程式載入後沒有參與 callback，保留作回歸測試；包含 inf 回波 |
| `src/talker.py` | chatter 示範，不處理人腳追蹤 |
| `src/test3.py` | 讀取 final_corss2.txt 的離線畫圖/濾波實驗 |
| `src/trainball.txt` | main*/final* 的其他訓練模型；不是 real_time_6_5.py 的模型 |
| `src/trainbox.txt` | main*/final* 的其他訓練模型；不是 real_time_6_5.py 的模型 |

壓縮檔額外包含 `src/hello.py`（print hello）及 `src/hello2.py`（雙迴圈印字）兩個示範。其餘 47 個檔案與 GitHub 資料夾逐 byte 相同。壓縮檔保留，兩個示範不加入 ROS 2 套件。

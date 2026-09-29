# Notion 匯出備份（尚非可執行原始碼）

來源：https://app.notion.com/p/3ea4a470e01e80a5a687c57cb0e1ab99
匯出日期：2026-09-29

本資料夾保存 Notion 中可讀取的 46 個檔案頁面，依摺疊層級保留 turtlebot3_sample/launch、turtlebot3_sample/src 與套件設定檔。
這不是從 TurtleBot 原始檔複製的逐位元備份，也尚未完成 ROS 2 遷移。

## 匯出方式

- _notion_snapshot.json 保存完整讀取回應、頁面來源及格式，供核對。
- 同名檔案為便於閱讀而轉換的文字：將 <br> 轉換為換行、移除程式碼圍欄及空區塊標記、還原 Markdown 轉義及自動超連結文字。
- 一般文字區中呈現為粗體的 init/name/main 還原為 Python 的雙底線名稱。
- 未推測或補回已遺失的縮排，未更改演算法，也未補造缺失的資料。
- 保留原始頁面名稱 CMakeList.txt；正式建置所需名稱 CMakeLists.txt 待遷移時處理。
- COLCON_IGNORE 防止 ROS 2 工作區誤將封存的 ROS 1 package.xml 當成待建置套件。

## 已確認的阻礙

1. Python 與 RViz YAML 的多處縮排在讀取內容中缺失。Python 類別、函式及 if/for 的範圍無法可靠還原。
2. Ball.txt、Box.txt、final_corss2.txt、laserball.txt、laserbox.txt 的結尾疑似在資料列中間中斷。Notion 回應未提供 truncated 標記，無法判定是貼入來源或讀取時截斷；不可認定這些檔案完整。
3. real_time_6_5.py 與 real_time_new.py 會開啟 stationary_simple_bencnmark.txt，但此檔不在 46 個頁面中，缺少時啟動會失敗。
4. 尚未確認 Ubuntu、ROS 2 發行版、TB3 型號及 LiDAR 型號。

## 初步依賴判斷（待原始檔核對）

- launch/hello.launch 指向 src/real_time_6_5.py 與 src/multi_tracking_rviz_5_28.rviz，是目前最直接的人腳追蹤入口。
- real_time_6_5.py 讀取 adaboost_trained_data_mess_430.txt 與缺失的 stationary_simple_bencnmark.txt。
- src/getdata.py 用於記錄 LaserScan，屬於使用者要求的收資料功能。
- sample.launch/sample.cpp 是另一個範例入口；test.launch/talker.py/listener.py 是 chatter 示範。
- main*、final*、real_time_new.py 等版本尚待比較，不因檔名或日期就判定無用。

## 下一步

請提供 TurtleBot 上完整的原始資料夾壓縮檔（包含縮排與模型/資料檔），以及 Ubuntu 和 ROS 2 版本。之後核對並更新此備份，再於 repo 根目錄整理 launch/src 等必要檔案、轉換 ROS 2，進行語法、依賴、建置及可用環境下的行為測試。
目前未執行 ROS、ROS 2 或實機測試。

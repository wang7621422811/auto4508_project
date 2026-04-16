Tasks to complete  
1. Drive the robot along a path, specified through a number of given GPS waypoints. The robot has to visit each waypoint before returning to its starting position. 
2. Each waypoint is marked by an orange traffic cone. Whenever a waypoint has  been reached (within 1-2 meters), the robot must take a photo of the marker object and then head towards the next waypoint. Always leave this marker to the robot’s right side. 
3. Between the 1st and 2nd waypoint there will be a number of cones at unknown  intervals, you will need to weave through these cones. 
4. At each waypoint, an additional coloured will be in the vicinity, but at an  unspecified distance. Identify the shape of the object, record a photo of it, and calculate its distance from the waypoint marker. 
5. A summary of the journey should be presented on the completion of driving. 
6. Use the Lidar sensor to avoid collisions with markers, objects and any other  stationary or moving obstacles, such as walls, vehicles, people, bikes, etc. 
7. For safety reasons, implement a Bluetooth link between the robot’s on-board PC  and a gamepad controller:  
    a. ‘X’ button enables automated mode.  In automated mode, use the back pedals as a dead-man switch. If released, the robot has to stop. Revision 1.0  
    b. Button ‘O’ enable manual mode (disable automated mode).  In manual mode, the steering controls can be used to manually drive the robot forward/backward and left/right.



In week 8 you will need to demo part 2 of the project during your allocated workshop time. On the scheduled presentation day (to be confirmed) at the end of the semester, all groups will give a practical demonstration of their projects and answer the project supervisors’ questions re. their implementation.



Resources  Pioneer:  
 ARIA Library: https://github.com/cinvesrob/Aria  Phidget IMU:  
 User Guide: https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=1204 
 Code Samples https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=1204  OAK-D Camera:  
 DepthAI API: https://docs.luxonis.com/projects/sdk/en/latest/ 
 Code Samples: https://docs.luxonis.com/projects/api/en/latest/tutorials/code_samples/  
SICK Lidar:  
 Info/ROS-Driver: https://www.sick.com/fr/en/tim781-2174101/p/p594148  
Lakibeam:  
 ROS2 driver + info: https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver



`dev_pioneer_part_2` 把 `todo.txt` 里的作业要求拆成若干 ROS 2 节点，由 `mission_controller` 做总状态机，其它节点各司其职。下面按 `todo.txt` 条目说明**项目打算怎么实现**（与 `README.md` 里的 “Task Coverage” 一致）。

---

### 1. 沿 GPS 航点行驶并回到起点

- **`mission_controller.py`**：从 `waypoints.yaml` 读航点序列，记录起点 TF，按顺序把路径发给底层导航；最后一个航点后进入 **RETURN**，回到起点。
- **`waypoint_controller.py`**：订阅 `/waypoints`（`nav_msgs/Path`），执行逐点跟踪并在到达时发 `/waypoint_reached`。

仿真里航点多半是 **map 坐标**（配合 SLAM/定位）；真机若用 GPS，需要把经纬度转到 `map` 或与现有控制器同一套坐标，逻辑仍是“有序航点 + 回起点”。

---

### 2. 到达航点（约 1–2 m）拍照、锥桶在右侧

- **到达判定**：`mission_params.yaml` / 参数里的 **`waypoint_reached_radius`（README 写约 1.5 m）**，与作业 1–2 m 一致。
- **拍照**：到达 **AT_WAYPOINT** 状态时调用 **`/capture_photo`**（`vision_detector.py`），照片存到 `~/mission_photos/`。
- **锥桶在右侧**：**`waypoint_controller.py`** 里通过偏置/通过策略让路径偏向“从右侧过锥”（README 称 pass-on-right bias）。

---

### 3. WP1 与 WP2 之间绕未知间隔的锥桶

- **`cone_weaver.py`**：在 **WEAVING** 状态用 **LiDAR 聚类**找锥桶，做左右交替的 **slalom**，完成后发 **`/weave_done`**。
- **`waypoints.yaml`** 里的 **`weave_segment`**、**`weave_cone_positions`** 用于定义区段和先验（仿真世界 `mission_world.sdf` 里也有布置）。

---

### 4. 每个航点附近彩色物体：形状识别、拍照、算距离

- **`vision_detector.py`**：相机 + **HSV 颜色** + **`approxPolyDP` 等多边形近似** 判形状；距离结合 **深度/LiDAR/针孔模型**（README 概述为 HSV + 形状 + LiDAR 或 pinhole 距离）。
- 任务控制器在航点调用 **`/detect_object`**，并订阅 **`/detection_result`** 汇总结果。

---

### 5. 行程结束输出摘要

- **`path_recorder.py`**：记录路径，提供 **`/path_recorder/journey_summary`**（及 README 中的保存地图/CSV 等）。
- **`mission_controller.py`**：任务 **COMPLETE** 时在终端打印并发布摘要（与 README “Journey summary printed to terminal” 一致）。

---

### 6. 用 LiDAR 避障

- **`waypoint_controller.py`**：对前方 LiDAR 扫描做 **减速/停车/转向**（README：forward cone scan, stop/slow/steer）。
- **`cone_weaver.py`** 在绕锥时也会用 LiDAR；整体与 Nav2/局部规划的关系取决于 `mission.launch.py` 里如何叠 `/cmd_vel`。

---

### 7. 蓝牙手柄（X 自动、O 手动、背键死区）

- **`gamepad_controller.py`**：订阅 **`/joy`**，发布 **`/gamepad_mode`**（`"auto"` / `"manual"` / `"stopped"`）和 **`/cmd_vel`**。
- **X** → 自动模式；**O** → 手动；自动模式下 **L2+R2 同时按住** 作为死区开关，松开则发零速度停车（README 与 `gamepad_controller` 文档一致；作业里写的 “back pedals” 在本项目里映射为 **双扳机**）。

---

### 启动与依赖关系（如何实现“整包跑起来”）

- **`mission.launch.py`**：拉起 Gazebo/机器人、Nav2/SLAM、上述节点等（需先按 README **构建 Part 1 的 `p3at`**，再构建 **`pioneer_part2`**）。
- **数据流概要**：手柄 → 模式 → `mission_controller` 状态机 → `waypoint_controller` / `cone_weaver` / `vision_detector` / `path_recorder`；配置在 **`config/waypoints.yaml`** 与 **`config/mission_params.yaml`**。

---

`todo.txt` 末尾列的是 **真机资源**（ARIA、Phidget IMU、OAK-D、SICK/Lakibeam LiDAR 等）；当前仓库 README 描述的是 **ROS 2 Jazzy + Gazebo + 与 Part 1 衔接的仿真栈**。若课程演示用真实 Pioneer，需要把同一套话题/服务接到真实驱动和传感器上，逻辑分工仍可按上表对应。

若你希望针对某一条（例如“右侧过锥”或“距离怎么算”）对照具体代码行说明，可以说一下条目编号。


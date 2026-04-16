# Pioneer P3-AT Part 2 实机操作手册

> 适用版本：ROS 2 Jazzy · pioneer_part2 包 · Ubuntu 24.04

## 目录

1. [硬件清单与连接示意](#1-硬件清单与连接示意)
2. [笔记本电脑环境准备](#2-笔记本电脑环境准备)
3. [机器人上位机环境准备（机载 PC）](#3-机器人上位机环境准备机载-pc)
4. [上电与硬件连接流程](#4-上电与硬件连接流程)
5. [每次出发前的配置确认](#5-每次出发前的配置确认)
6. [输入实地 GPS 坐标](#6-输入实地-gps-坐标)
7. [启动程序（实机运行）](#7-启动程序实机运行)
8. [手柄控制操作](#8-手柄控制操作)
9. [任务完成后的数据查看](#9-任务完成后的数据查看)
10. [调试方法与常用命令](#10-调试方法与常用命令)
11. [常见问题排查](#11-常见问题排查)
12. [快速参考卡](#12-快速参考卡)

---

## 1. 硬件清单与连接示意

### 所需硬件

| 设备 | 型号 | 接口 | 说明 |
|---|---|---|---|
| 机器人底盘 | Pioneer 3-AT | RS-232 / USB | ARIA 驱动控制 |
| 激光雷达 | SICK TIM781 或 Lakibeam | 以太网（RJ-45） | `/scan` 话题 |
| 摄像头 | OAK-D (Luxonis) | USB 3.0 | RGB + 深度 |
| IMU | Phidget Spatial | USB | 姿态数据 |
| GPS 接收器 | 任意 NMEA-0183 GPS | USB-串口 | `/gps/fix` 话题 |
| 手柄 | PS4 DualShock 4 | Bluetooth | 自动/手动切换 |
| 机载 PC | Ubuntu 24.04 + ROS 2 Jazzy | — | 运行所有 ROS 节点 |

### 连接拓扑

```
Pioneer P3-AT 底盘
  └── /dev/ttyUSB0 ──→ 机载 PC (ARIA 驱动)

SICK TIM781 激光雷达
  └── 以太网口 192.168.0.1 ──→ 机载 PC 有线网卡
                                (网卡配置为 192.168.0.100)

OAK-D 摄像头
  └── USB 3.0 ──→ 机载 PC

Phidget IMU
  └── USB ──→ 机载 PC

GPS 接收器
  └── /dev/ttyUSB1 ──→ 机载 PC

PS4 手柄
  └── Bluetooth ──→ 机载 PC

机载 PC ──→ WiFi / SSH ──→ 笔记本电脑（远程监控）
```

---

## 2. 笔记本电脑环境准备

笔记本电脑用于 SSH 远程操控、查看 RViz 或监控日志。

### 2.1 安装 ROS 2 Jazzy（如未安装）

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 2.2 SSH 登录机载 PC

```bash
# 查找机载 PC 的 IP（在机载 PC 上执行）
hostname -I

# 从笔记本登录
ssh robot@<机器人IP>

# 若需要转发图形界面（RViz）
ssh -X robot@<机器人IP>
```

---

## 3. 机器人上位机环境准备（机载 PC）

**以下所有步骤在机载 PC 上执行（SSH 登录后操作）。**

### 3.1 安装 ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 3.2 安装所有硬件驱动包

```bash
sudo apt update

# ARIA 电机驱动（优先用 apt，若无则从源码编译，见 3.3）
sudo apt install -y ros-jazzy-ros2aria || echo "需要从源码编译，见 3.3"

# SICK 激光雷达
sudo apt install -y ros-jazzy-sick-scan-xd

# OAK-D 摄像头（DepthAI）
sudo apt install -y ros-jazzy-depthai-ros

# Phidget IMU
sudo apt install -y ros-jazzy-phidgets-spatial

# GPS 驱动
sudo apt install -y ros-jazzy-nmea-navsat-driver

# 手柄
sudo apt install -y ros-jazzy-joy

# SLAM / Nav2（Part 1 依赖）
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup

# OpenCV / cv_bridge（视觉节点）
sudo apt install -y ros-jazzy-cv-bridge python3-opencv
```

### 3.3 从源码编译 ros2aria（若 apt 无此包）

```bash
mkdir -p ~/aria_ws/src && cd ~/aria_ws/src
git clone https://github.com/amor-ros-pkg/rosaria.git -b jazzy
# 或使用 pioneer-ros2 fork:
# git clone https://github.com/Pioneer-ROSbot/ros2aria.git

cd ~/aria_ws
source /opt/ros/jazzy/setup.bash
colcon build
echo "source ~/aria_ws/install/setup.bash" >> ~/.bashrc
```

### 3.4 设置串口权限（每台机器只需做一次）

```bash
# 添加用户到串口组
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER

# 重新登录使权限生效
exit   # 退出后重新 SSH 登录

# 查看 GPS 接收器的 USB ID（用于创建固定设备名规则）
udevadm info /dev/ttyUSB1 | grep -E "ID_VENDOR_ID|ID_MODEL_ID"

# 创建 udev 规则（根据实际 ID 修改 idVendor/idProduct）
sudo tee /etc/udev/rules.d/99-robot-devices.rules << 'EOF'
# Pioneer 底盘（USB-串口适配器，FTDI）
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="pioneer"
# GPS 接收器（Prolific USB-串口）
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="gps_serial"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

# 验证（需重新插拔设备）
ls -la /dev/pioneer /dev/gps_serial
```

### 3.5 SICK 激光雷达网络配置

SICK TIM781 默认 IP 为 `192.168.0.1`，需把机载 PC 的有线网卡配置为同一子网。

```bash
# 查找有线网卡名称
ip link show

# 永久配置（Ubuntu 24.04 使用 netplan）
sudo tee /etc/netplan/99-lidar.yaml << 'EOF'
network:
  version: 2
  ethernets:
    eth0:                        # 改为实际网卡名（ip link show 查看）
      addresses:
        - 192.168.0.100/24
      dhcp4: false
EOF
sudo netplan apply

# 验证连通性
ping -c 3 192.168.0.1
```

### 3.6 配置蓝牙手柄（PS4 DualShock 4）

```bash
sudo apt install -y bluez
bluetoothctl
```

在 `bluetoothctl` 提示符下依次执行：

```
power on
agent on
default-agent
scan on
```

同时按住手柄 **PS 键 + Share 键** 直到白灯快速闪烁，等待设备出现后：

```
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
quit
```

验证配对成功：

```bash
ls /dev/input/js*       # 应出现 /dev/input/js0
```

### 3.7 配置 GPS（gpsd）

```bash
sudo apt install -y gpsd gpsd-clients

# 配置 gpsd 自动读取 GPS 串口
sudo tee /etc/default/gpsd << 'EOF'
START_DAEMON="true"
USBAUTO="true"
DEVICES="/dev/ttyUSB1"      # 换成实际 GPS 串口（或 /dev/gps_serial）
GPSD_OPTIONS="-n"
EOF

sudo systemctl enable gpsd
sudo systemctl start gpsd

# 验证 GPS 数据（需在室外，等约 1 分钟）
gpsmon                     # 实时监控，Ctrl+C 退出
```

### 3.8 编译 pioneer_part2 工作空间

```bash
cd /home/parallels/auto4508/dev_pioneer_part_2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
echo "构建完成，退出码: $?"

# 写入 bashrc（避免每次手动 source）
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /home/parallels/auto4508/dev_pioneer_part_2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. 上电与硬件连接流程

**每次启动机器人前按以下顺序操作：**

```
步骤 1  连接所有 USB 设备（OAK-D 摄像头、Phidget IMU、GPS 接收器）
步骤 2  连接 SICK 激光雷达网线到机载 PC 有线网口
步骤 3  Pioneer 底盘上电（拨动主开关）
步骤 4  等待底盘蜂鸣声（约 5 秒）
步骤 5  连接 Pioneer 底盘 USB 串口线到机载 PC
步骤 6  开启机载 PC
步骤 7  蓝牙手柄连接（开机后一般自动重连）
步骤 8  移到室外开阔地，等待 GPS 定位（约 1-2 分钟）
```

### 确认所有设备已识别

```bash
# 检查串口设备
ls /dev/ttyUSB*
# 预期：/dev/ttyUSB0 (Pioneer)  /dev/ttyUSB1 (GPS)

# 检查 USB 设备（摄像头和 IMU）
lsusb | grep -iE "luxonis|movidius|phidget"

# 检查网络连通性（SICK 激光雷达）
ping -c 3 192.168.0.1

# 检查手柄
ls /dev/input/js*

# 检查 GPS 服务
sudo systemctl status gpsd
```

---

## 5. 每次出发前的配置确认

### 5.1 检查串口映射

```bash
# 查看内核设备识别日志
dmesg | tail -20

# 若 Pioneer 和 GPS 串口编号与配置不符，有两种解决方法：
# 方法 A：在 robot.launch.py 中修改 port 参数
# 方法 B：使用 udev 规则固定设备名（推荐，见 3.4）
```

### 5.2 确认激光雷达连通

```bash
ping -c 3 192.168.0.1
# 若不通，检查网卡配置：
ip addr show eth0   # 应显示 192.168.0.100/24
```

### 5.3 验证 GPS 有效定位

```bash
# 在室外开阔处，等信号稳定后执行（约 1-2 分钟）
gpspipe -w | python3 -c "
import sys, json
for line in sys.stdin:
    try:
        d = json.loads(line)
        if d.get('class') == 'TPV' and d.get('mode', 0) >= 2:
            print(f'GPS 定位成功: lat={d[\"lat\"]:.7f}, lon={d[\"lon\"]:.7f}')
            break
    except:
        pass
"
```

---

## 6. 输入实地 GPS 坐标

**这是每次去新场地必须执行的步骤。**

### 6.1 在现场采集 GPS 坐标

到达现场后，站在每个锥桶旁边，运行以下命令记录当前坐标：

```bash
gpspipe -w | python3 -c "
import sys, json
for line in sys.stdin:
    try:
        d = json.loads(line)
        if d.get('class') == 'TPV' and d.get('mode', 0) >= 2:
            print(f'纬度: {d[\"lat\"]:.7f}  经度: {d[\"lon\"]:.7f}')
    except:
        pass
"
```

逐个锥桶采集，填入下表：

| 点位 | 说明 | 纬度（latitude） | 经度（longitude） |
|---|---|---|---|
| datum | **起始点/参考原点**（机器人出发位置） | | |
| WP1 | 第 1 个橙色锥桶 | | |
| WP2 | 第 2 个橙色锥桶（穿越锥桶区终点） | | |
| WP3 | 第 3 个橙色锥桶 | | |
| WP4 | 第 4 个橙色锥桶 | | |

### 6.2 更新 waypoints_gps.yaml

```bash
nano /home/parallels/auto4508/dev_pioneer_part_2/src/pioneer_part2/config/waypoints_gps.yaml
```

将采集到的坐标填入（保留至少 5 位小数）：

```yaml
datum:
  latitude:  -31.9799000   # ← 改为实际起点纬度
  longitude: 115.8174000   # ← 改为实际起点经度

waypoints:
  - name: WP1
    latitude:  -31.9796000   # ← 改为 WP1 实际纬度
    longitude: 115.8177000   # ← 改为 WP1 实际经度

  - name: WP2
    latitude:  -31.9802000
    longitude: 115.8171000

  - name: WP3
    latitude:  -31.9797000
    longitude: 115.8179000

  - name: WP4
    latitude:  -31.9803000
    longitude: 115.8170000

weave_segment: [0, 1]   # WP1 → WP2 之间穿越锥桶
```

> **注意**：使用 `--symlink-install` 编译后，YAML 文件修改无需重新编译，重启 launch 即可生效。

### 6.3 验证坐标换算是否合理

启动程序后，检查 ENU 转换结果（WP 之间距离应在 10–100 m 之间）：

```bash
ros2 topic echo /gps_waypoints
# pose.position.x = 东方向（米），pose.position.y = 北方向（米），相对于 datum
```

---

## 7. 启动程序（实机运行）

### 7.1 设置环境

```bash
# 若已写入 ~/.bashrc，直接 source 即可
source ~/.bashrc

# 若未写入，手动 source
source /opt/ros/jazzy/setup.bash
source /home/parallels/auto4508/dev_pioneer_part_2/install/setup.bash
# 若有独立的 ros2aria 工作空间
# source ~/aria_ws/install/setup.bash
```

### 7.2 启动完整任务

```bash
# 标准启动（SICK 激光雷达 + RViz）
ros2 launch pioneer_part2 robot.launch.py

# 使用 Lakibeam 激光雷达
ros2 launch pioneer_part2 robot.launch.py lidar_driver:=lakibeam

# SSH 无图形界面时，关闭 RViz
ros2 launch pioneer_part2 robot.launch.py rviz:=false

# 同时指定多个参数
ros2 launch pioneer_part2 robot.launch.py lidar_driver:=sick rviz:=false
```

### 7.3 启动后的预期日志

```
[robot_state_publisher] Loaded robot description
[ros2aria] Connected to Pioneer robot on /dev/ttyUSB0
[sick_lidar] Connected to SICK TIM781 at 192.168.0.1
[gps_driver] Port opened: /dev/ttyUSB1
[INFO] [gps_converter]: GPS converter ready — waiting for /gps/fix
[INFO] [slam_toolbox]: Mapping started
--- 约 30-120 秒后（GPS 定位成功）---
[INFO] [gps_converter]: ENU origin set from first fix: lat=-31.979900, lon=115.817400
[INFO] [gps_converter]: WP 'WP1': (-31.979600, 115.817700) → (24.31, 33.42) m
[INFO] [gps_converter]: WP 'WP2': (-31.980200, 115.817100) → (-26.87, -33.14) m
[INFO] [gps_converter]: Published 4 converted GPS waypoints on /gps_waypoints
[INFO] [mission_controller]: GPS waypoints received: 4 poses
[INFO] [mission_controller]: IDLE — waiting for automated mode (X button)
```

### 7.4 监控 GPS 定位状态

```bash
# 新开终端
source ~/.bashrc
ros2 topic echo /gps/fix --once
# status.status: 0 = 有定位，-1 = 无信号
```

---

## 8. 手柄控制操作

### 按键功能说明（PS4 DualShock 4）

| 按键 | 功能 |
|---|---|
| **× 键（Cross）** | 切换到**自动任务模式**，机器人开始执行任务 |
| **○ 键（Circle）** | 切换到**手动控制模式** |
| **L2 + R2 同时按住** | 自动模式**死人开关**（必须同时按住，松开立即停止） |
| **左摇杆 上/下** | 手动模式：前进 / 后退 |
| **左摇杆 左/右** | 手动模式：左转 / 右转 |

### 标准任务操作流程

```
1. 确认 launch 已启动，日志显示：
   "IDLE — waiting for automated mode (X button)"

2. 将机器人放置在 datum（出发点）位置，车头朝向任务方向

3. 手柄操作：
   a. 按住 L2 + R2（死人开关，全程不松开）
   b. 按下 × 键
   → 日志显示 "Automated mode engaged — starting mission"

4. 机器人自动执行：
   → 导航到 WP1（保持锥桶在机器人右侧 1-2 m）
   → 到达 WP1：自动拍照 + 检测彩色物体 + 停留 3 秒
   → 穿越 WP1→WP2 之间的锥桶区（slalom 绕桩）
   → 依次到达 WP2、WP3、WP4（每到一个锥桶重复拍照+检测）
   → 自动返回出发点（datum）
   → 打印任务摘要

5. 紧急停止：松开 L2 或 R2 → 机器人立即停止

6. 切换手动：按 ○ 键 → 用左摇杆控制
```

---

## 9. 任务完成后的数据查看

### 9.1 查看任务摘要

任务完成后终端自动打印摘要。也可实时监控：

```bash
source ~/.bashrc
ros2 topic echo /mission_status
```

摘要示例：

```
============================================================
  MISSION JOURNEY SUMMARY
============================================================
  Total mission time : 8m 34s
  Waypoints visited  : 4
  Photos taken       : 4
  Final GPS position : lat=-31.979901  lon=115.817402  alt=12.1 m

  Photos:
    /home/robot/mission_photos/wp_1_20260416_143022.jpg
    ...

  Object detections at waypoints:
    WP1: red circle — distance 1.23 m
    WP2: blue rectangle — distance 1.45 m
    WP3: green sphere — distance 1.08 m
    WP4: yellow triangle — distance 1.67 m
============================================================
```

### 9.2 查看和拷贝照片

```bash
ls ~/mission_photos/

# 从笔记本电脑拷贝照片
scp robot@<机器人IP>:~/mission_photos/* ./local_photos/
```

### 9.3 查看行驶数据

```bash
ls ~/mission_data/
# path_YYYYMMDD_HHMMSS.csv  — 行驶路径坐标（stamp, x, y, yaw）
# map_YYYYMMDD_HHMMSS.pgm   — SLAM 构建的环境地图
# map_YYYYMMDD_HHMMSS.yaml  — 地图元数据

# 查看路径内容
head -5 ~/mission_data/path_*.csv
```

### 9.4 手动保存地图和路径

```bash
ros2 service call /path_recorder/save std_srvs/srv/Empty
```

---

## 10. 调试方法与常用命令

### 10.1 关键话题速查

```bash
ros2 topic list                # 列出所有话题

/gps/fix             # GPS 原始定位（NavSatFix）
/gps_waypoints       # 转换后的路径点（Path）
/gps_odom            # GPS 推算里程计
/scan                # 激光雷达点云
/camera              # RGB 图像（OAK-D）
/oak/stereo/image_raw  # OAK-D 深度图像
/odom                # ARIA 里程计
/tf                  # 坐标变换树
/mission_status      # 任务状态字符串
/mission_active      # 任务激活信号（Bool）
/waypoint_reached    # 到达路径点通知（Int32）
/cmd_vel             # 速度指令（Twist）
/joy                 # 手柄输入
/driven_path         # 已行驶路径（Path）
/detection_result    # 视觉检测结果（JSON String）
```

### 10.2 实时监控命令

```bash
# 机器人当前位置（TF：map → base_link）
ros2 run tf2_ros tf2_echo map base_link

# GPS 定位频率
ros2 topic hz /gps/fix

# 转换后路径点
ros2 topic echo /gps_waypoints

# 激光雷达频率（SICK 约 15 Hz，Lakibeam 约 10 Hz）
ros2 topic hz /scan

# 摄像头帧率
ros2 topic hz /camera

# 手柄输入（按键时有数据）
ros2 topic echo /joy

# 速度指令
ros2 topic echo /cmd_vel

# 任务状态
ros2 topic echo /mission_status

# 检测结果
ros2 topic echo /detection_result
```

### 10.3 单独测试各模块

**测试 GPS 转换节点：**

```bash
ros2 run pioneer_part2 gps_converter --ros-args \
  -p waypoints_gps_file:=$(ros2 pkg prefix pioneer_part2)/share/pioneer_part2/config/waypoints_gps.yaml
```

**测试 SICK 激光雷达：**

```bash
ros2 launch sick_scan_xd sick_tim_7xx.launch.py hostname:=192.168.0.1
ros2 topic hz /scan          # 应约 15 Hz
```

**测试 Lakibeam 激光雷达：**

```bash
ros2 run lakibeam_ros2 lakibeam_node --ros-args -p ip_address:=192.168.0.3
ros2 topic hz /scan
```

**测试 OAK-D 摄像头：**

```bash
ros2 launch depthai_ros_driver camera.launch.py
ros2 topic hz /oak/rgb/image_raw
```

**测试 Phidget IMU：**

```bash
ros2 launch phidgets_spatial spatial.launch.py
ros2 topic echo /imu/data_raw --once
```

**测试手柄输入：**

```bash
ros2 run joy joy_node
ros2 topic echo /joy         # 按任意按键确认有数据
```

**手动触发拍照：**

```bash
ros2 service call /capture_photo std_srvs/srv/Empty
```

**手动触发物体检测：**

```bash
ros2 service call /detect_object std_srvs/srv/Empty
ros2 topic echo /detection_result --once
```

### 10.4 查看 TF 坐标树

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
# 正常应包含：map → odom → base_link → laser
#             map → gps_origin
```

### 10.5 调整日志详细程度

```bash
# 启动时开启全局 DEBUG
ros2 launch pioneer_part2 robot.launch.py --ros-args --log-level DEBUG

# 只对特定节点开启 DEBUG
ros2 run pioneer_part2 gps_converter --ros-args --log-level gps_converter:=DEBUG
```

### 10.6 录制 bag 文件（用于事后分析）

```bash
# 录制关键话题
ros2 bag record -o ~/mission_bag_$(date +%Y%m%d_%H%M%S) \
  /gps/fix /scan /camera /odom /tf /tf_static \
  /mission_status /waypoint_reached /detection_result \
  /cmd_vel /joy /driven_path

# 在实验室回放
ros2 bag play ~/mission_bag_*/ --loop

# 查看 bag 文件信息
ros2 bag info ~/mission_bag_*/
```

---

## 11. 常见问题排查

### 问题 1：Pioneer 底盘无法连接

**错误信息：**
```
[ros2aria] Cannot open device /dev/ttyUSB0: No such file or directory
```

**解决步骤：**

```bash
ls /dev/ttyUSB*                    # 确认串口存在
ls -la /dev/ttyUSB0                # 确认有 rw 权限
sudo chmod 666 /dev/ttyUSB0        # 临时解决
sudo fuser /dev/ttyUSB0            # 检查是否被占用
dmesg | tail -10                   # 查看内核识别日志
```

---

### 问题 2：GPS 长时间无定位

**现象：** `[gps_converter]: Waiting for /gps/fix...` 持续超过 3 分钟

**解决步骤：**

```bash
sudo systemctl status gpsd          # 确认 gpsd 运行
sudo systemctl restart gpsd         # 尝试重启
cat /dev/ttyUSB1                    # 确认有 NMEA 字符串输出
ros2 topic echo /gps/fix            # 确认 ROS 话题有数据
gpspipe -r | head -10               # 查看原始 NMEA 数据

# 必须在室外开阔地（避免高楼、树荫）
# 冷启动最长需等待 5 分钟
```

---

### 问题 3：SICK 激光雷达无数据

**错误信息：**
```
[sick_lidar]: Failed to connect to 192.168.0.1
```

**解决步骤：**

```bash
ping -c 3 192.168.0.1               # 测试连通性
ip addr show eth0                   # 确认 IP 为 192.168.0.100/24

# 若没有 IP，手动配置
sudo ip addr flush dev eth0
sudo ip addr add 192.168.0.100/24 dev eth0
sudo ip link set eth0 up
```

---

### 问题 4：OAK-D 摄像头打开失败

**错误信息：**
```
[oakd_camera]: No OAK-D device found
```

**解决步骤：**

```bash
lsusb | grep -i "03e7"              # 确认设备识别（03e7 = Luxonis）

# 安装/更新 udev 规则
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# 确认使用 USB 3.0 口（蓝色接口），重新插拔后等待 5 秒
```

---

### 问题 5：手柄死人开关无效

**现象：** 按下 × 键后机器人不启动，或启动后立即停止

**解决步骤：**

```bash
ls /dev/input/js0                   # 确认手柄已连接

# 查看轴值（L2 完全按下时 axes[2] 应接近 -1.0）
ros2 topic echo /joy

# 若轴编号与配置不符，修改 mission_params_robot.yaml：
# deadman_axis_l: 2   ← 改为实际 L2 轴编号
# deadman_axis_r: 5   ← 改为实际 R2 轴编号
```

---

### 问题 6：机器人到达路径点后不切换状态

**现象：** 任务卡在 `NAVIGATE`，不转为 `AT_WAYPOINT`

**解决步骤：**

```bash
ros2 topic echo /waypoint_reached   # 确认话题在发布
ros2 run tf2_ros tf2_echo map base_link  # 查看当前位置

# 若 GPS 精度差（>3 m），增大容忍半径
# 编辑 mission_params_robot.yaml：
# waypoint_reached_radius: 3.0      # 从 1.5 增大到 3.0
```

---

### 问题 7：SLAM 地图无法构建

**错误信息：**
```
[slam_toolbox]: Could not get transform from odom to base_link
```

**解决步骤：**

```bash
ros2 run tf2_tools view_frames      # 检查 TF 树
ros2 run tf2_ros tf2_echo odom base_link  # 确认 ARIA 在发布 TF
ros2 topic echo /odom               # 确认里程计话题有数据
```

---

### 问题 8：视觉检测无结果

**现象：** `/detection_result` 收到 `"status": "no_image"` 或 `"nothing_found"`

**解决步骤：**

```bash
ros2 topic hz /camera               # 确认摄像头有图像（约 30 Hz）
ros2 topic echo /camera --once      # 确认有数据

# 若检测颜色/形状不准，调整 mission_params_robot.yaml 中的 HSV 参数：
# orange_h_low: 5
# orange_h_high: 20
# orange_s_low: 150
# orange_v_low: 100
```

---

## 12. 快速参考卡

```
╔══════════════════════════════════════════════════════════════╗
║           Pioneer P3-AT Part 2 快速参考卡                    ║
╠══════════════════════════════════════════════════════════════╣
║  【启动流程】                                                 ║
║  1. 硬件上电，连接所有设备                                    ║
║  2. SSH 登录机载 PC                                          ║
║  3. 更新 GPS 坐标（必须！）:                                  ║
║     nano .../config/waypoints_gps.yaml                       ║
║  4. 启动程序:                                                 ║
║     ros2 launch pioneer_part2 robot.launch.py rviz:=false    ║
║  5. 等待日志: "GPS waypoints received" + "IDLE..."            ║
║  6. 手柄: 按住 L2+R2，按 × → 任务开始                        ║
╠══════════════════════════════════════════════════════════════╣
║  【紧急停止】                                                 ║
║  - 松开手柄 L2 或 R2（死人开关）                             ║
║  - 或执行: ros2 topic pub /cmd_vel                           ║
║      geometry_msgs/msg/Twist '{}' --once                     ║
╠══════════════════════════════════════════════════════════════╣
║  【任务数据位置】                                             ║
║  照片: ~/mission_photos/wp_N_YYYYMMDD_HHMMSS.jpg             ║
║  路径: ~/mission_data/path_YYYYMMDD_HHMMSS.csv               ║
║  地图: ~/mission_data/map_YYYYMMDD_HHMMSS.pgm                ║
╠══════════════════════════════════════════════════════════════╣
║  【常用调试命令】                                             ║
║  ros2 topic echo /mission_status      # 任务状态              ║
║  ros2 topic echo /gps/fix             # GPS 定位              ║
║  ros2 topic echo /gps_waypoints       # 转换后坐标点          ║
║  ros2 topic hz /scan                  # 激光雷达频率          ║
║  ros2 run tf2_ros tf2_echo map base_link  # 机器人位置        ║
╚══════════════════════════════════════════════════════════════╝
```

---

*最后更新：2026-04-16 | 适用包版本：pioneer_part2 v0.0.0*

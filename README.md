# RoboMaster 自瞄程序（入队考核）

订阅游戏发布的 `/image_raw` 图像话题，检测装甲板并用卡尔曼滤波预测其运动，
通过虚拟串口控制炮台转向与开火。

## 环境依赖

- Ubuntu 22.04，ROS 2 Humble（rclcpp、sensor_msgs）
- OpenCV 4（含开发头文件）
- CMake ≥ 3.8，g++（C++17）
- 串口使用 POSIX termios，无额外第三方库

## 编译

```bash
source /opt/ros/humble/setup.bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## 运行

先启动游戏（游戏需要在 ROS 环境下启动，否则不会发布图像话题），然后：

```bash
source /opt/ros/humble/setup.bash
./build/auto_aim
```

在游戏中选择难度并点击「开始游戏」，程序会自动识别本局的虚拟串口并开始控制。
一局结束后程序退出；使用 `run.sh` 可常驻运行，自动等待下一局：

```bash
bash run.sh
```

## 目录结构

```
include/auto_aim/    头文件
src/                 实现
CMakeLists.txt       构建配置
run.sh               一键启动脚本
```

主要模块：

- `AimNode`：主循环（订阅图像、状态机、开火决策）
- `PlateDetector`：装甲板检测与敌我颜色判定
- `PlateTracker`：装甲板跟踪
- `Kalman1DFilter`：一维卡尔曼滤波（单轴匀速模型）
- `LineGuard`：火线拦截检查（避免误伤友方板）
- `SerialTurret`：串口通信

## 串口协议

- 转向：`0x01` + float32（小端，角度制，-180°~180°，0° 为水平向右）
- 开火：`0x02`（单独发送）
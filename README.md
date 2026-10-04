# RoboMaster 自瞄程序（入队考核）

订阅游戏发布的 `/image_raw` 图像话题，检测装甲板并用卡尔曼滤波预测其运动，
通过虚拟串口控制炮台转向与开火。全部操作在终端完成，无需 IDE。

## 环境依赖

- Ubuntu 22.04，ROS 2 Humble（rclcpp、sensor_msgs）
- OpenCV 4（含开发头文件）
- CMake ≥ 3.8，g++（支持 C++17）
- 串口使用 POSIX termios，无额外第三方库

## 准备游戏

比赛用的游戏程序**不在本仓库内**，需要自行获取并解压到任意目录（下称游戏目录）。
下面示例中用 `/path/to/homework2026` 代指，请替换为你自己的实际路径。

游戏必须在 ROS 环境下启动，否则不会发布 `/image_raw` 话题。建议新开一个终端：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/homework2026
./homework2026.sh
```

也可以让它在后台运行，把当前终端留作他用：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/homework2026
nohup ./homework2026.sh > /tmp/homework2026_launch.log 2>&1 &
```

游戏窗口出现后，手动选择难度并点击「开始游戏」。注意只允许运行一个游戏实例，
重复启动会让串口和 ROS 话题错乱。

## 编译

在本仓库根目录下执行：

```bash
source /opt/ros/humble/setup.bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## 运行

游戏运行后，在另一个终端、于本仓库根目录启动自瞄程序：

```bash
source /opt/ros/humble/setup.bash
./build/auto_aim
```

程序启动后会等待游戏的第一帧图像，并持续扫描本局新建的虚拟串口，游戏点
「开始游戏」即自动接管。一局结束后程序退出。

## 一键脚本

`run.sh` 会检查游戏是否在运行、首次运行时自动编译，并常驻等待后续每一局。
游戏目录不在本仓库内，因此需要通过环境变量 `GAME_DIR` 告知：

```bash
GAME_DIR=/path/to/homework2026 bash run.sh
```

若游戏已在运行，或你打算自己手动启动游戏，可以省略 `GAME_DIR`：

```bash
bash run.sh
```

保持该终端开启，`Ctrl+C` 退出。

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


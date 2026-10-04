## 准备游戏

下面示例中 `/path/to/homework2026` ，请替换为自己的实际路径。
新开一个终端：
```bash
source /opt/ros/humble/setup.bash
cd /path/to/homework2026
./homework2026.sh
```

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


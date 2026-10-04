## 编译

```bash
source /opt/ros/humble/setup.bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## 运行

先启动游戏然后：

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


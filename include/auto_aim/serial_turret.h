#ifndef AUTO_AIM_SERIAL_TURRET_H_
#define AUTO_AIM_SERIAL_TURRET_H_

#include <string>

namespace auto_aim {

// 虚拟串口通信：定位游戏进程打开的串口，发送炮台转向/开火指令。
class SerialTurret {
 public:
  SerialTurret() = default;

  // 扫描 /proc 中运行 "homework" 的进程，返回其打开的 /dev/pts/ 路径
  // （fd 编号最大者），未找到返回空串。
  static std::string FindSlave();

  // 以 raw 模式打开串口。成功返回 true。
  bool Open(const std::string& slave);
  void Close();
  bool IsOpen() const { return fd_ >= 0; }

  // 发送炮台转向指令（0x01 + float32 小端角度，度）。失败返回 false。
  bool SendTurn(float angle_deg);
  // 发送开火指令（0x02，独占一包）。失败返回 false。
  bool SendFire();

 private:
  int RawWrite(const unsigned char* data, int len);

  int fd_ = -1;
};

}  // namespace auto_aim

#endif  // AUTO_AIM_SERIAL_TURRET_H_
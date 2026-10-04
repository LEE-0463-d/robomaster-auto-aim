#include "auto_aim/serial_turret.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace auto_aim {

namespace {

constexpr int kMinFd = 3;
constexpr char kProcPath[] = "/proc";
constexpr char kFdDirName[] = "fd";
constexpr char kDevPtsPrefix[] = "/dev/pts/";
constexpr char kProcessKeyword[] = "homework";

bool IsDigits(const std::string& s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) {
    return std::isdigit(c) != 0;
  });
}

}  // namespace

std::string SerialTurret::FindSlave() {
  std::string best_slave;
  int best_fd = -1;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(kProcPath, ec)) {
    if (ec) {
      break;
    }
    if (!entry.is_directory(ec)) {
      ec.clear();
      continue;
    }
    const std::string pid_str = entry.path().filename().string();
    if (!IsDigits(pid_str)) {
      ec.clear();
      continue;
    }
    std::ifstream cmdline(entry.path() / "cmdline", std::ios::binary);
    const std::string cmd((std::istreambuf_iterator<char>(cmdline)),
                          std::istreambuf_iterator<char>());
    if (cmd.find(kProcessKeyword) == std::string::npos) {
      continue;
    }
    std::vector<int> fds;
    const std::filesystem::path fd_dir = entry.path() / kFdDirName;
    for (const auto& fd_entry :
         std::filesystem::directory_iterator(fd_dir, ec)) {
      if (ec) {
        break;
      }
      const std::string fd_name = fd_entry.path().filename().string();
      if (IsDigits(fd_name)) {
        fds.push_back(std::stoi(fd_name));
      }
    }
    ec.clear();
    std::sort(fds.begin(), fds.end());
    for (const int fd : fds) {
      if (fd < kMinFd) {
        continue;
      }
      std::filesystem::path resolve_path = fd_dir / std::to_string(fd);
      const std::string target =
          std::filesystem::read_symlink(resolve_path, ec).string();
      ec.clear();
      if (target.rfind(kDevPtsPrefix, 0) != 0) {
        continue;
      }
      if (fd > best_fd) {
        best_fd = fd;
        best_slave = target;
      }
    }
  }
  return best_slave;
}

bool SerialTurret::Open(const std::string& slave) {
  Close();
  const int fd = ::open(slave.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    return false;
  }
  struct termios t;
  if (::tcgetattr(fd, &t) != 0) {
    ::close(fd);
    return false;
  }
  t.c_iflag = 0;
  t.c_oflag = 0;
  t.c_lflag = 0;  // 保留 c_cflag（波特率配置）
  if (::tcsetattr(fd, TCSANOW, &t) != 0) {
    ::close(fd);
    return false;
  }
  ::tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  return true;
}

void SerialTurret::Close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int SerialTurret::RawWrite(const unsigned char* data, int len) {
  if (fd_ < 0) {
    return -1;
  }
  const ssize_t n = ::write(fd_, data, static_cast<size_t>(len));
  if (n < 0) {
    return -1;
  }
  ::tcdrain(fd_);
  return static_cast<int>(n);
}

bool SerialTurret::SendTurn(float angle_deg) {
  unsigned char buf[5];
  buf[0] = 0x01;
  std::memcpy(buf + 1, &angle_deg, sizeof(angle_deg));
  return RawWrite(buf, 5) == 5;
}

bool SerialTurret::SendFire() {
  const unsigned char buf[1] = {0x02};
  return RawWrite(buf, 1) == 1;
}

}  // namespace auto_aim
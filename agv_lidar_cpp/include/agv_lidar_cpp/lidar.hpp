#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <sockpp/tcp_connector.h>

#include <string>

namespace io
{
constexpr std::size_t BUFF_SIZE = 1024;

class Lidar
{
public:
  Lidar(std::string host = "192.168.192.101", in_port_t port = 2111);

  void start_recv();
  bool recv_loop();

  float angle_res();
  float scan_frq();
  std::vector<float> ranges();
  std::vector<float> intensities();

private:
  sockpp::tcp_connector sock_;
  uint8_t buff_[BUFF_SIZE];

  float angle_res_;
  float scan_frq_;
  std::vector<float> ranges_;
  std::vector<float> intensities_;

  uint16_t last_scan_i_;
  int16_t last_frame_i_;  // signed for -1
  uint16_t points_received;

  void read(uint8_t * bytes, std::size_t size);
  void print(uint8_t * bytes, std::size_t size);
  void write(const std::vector<uint8_t> & bytes);

  uint16_t to_uint16(uint8_t * bytes);
  uint8_t check(uint8_t * bytes, std::size_t size);

  void login();
  void sync_header();
};

}  // namespace io

#endif  // LIDAR_HPP
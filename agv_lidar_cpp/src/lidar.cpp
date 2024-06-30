#include "agv_lidar_cpp/lidar.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <vector>

using namespace std::chrono_literals;

namespace io
{
Lidar::Lidar(std::string host, in_port_t port)
{
  sockpp::initialize();

  sock_ = std::move(sockpp::tcp_connector({host, port}, 1s));
  if (!sock_) throw std::runtime_error("failed to open socket!");

  login();
  set_degrees(-90, 90);
  start_recv();
}

bool Lidar::scan()
{
  sync_header();

  read(&(buff_[4]), 2);

  auto length = to_uint16(&(buff_[4]));
  if (length > BUFF_SIZE) throw std::runtime_error("invalid length: " + std::to_string(length));

  read(&(buff_[6]), length - 6);

  auto checksum = buff_[length - 1];
  if (checksum != check(buff_, length - 1)) throw std::runtime_error("invalid checksum!");

  auto status = buff_[8];
  if (status != 0x01) throw std::runtime_error("invalid status: " + std::to_string(status));

  auto scan_i = to_uint16(&(buff_[9]));
  auto frame_i = buff_[11];
  auto scan_frq = to_uint16(&(buff_[12])) / 1e2;
  auto degree_res = to_uint16(&(buff_[14])) / 1e4;
  auto points_per_scan = to_uint16(&(buff_[16]));
  auto point_i = to_uint16(&(buff_[18]));
  auto points_per_frame = to_uint16(&(buff_[20]));

  if (frame_i == 0) {
    last_frame_i_ = -1;
    last_scan_i_ = scan_i;
    points_received = 0;
  }

  if (last_frame_i_ + 1 != frame_i || last_scan_i_ != scan_i) return false;

  last_frame_i_ = frame_i;
  degree_res_ = degree_res;
  scan_frq_ = scan_frq;

  if (points_per_scan != ranges_.size()) {
    ranges_.resize(points_per_scan);
    intensities_.resize(points_per_scan);
  }

  for (int i = 0; i < points_per_frame; i++) {
    ranges_[point_i + i] = to_uint16(&(buff_[22 + 4 * i])) / 1e3;
    intensities_[point_i + i] = to_uint16(&(buff_[24 + 4 * i]));
  }

  points_received += points_per_frame;
  return points_received == points_per_scan;
}

float Lidar::angle_min() { return min_degree_ / 180.0 * M_PI; }

float Lidar::angle_max() { return max_degree_ / 180.0 * M_PI; }

float Lidar::angle_res() { return degree_res_ / 180.0 * M_PI; }

float Lidar::scan_frq() { return scan_frq_; }

std::vector<float> Lidar::ranges() { return ranges_; }

std::vector<float> Lidar::intensities() { return intensities_; }

void Lidar::read(uint8_t * bytes, std::size_t size)
{
  auto n = sock_.read_n(bytes, size);
  if (n != size) throw std::runtime_error("failed to read!");
}

void Lidar::print(uint8_t * bytes, std::size_t size)
{
  std::cout << std::hex;
  for (std::size_t i = 0; i < size; i++) {
    std::cout.fill('0');
    std::cout.width(2);
    std::cout << static_cast<int>(bytes[i]) << " ";
  }
  std::cout << std::dec << "\n";
}

void Lidar::write(const std::vector<uint8_t> & bytes)
{
  auto n = sock_.write_n(bytes.data(), bytes.size());
  if (n != bytes.size()) throw std::runtime_error("failed to write!");
}

void Lidar::to_bytes(int32_t value, uint8_t * bytes)
{
  bytes[0] = (value >> 24) & 0xFF;
  bytes[1] = (value >> 16) & 0xFF;
  bytes[2] = (value >> 8) & 0xFF;
  bytes[3] = value & 0xFF;
}

uint16_t Lidar::to_uint16(uint8_t * bytes)
{
  return (static_cast<uint16_t>(bytes[0]) << 8) | static_cast<uint16_t>(bytes[1]);
}

uint8_t Lidar::check(uint8_t * bytes, std::size_t size)
{
  uint8_t sum = 0;
  for (std::size_t i = 0; i < size; i++) sum += bytes[i];
  return sum;
}

void Lidar::login()
{
  std::vector<uint8_t> success = {0x02, 0x02, 0x02, 0x02, 0x00, 0x0A, 0x12, 0x01, 0x01, 0x26};
  std::vector<uint8_t> request = {0x02, 0x02, 0x02, 0x02, 0x00, 0x0E, 0x02,
                                  0x01, 0x03, 0xF4, 0x72, 0x47, 0x44, 0x0D};

  write(request);

  std::vector<uint8_t> reply(success.size());
  read(reply.data(), reply.size());

  std::cout << "login reply: ";
  print(reply.data(), reply.size());

  if (reply != success) throw std::runtime_error("failed to login!");
  std::cout << "login success" << std::endl;
}

void Lidar::set_degrees(float min_degree, float max_degree)
{
  int32_t start = (min_degree + 90) * 1e4;
  uint8_t starts[4];
  to_bytes(start, starts);

  int32_t end = (max_degree + 90) * 1e4;
  uint8_t ends[4];
  to_bytes(end, ends);

  std::vector<uint8_t> request = {0x02,      0x02,    0x02,    0x02,      0x00,      0x12,
                                  0x01,      0x1B,    0x01,    starts[0], starts[1], starts[2],
                                  starts[3], ends[0], ends[1], ends[2],   ends[3]};
  auto checksum = check(request.data(), request.size());
  request.push_back(checksum);

  write(request);

  std::vector<uint8_t> reply(request.size());
  read(reply.data(), reply.size());

  std::cout << "set_degrees reply: ";
  print(reply.data(), reply.size());

  // TODO check reply

  min_degree_ = min_degree;
  max_degree_ = max_degree;
}

void Lidar::start_recv()
{
  std::vector<uint8_t> success = {0x02, 0x02, 0x02, 0x02, 0x00, 0x0A, 0x12, 0x31, 0x01, 0x56};
  std::vector<uint8_t> request = {0x02, 0x02, 0x02, 0x02, 0x00, 0x0a, 0x02, 0x31, 0x01, 0x46};

  write(request);

  std::vector<uint8_t> reply(success.size());
  read(reply.data(), reply.size());

  std::cout << "start_recv reply: ";
  print(reply.data(), reply.size());

  if (reply != success) throw std::runtime_error("failed to start_recv!");
  std::cout << "start_recv success" << std::endl;
}

void Lidar::sync_header()
{
  int i = 0;
  while (i < 4) {
    read(&(buff_[i]), 1);

    if (buff_[i] == 0x02)
      i++;
    else
      i = 0;
  }
}

}  // namespace io

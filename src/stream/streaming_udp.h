#ifndef STREAMINGUDP_H
#define STREAMINGUDP_H

#include "../types/types.hpp"

#include <vector>
#include <opencv2/opencv.hpp>

class StreamingUdp {
 public:
  StreamingUdp();

 public:
  auto stream() -> void;

 private:
  auto readHeader(char *buffer, FrameHeader &fh) -> void;
  auto vectorToDouble(std::vector<std::uint8_t> byte_vector) -> std::pair<bool, double>;
  auto readAccelerometerData(double &linearAccelerationX,
                                    double &linearAccelerationY,
                                    double &linearAccelerationZ,
                                    const std::vector<std::uint8_t> &accelero_data) -> bool;

 public:
  auto setListenPort(std::uint16_t listen_port) -> void;
  auto img() const -> const cv::Mat &;

 private:
  inline void debugDataBytes(const std::string &prefix, const std::vector<uint8_t> &bytes)
  {
#if DEBUG_DATA
    std::cout << prefix << " [";
    for (auto byte : bytes) {
      std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << "]" << std::endl;
#endif
  };

 private:
  std::uint16_t m_listen_port = 50000;
  cv::Mat m_img;
};

#endif // STREAMINGUDP_H

#ifndef TCP_SERVER_HPP
#define TCP_SERVER_HPP

#include "../types/types.hpp"

#include <opencv4/opencv2/core.hpp>

#include <mutex>
#include <string>
#include <thread>

namespace tcp {

  class FrameReceiverServer {
   public:
    struct FrameStructure {
      auto operator==(const FrameStructure &other) -> bool
      {
        return other.format == format /*&& other.frame_id == frame_id*/ && other.height == height &&
               other.width == width;
      }
      auto operator==(FrameStructure &&other) -> bool
      {
        return other.format == format && other.frame_id == frame_id && other.height == height &&
               other.width == width;
      }

      std::uint16_t width{0};
      std::uint16_t height{0};
      std::uint16_t frame_id{0};
      EImageFormat format{YUV};
    };

   public:
    FrameReceiverServer(const std::string &server_address, int listening_port);
    ~FrameReceiverServer();
    auto connection() -> bool;

   public:
    auto readFrames() -> void;
    auto getFrame() -> cv::Mat;

   private:
    auto readHeader() -> bool;
    auto readRows() -> void;
    auto readYUV() -> void;
    auto readBGR() -> void;
    auto readMONO() -> void;

    auto debugDataBytes(const std::string &prefix, const std::vector<std::uint8_t> &bytes) -> void;

   private:
    std::string m_server_address;
    int m_listening_port;
    int m_listening_sockfd;
    int m_client_sockfd;
    cv::Mat m_bgr_mat;
    cv::Mat m_yuv_mat;
    std::mutex m_lock;
    std::thread m_listening_thread;
    FrameStructure m_frame_info;
  };

} // namespace tcp

#endif // TCP_SERVER_HPP

#ifndef UDP_SERVER_HPP
#define UDP_SERVER_HPP

#include "../types/types.hpp"

#include <opencv4/opencv2/core.hpp>

#include <mutex>
#include <string>
#include <thread>

namespace udp {

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

   public:
    auto connection() -> bool;
    auto listen() -> void;
    auto getFrame() -> cv::Mat;

   private:
    auto reallocateBuffer(std::size_t size) -> void;
    auto buffering() -> ssize_t;
    auto readHeader() -> bool;
    auto readData() -> void;
    auto readYUV() -> void;
    auto readBGR() -> void;
    auto readMONO() -> void;

    auto debugDataBytes(const std::string &prefix, const std::vector<std::uint8_t> &bytes) -> void;

   private:
    std::atomic_bool m_close{false};

   private:
    std::string m_server_address;
    int m_listening_port;
    int m_listening_sockfd;

    std::uint8_t *m_buffer = nullptr;
    std::size_t m_buff_size = 0;

    cv::Mat m_bgr_mat;
    cv::Mat m_yuv_mat;

    std::mutex m_lock;
    std::thread m_listening_thread;

    FrameStructure m_frame_info;
  };

} // namespace udp

#endif // UDP_SERVER_HPP

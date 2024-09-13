#ifndef UDP_CLIENT_HPP
#define UDP_CLIENT_HPP

#include <opencv4/opencv2/core.hpp>

#include <mutex>
#include <string>
#include <thread>

namespace udp {

  class FrameClient {
   public:
    enum EImageFormat : std::uint16_t { BGR_RAW = 0x002A, YUV = 0x01a6 };
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
    FrameClient(const std::string &address,
                int port,
                const std::string &send_address,
                int send_port,
                const std::string &output_file);
    ~FrameClient();
    void read();

   public:
    auto getFrame() -> cv::Mat;

   private:
    auto readHeader() -> FrameStructure;
    auto fillLines(const FrameStructure &frame_info) -> void;
    auto readLines(FrameStructure frame_info) -> void;

    auto debugDataBytes(const std::string &prefix, const std::vector<std::uint8_t> &bytes) -> void;

   private:
    std::string receive_address_;
    std::string send_address_;
    int receive_port_;
    int send_port_;
    int receive_sockfd_;
    int send_sockfd_;
    std::string output_file_;
    cv::Mat bgr_mat_;
    cv::Mat yuv_mat_;
    std::mutex lock_;
    std::thread receiving_;
  };

} // namespace udp

#endif // UDP_RECORDER_HPP

#include "client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <cstdint> // For uint8_t
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip> // For std::hex and std::setfill
#include <iostream>
#include <unistd.h>
#include <vector>

using namespace udp;

#define SRC_PORT 50000
#define DST_PORT 50002

#define ACK 0xFF
#define NACK 0xFE

FrameClient::FrameClient(const std::string &receive_address,
                         int receive_port,
                         const std::string &send_address,
                         int send_port,
                         const std::string &output_file)
  : receive_address_(receive_address),
    receive_port_(receive_port),
    send_address_(send_address),
    send_port_(send_port),
    output_file_(output_file),
    receive_sockfd_(-1),
    send_sockfd_(-1),
    bgr_mat_(cv::Mat::zeros(600, 800, CV_8UC3)),
    yuv_mat_(cv::Mat::zeros(600, 800, CV_8UC1))
{
}

FrameClient::~FrameClient()
{
  lock_.unlock();

  receiving_.join();
  close(receive_sockfd_);
  close(send_sockfd_);
}

auto FrameClient::readHeader() -> FrameStructure
{
  constexpr int header_buffer_size = 1024;
  char header_buffer[header_buffer_size];

  constexpr uint8_t begin_code = 0xb8;
  constexpr uint16_t yuv_code = 0x01a6;

  // Example:
  // b8 01a6 0280 01e0 0023 08fc
  uint16_t width = 0;
  uint16_t height = 0;
  uint16_t frame_id = 0;
  uint16_t ecc = 0;
  int total_length = 0;
  EImageFormat frame_format = EImageFormat::BGR_RAW;
  while (true) {
    int len = recv(receive_sockfd_, header_buffer, sizeof(header_buffer), 0);
    if (len <= 0) {
      std::cerr << "Nothing to read - recv code = " << len << std::endl;
      continue;
    }
    // Convert to vector<uint8_t>
    std::vector<uint8_t> vec(header_buffer,
                             header_buffer + sizeof(header_buffer) / sizeof(header_buffer[0]));
    // Output the contents of the vector
    bool found = false;
    auto it = vec.begin();
    while (it != vec.end()) {
      if (*it == begin_code) {
        found = true;
        break;
      }
      ++it;
    }
    if (!found) {
      continue;
    }
    for (std::size_t i = 0; i < vec.size(); ++i) {
      // start of frame 0xb8
      uint8_t code = vec[i++];
      std::cerr << "begin code at " << i << " is " << code << std::endl;
      if (code != begin_code) {
        continue;
      }
      // test YUV frame encoding
      uint16_t frame_encoding = vec[i++] << 8;
      frame_encoding += vec[i++];
      if (frame_encoding == yuv_code) {
        frame_format = EImageFormat::YUV;
        std::cerr << "YUV header format detected: " << frame_encoding << std::endl;
      }

      // get width
      width = vec[i++] << 8;
      width += vec[i++];
      // get height
      height = vec[i++] << 8;
      height += vec[i++];
      // get frame id
      frame_id = vec[i++] << 8;
      frame_id += vec[i++];
      // get validation ecc code
      ecc = vec[i++] << 8;
      ecc += vec[i++];
      const uint16_t computed_ecc = (code * code + frame_encoding) / (width * 3 - height) * 100;
      if (computed_ecc == ecc) {
        // Send the begin of frame confirmation
        // FF XX XX -> XX XX is the frame ID encoded in on tow bytes (maximum value 65535)
        std::vector<std::uint8_t> response{
            0xFF,
            vec[7],
            vec[8],
        };
        debugDataBytes("Sending response", response);

        std::size_t bytes_sent = sendto(send_port_,
                                        &response[0],
                                        response.size(),
                                        0,
                                        (struct sockaddr *) &send_address_,
                                        sizeof(send_address_));
        if (bytes_sent < 0) {
          std::cerr << "Error sending data" << std::endl;
          continue;
        }

        std::cout << "Header of frame " << frame_id << " validation successfull: " << ecc
                  << std::endl;
        return {width, height, frame_id, frame_format};
      }
      else {
        std::cerr << "Failed to validate ECC: " << ecc << std::endl;
        continue;
      }
    }
  }
}

void FrameClient::readLines(FrameStructure frame_info)
{
  // Create a YUV matrix (assuming YUV_420_888 format)
  yuv_mat_ = cv::Mat(frame_info.height + frame_info.height / 2, frame_info.width, CV_8UC1);

  // While receiving
  // FF frame_id(2bytes) row_id(2bytes)

  // For response
  // ACK = 0xFF and NACK = 0xFE
  // FF 00 00 00 00
  // 1st byte = ACK or NACK
  // 2nd to 3rd bytes = frameId
  // 4th to 5th bytes = rowId

  receiving_ = std::thread([this, frame_info]() {
    while (true) {
      const int buffer_size = frame_info.width + 6;
      std::vector<uint8_t> vec(buffer_size, 0);

      std::size_t n_data = 0;
      while (n_data < yuv_mat_.total()) {
        int len = recv(receive_sockfd_, &vec[0], buffer_size, 0);
        if (len > 0) {
          // TODO
          // Parse FF frame_id row_id etc.
          // Send the begin of frame confirmation
          std::vector<std::uint8_t> response{
              0xFF,
              vec[7],
              vec[8],
          };
          debugDataBytes("Reading line", vec);
          debugDataBytes("Sending response", response);

          std::size_t bytes_sent = sendto(send_sockfd_,
                                          &response[0],
                                          response.size(),
                                          0,
                                          (struct sockaddr *) &send_address_,
                                          sizeof(send_address_));
          if (bytes_sent < 0) {
            std::cerr << "Error sending data" << std::endl;
            continue;
          }

          std::lock_guard<std::mutex> lk(lock_);
          // Get row data
          for (std::size_t pos = 0; pos < len; ++pos) {
            yuv_mat_.at<uchar>(n_data + pos) = vec[pos + 5];
          }
          // std::cout << "New data len: " << len << std::endl;
          // std::cout << "Data size : " << n_data << std::endl;
          n_data += len;
        }
      }
      std::cout << "Whole frame parsed" << std::endl;

      auto new_frame_info = readHeader();
      if (frame_info == new_frame_info) {
        std::cout << "New header" << std::endl;
        break;
      }
    }
  });
}

void FrameClient::debugDataBytes(const std::string &prefix, const std::vector<uint8_t> &bytes)
{
  std::cout << prefix << " [";
  for (auto byte : bytes) {
    std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
  }
  std::cout << "]" << std::endl;
}

void FrameClient::fillLines(const FrameStructure &frame_info)
{
  // Create a YUV matrix (assuming YUV_420_888 format)
  yuv_mat_ = cv::Mat(frame_info.height * 3 / 2, frame_info.width, CV_8UC1);

  receiving_ = std::thread([this, frame_info]() {
    std::uint8_t begin_code{0xff};

    // Height is 3/2 times the Y plane height
    // Access individual planes (pointers to data)
    cv::Mat y_plane = yuv_mat_.rowRange(0, frame_info.height);
    cv::Mat u_plane =
        yuv_mat_.rowRange(frame_info.height, frame_info.height + frame_info.height / 4);
    cv::Mat v_plane =
        yuv_mat_.rowRange(frame_info.height + frame_info.height / 4, frame_info.height * 3 / 2);

    std::uint8_t channel{0};
    std::uint16_t row_pos{0};
    std::uint16_t width{0};

    const int row_buffer_size = frame_info.width + 6;
    std::vector<uint8_t> vec(row_buffer_size, 0);

    while (true) {
      int len = recv(receive_sockfd_, &vec[0], vec.size(), 0);
      if (len > 0) {
        if (vec[0] != begin_code) {
          continue;
        }
        // Get row data
        std::size_t i = 1;
        channel = vec[i++];
        row_pos = vec[i++] << 8;
        row_pos += vec[i++];
        width = vec[i++] << 8;
        width += vec[i++];
        // Get the line
        if (width == frame_info.width && row_pos < width) {
          if (channel == 0) {
            for (std::size_t x = 0; x < width; ++x) {
              y_plane.at<uchar>(row_pos, x) = vec[i++];
            }
          }
          else if (channel == 1) {
            for (std::size_t x = 0; x < width; ++x) {
              u_plane.at<uchar>(row_pos * width / 2 + x) = vec[i++];
            }
          }
          else if (channel == 2) {
            for (std::size_t x = 0; x < width; ++x) {
              v_plane.at<uchar>(row_pos * width / 2 + x) = vec[i++];
            }
          }
        }
      }
    }
  });
}

void FrameClient::read()
{
  struct sockaddr_in self_addr;

  // 1. Create receive socket
  {
    receive_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (receive_sockfd_ < 0) {
      std::cerr << "Socket creation failed" << std::endl;
      return;
    }

    memset(&self_addr, 0, sizeof(self_addr));

    // Fill server information
    self_addr.sin_family = AF_INET;
    self_addr.sin_port = htons(receive_port_);
    self_addr.sin_addr.s_addr = inet_addr(receive_address_.c_str());

    // Bind the socket with the server address
    auto res = bind(receive_sockfd_, (const struct sockaddr *) &self_addr, sizeof(self_addr));
    if (res != 0) {
      std::cerr << "Bind failed with error: " << res << " -> " << strerror(errno) << std::endl;
      close(receive_sockfd_);
      return;
    }
  }
  std::cout << "Receiving socket for UDP stream created..." << std::endl;

  // 2. Create send socket
  {
    send_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_sockfd_ < 0) {
      std::cerr << "Error creating socket" << std::endl;
      return;
    }

    // 2. Specify the server address and port
    struct sockaddr_in device_addr;
    memset(&device_addr, 0, sizeof(device_addr)); // Zero out the struct
    device_addr.sin_family = AF_INET;             // Use IPv4
    device_addr.sin_port = htons(send_port_);     // Destination port

    // Set destination IP address (replace with actual IP)
    if (inet_pton(AF_INET, send_address_.c_str(), &device_addr.sin_addr) <= 0) {
      std::cerr << "Invalid address or address not supported" << std::endl;
      close(send_sockfd_);
      return;
    }
  }
  std::cout << "Sending socket for UDP acknowledgement created..." << std::endl;

  std::cout << "Recording UDP stream..." << std::endl;
  const auto &frame_info = readHeader();
  // fillLines(frame_info);
  readLines(frame_info);
}

cv::Mat FrameClient::getFrame()
{
  std::lock_guard<std::mutex> lk(lock_);

  // Convert NV21 to BGR
  // cv::cvtColor(yuv_mat_, bgr_mat_, cv::COLOR_YUV2BGR_NV21);

  // return bgr_mat_;
  return yuv_mat_;
}

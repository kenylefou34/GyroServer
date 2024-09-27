#include "server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip> // For std::hex and std::setfill
#include <iostream>
#include <unistd.h>
#include <vector>

using namespace tcp;

#define TCP_SOCKET_BUFFER_BYTES 212992

#define BUFFER_SIZE TCP_SOCKET_BUFFER_BYTES
#define N_MAX_CONNECTION_ATTEMPT 3
#define CONNECTION_TIMEOUT 60 // Timeout in seconds
#define DEBUG_DATA false

FrameReceiverServer::FrameReceiverServer(const std::string &server_address, int listening_port)
  : m_server_address(server_address),
    m_listening_port(listening_port),
    m_listening_sockfd(-1),
    m_client_sockfd(-1),
    m_bgr_mat(cv::Mat::zeros(600, 800, CV_8UC3)),
    m_yuv_mat(cv::Mat::zeros(600, 800, CV_8UC1))
{
}

FrameReceiverServer::~FrameReceiverServer()
{
  m_lock.unlock();

  m_listening_thread.join();
  close(m_listening_sockfd);
}

auto FrameReceiverServer::connection() -> bool
{
  // 1. Create the socket
  m_listening_sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (m_listening_sockfd < 0) {
    std::cerr << "Socket creation failed!" << std::endl;
    return false;
  }
  std::cout << "Socket created successfully." << std::endl;

  // 2. Configuring the socket
  int opt = 1;
  if (setsockopt(m_listening_sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
    std::cerr << "Error while configuring the socket" << std::endl;
    close(m_listening_sockfd);
    return false;
  }

  // 3. Configure server address
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY; // Accept connexion from any
  server_addr.sin_port = htons(m_listening_port);

  // Bind socket to the specified adress and port
  if (bind(m_listening_sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "Error while binding socket" << std::endl;
    close(m_listening_sockfd);
    return false;
  }

  // Server is listening to the port
  if (listen(m_listening_sockfd, N_MAX_CONNECTION_ATTEMPT) < 0) {
    std::cerr << "Error while listening to the socket" << std::endl;
    close(m_listening_sockfd);
    return false;
  }

  std::cout << "Server listening on port: " << m_listening_port << std::endl;

  // Accept client connection
  struct sockaddr_in client_address;
  socklen_t addrlen = sizeof(client_address);
  m_client_sockfd = accept(m_listening_sockfd, (struct sockaddr *) &client_address, &addrlen);
  if (m_client_sockfd < 0) {
    std::cerr << "Error while accpeting the connection" << std::endl;
    close(m_listening_sockfd);
    return false;
  }

  char client_ip[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &(client_address.sin_addr), client_ip, INET_ADDRSTRLEN);

  std::cout << "The client `" << client_ip << "` is connected. Then we can read data..."
            << std::endl;

  return true;
}

void FrameReceiverServer::readFrames()
{
  m_listening_thread = std::thread([this]() {
    while (true) {
      if (readHeader()) {
        readRows();
      }
    }
  });
}

auto FrameReceiverServer::readHeader() -> bool
{
  std::uint8_t buffer[BUFFER_SIZE];

  constexpr uint8_t begin_code = 0xb8;

  // 4. Receive data from the server
  int bytes_received = read(m_client_sockfd, buffer, BUFFER_SIZE);
  if (bytes_received < 0) {
    std::cerr << "Receiving data failed!" << std::endl;
    return readHeader();
  }
  else if (bytes_received == 0) {
    std::cout << "Connection closed by server." << std::endl;
    connection();
    return readHeader();
  }
  else {
#if DEBUG_DATA
    debugDataBytes("Header data", header_buffer);
#endif
    // Copy buffer
    std::vector<uint8_t> header_buffer(buffer, buffer + sizeof(buffer) / sizeof(std::uint8_t));
    // Clear buffer
    memset(buffer, 0, BUFFER_SIZE);

    // Example:
    // b8 01a6 0280 01e0 0023 08fc
    uint16_t width = 0;
    uint16_t height = 0;
    uint16_t frame_id = 0;
    uint16_t ecc = 0;
    EImageFormat frame_format = EImageFormat::BGR_RAW;

    std::size_t i = 0;
    // start of frame 0xb8
    uint8_t code = header_buffer[i++];
    // std::cerr << "begin code at " << i << " is " << code << std::endl;
    if (code != begin_code) {
      return false;
    }
    // test YUV frame encoding
    uint16_t frame_encoding = header_buffer[i++] << 8;
    frame_encoding += header_buffer[i++];
    if (frame_encoding == EImageFormat::YUV) {
      frame_format = EImageFormat::YUV;
      std::cerr << "YUV header format detected: " << frame_encoding << std::endl;
    }
    else if (frame_encoding == EImageFormat::BGR_RAW) {
      frame_format = EImageFormat::BGR_RAW;
      std::cerr << "BGR header format detected: " << frame_encoding << std::endl;
    }
    else if (frame_encoding == EImageFormat::MONO) {
      frame_format = EImageFormat::MONO;
      std::cerr << "MONO header format detected: " << frame_encoding << std::endl;
    }
    else {
      std::cerr << "Uknown header format detected: " << frame_encoding << std::endl;
      return false;
    }

    // get width
    width = header_buffer[i++] << 8;
    width += header_buffer[i++];
    // get height
    height = header_buffer[i++] << 8;
    height += header_buffer[i++];
    // get frame id
    frame_id = header_buffer[i++] << 8;
    frame_id += header_buffer[i++];
    // get validation ecc code
    ecc = header_buffer[i++] << 8;
    ecc += header_buffer[i++];
    const uint16_t computed_ecc = (code * code + frame_encoding) / (width * 3 - height) * 100;
    if (computed_ecc == ecc) {
      std::cout << "Header of frame " << frame_id << " validation successfull: " << ecc
                << std::endl;
      m_frame_info = FrameStructure{width, height, frame_id, frame_format};
      return true;
    }
    else {
      std::cerr << "Failed to validate ECC: " << ecc << std::endl;
    }
  }

  return false;
}

void FrameReceiverServer::readRows()
{
  {
    switch (m_frame_info.format) {
      case EImageFormat::YUV: {
        // Create a YUV matrix (assuming YUV_420_888 format)
        std::lock_guard<std::mutex> lk(m_lock);
        m_yuv_mat =
            cv::Mat(m_frame_info.height + m_frame_info.height / 2, m_frame_info.width, CV_8UC1);
      }
        readYUV();
        break;
      case EImageFormat::BGR_RAW: {
        // Create a BGR matrix (assuming RAW format)
        m_bgr_mat = cv::Mat(m_frame_info.height, m_frame_info.width, CV_8UC3);
      }
        readBGR();
        break;
      case EImageFormat::MONO: {
        // Create a BGR matrix (assuming all channel have the same layer)
        m_bgr_mat = cv::Mat(m_frame_info.height, m_frame_info.width, CV_8UC3);
      }
        readMONO();
        break;
      case EImageFormat::NONE:
        std::cerr << "Unknown image format: " << m_frame_info.format << std::endl;
        return;
    }
  }

  // While receiving
  // FF frame_id(2bytes) row_id(2bytes)

  // For response
  // ACK = 0xFF and NACK = 0xFE
  // FF 00 00 00 00
  // 1st byte = ACK or NACK
  // 2nd to 3rd bytes = frameId
  // 4th to 5th bytes = rowId
}

void FrameReceiverServer::readYUV()
{
  const int buffer_size = m_frame_info.width + 5;
  std::uint8_t *buffer = new std::uint8_t[buffer_size];

  int n_rows = 0;
  while (n_rows < m_yuv_mat.rows) {
    int len = read(m_client_sockfd, buffer, buffer_size);
    if (len > 0) {
      // Copy buffer
      std::vector<uint8_t> data_buffer(buffer + 5, buffer + buffer_size + 5);
      // Clear buffer
      memset(buffer, 0, buffer_size);

      debugDataBytes("Reading line", data_buffer);

      std::lock_guard<std::mutex> lk(m_lock);
      // Get row data
      auto start_pos = n_rows * m_yuv_mat.cols;
      for (const auto &val : data_buffer) {
        m_yuv_mat.at<uchar>(start_pos++) = val;
      }
      ++n_rows;
    }
  }

  delete[] buffer;
  buffer = nullptr;

  std::cout << "Whole frame " << m_frame_info.frame_id << " parsed" << std::endl;
}

void FrameReceiverServer::readBGR()
{
  const int buffer_size = m_frame_info.width + 5;
  std::uint8_t *buffer = new std::uint8_t[buffer_size];

  for (int c = 0; c < m_bgr_mat.channels(); ++c) {
    int n_rows = 0;
    while (n_rows < m_bgr_mat.rows) {
      int len = read(m_client_sockfd, buffer, buffer_size);
      if (len > 0) {
        // Copy buffer
        std::vector<uint8_t> data_buffer(buffer + 5, buffer + buffer_size + 5);
        // Clear buffer
        memset(buffer, 0, buffer_size);

        debugDataBytes("Reading line", data_buffer);

        std::lock_guard<std::mutex> lk(m_lock);
        // Get row data
        auto start_pos = n_rows * m_bgr_mat.cols;
        for (const auto &val : data_buffer) {
          m_bgr_mat.at<cv::Vec3b>(start_pos++)[c] = val;
        }
        ++n_rows;
      }
    }
  }

  delete[] buffer;
  buffer = nullptr;

  std::cout << "Whole frame " << m_frame_info.frame_id << " parsed" << std::endl;
}

void FrameReceiverServer::readMONO()
{
  const int buffer_size = m_frame_info.width + 5;
  std::uint8_t *buffer = new std::uint8_t[buffer_size];

  int n_rows = 0;
  while (n_rows < m_bgr_mat.rows) {
    int len = read(m_client_sockfd, buffer, buffer_size);
    if (len > 0) {
      // Copy buffer
      std::vector<uint8_t> data_buffer(buffer + 5, buffer + buffer_size + 5);
      // Clear buffer
      memset(buffer, 0, buffer_size);

      debugDataBytes("Reading line", data_buffer);

      std::lock_guard<std::mutex> lk(m_lock);
      // Get row data
      auto start_pos = n_rows * m_bgr_mat.cols;
      for (const auto &val : data_buffer) {
        m_bgr_mat.at<cv::Vec3b>(start_pos++) = cv::Vec3b{val, val, val};
      }
      ++n_rows;
    }
  }

  delete[] buffer;
  buffer = nullptr;

  std::cout << "Whole frame " << m_frame_info.frame_id << " parsed" << std::endl;
}

void FrameReceiverServer::debugDataBytes(const std::string &prefix,
                                         const std::vector<uint8_t> &bytes)
{
#if DEBUG_DATA
  std::cout << prefix << " [";
  for (auto byte : bytes) {
    std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
  }
  std::cout << "]" << std::endl;
#endif
}

cv::Mat FrameReceiverServer::getFrame()
{
  std::lock_guard<std::mutex> lk(m_lock);

  switch (m_frame_info.format) {
    case EImageFormat::BGR_RAW:
    case EImageFormat::MONO:
    case EImageFormat::NONE:
      break;
    case EImageFormat::YUV:
      // Convert NV21 to BGR
      cv::cvtColor(m_yuv_mat, m_bgr_mat, cv::COLOR_YUV2BGR_NV21);
      break;
  }

  return m_bgr_mat;
}

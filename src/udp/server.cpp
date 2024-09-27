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

using namespace udp;

#define DEFAULT_BUFFER_SIZE 1024 // In Bytes
#define PAYLOAD_HEADER_SIZE 5    // In Bytes
#define MAX_PRIORITY 6           // For SO_PRIORITY, higher numbers represent higher priority
#define DEBUG_DATA false
constexpr int RECV_BUFFER_SIZE = 212992; // Taille du tampon de réception
constexpr uint8_t FRAME_BEGIN_CODE = 0xb8;

FrameReceiverServer::FrameReceiverServer(const std::string &server_address, int listening_port)
  : m_server_address(server_address),
    m_listening_port(listening_port),
    m_listening_sockfd(-1),
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
  struct sockaddr_in server_addr;

  // Créer un socket UDP
  m_listening_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (m_listening_sockfd < 0) {
    std::cerr << "Erreur lors de la création du socket." << std::endl;
    return false;
  }

  // Configurer l'adresse du serveur
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;       // Accepter les connexions de toutes les adresses
  server_addr.sin_port = htons(m_listening_port); // Port du serveur

  // Lier le socket à l'adresse et au port spécifiés
  if (bind(m_listening_sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "Erreur lors de la liaison du socket." << std::endl;
    close(m_listening_sockfd);
    return false;
  }

  // Set the maximum Type of Service (ToS) for high priority traffic (0x10 for minimum delay)
  int tos = 0x10; // TOS = 16 (low delay)
  if (setsockopt(m_listening_sockfd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) < 0) {
    std::cerr << "Error setting IP_TOS." << std::endl;
  }
  else {
    std::cout << "Set IP_TOS to 0x10 for low delay." << std::endl;
  }

  // Set the socket priority (SO_PRIORITY)
  int priority = MAX_PRIORITY; // Maximum priority for Linux (6 is max priority for SO_PRIORITY)
  if (setsockopt(m_listening_sockfd, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) < 0) {
    std::cerr << "Error setting SO_PRIORITY." << std::endl;
  }
  else {
    std::cout << "Set SO_PRIORITY to " << MAX_PRIORITY << " (maximum priority)." << std::endl;
  }

  // Configurer la taille du tampon de réception
  if (setsockopt(
          m_listening_sockfd, SOL_SOCKET, SO_RCVBUF, &RECV_BUFFER_SIZE, sizeof(RECV_BUFFER_SIZE)) <
      0) {
    std::cerr << "Erreur lors de la configuration du tampon de réception." << std::endl;
    close(m_listening_sockfd);
    return false;
  }

  std::cout << "Serveur UDP en écoute sur le port " << m_listening_port
            << " avec un tampon de réception de " << RECV_BUFFER_SIZE << " octets." << std::endl;

  return true;
}

void FrameReceiverServer::listen()
{
  reallocateBuffer(DEFAULT_BUFFER_SIZE);

  std::cout << "Looking for a frame header" << std::endl;
  // Try to find a frame header
  while (!readHeader()) {
  }

  std::cout << "Header found => Start streaming thread" << std::endl;
  m_listening_thread = std::thread([this]() {
    // Loop - listening data
    while (!m_close) {
      // Launch reception
      readData();
    }

    delete[] m_buffer;
    m_buffer = nullptr;
  });
}

void FrameReceiverServer::reallocateBuffer(std::size_t size)
{
  assert(size);

  delete[] m_buffer;
  m_buffer = new std::uint8_t[size];
  m_buff_size = size;
}

ssize_t FrameReceiverServer::buffering()
{
  if (m_buffer == nullptr) {
    std::cerr << "Empty buffer" << std::endl;
    return 0;
  }

  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);

  // Réception des paquets UDP
  ssize_t num_bytes_received = recvfrom(
      m_listening_sockfd, m_buffer, m_buff_size, 0, (struct sockaddr *) &client_addr, &addr_len);
  // TODO: client_addr & addr_len maybe useless

  if (num_bytes_received <= 0) {
    std::cerr << "Erreur lors de la réception des données." << std::endl;
  }
  /*
  else {
    std::cout << "Reçu " << num_bytes_received << " octets de " << inet_ntoa(client_addr.sin_addr)
              << ":" << ntohs(client_addr.sin_port) << std::endl;
  }*/

  return num_bytes_received;
}

auto FrameReceiverServer::readHeader() -> bool
{
  if (buffering() <= 0) {
    return false;
  }

#if DEBUG_DATA
  debugDataBytes("Header data", header_buffer);
#endif
  // Copy buffer
  std::vector<std::uint8_t> header_buffer(m_buffer, m_buffer + m_buff_size / sizeof(std::uint8_t));

  // Example:
  // b8 01a6 0280 01e0 0023 08fc
  EImageFormat frame_format = EImageFormat::BGR_RAW;

  std::size_t i = 0;
  // start of frame 0xb8
  std::uint8_t code = header_buffer[i++];
  if (code == FRAME_BEGIN_CODE) {
    // Deduce frame format encoding
    uint16_t frame_encoding = header_buffer[i++] << 8;
    frame_encoding += header_buffer[i++];
    if (frame_encoding == EImageFormat::YUV) {
      frame_format = EImageFormat::YUV;
      // std::cerr << "YUV header format detected: " << frame_encoding << std::endl;
    }
    else if (frame_encoding == EImageFormat::BGR_RAW) {
      frame_format = EImageFormat::BGR_RAW;
      // std::cerr << "BGR header format detected: " << frame_encoding << std::endl;
    }
    else if (frame_encoding == EImageFormat::MONO) {
      frame_format = EImageFormat::MONO;
      // std::cerr << "MONO header format detected: " << frame_encoding << std::endl;
    }
    else {
      std::cerr << "Uknown header format detected: " << frame_encoding << std::endl;
      return false;
    }

    uint16_t width = 0;
    uint16_t height = 0;
    uint16_t frame_id = 0;
    uint16_t ecc = 0;

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
      /* std::cout << "Header of frame " << frame_id << " validation successfull: " << ecc <<
       * std::endl; */
      m_frame_info = FrameStructure{width, height, frame_id, frame_format};
      return true;
    }
    else {
      std::cerr << "Failed to validate ECC: " << ecc << std::endl;
    }
  }

  return false;
}

void FrameReceiverServer::readData()
{
  reallocateBuffer(m_frame_info.width + PAYLOAD_HEADER_SIZE);

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
  // While we are reading payload frame data
  std::uint16_t n_prev_row = 0;
  while (!readHeader()) {
    // Get rows data

    // Copy buffer
    std::vector<uint8_t> data_buffer(m_buffer + PAYLOAD_HEADER_SIZE,
                                     m_buffer + m_buff_size + PAYLOAD_HEADER_SIZE);

    debugDataBytes("Reading line", data_buffer);

    std::uint16_t frame_id = m_buffer[1] << 8;
    frame_id += m_buffer[2];

    std::uint16_t n_row = m_buffer[3] << 8;
    n_row += m_buffer[4];

    if (m_frame_info.frame_id != frame_id) {
      std::cout << "Frame was " << m_frame_info.frame_id << ". But the current one is " << frame_id
                << std::endl;
      std::cout << "Previous row was " << n_prev_row << " and the current one is " << n_row
                << std::endl;

      m_frame_info.frame_id = frame_id;
      break;
    }
    n_prev_row = n_row;

    std::lock_guard<std::mutex> lk(m_lock);
    // Get row data
    auto start_pos = n_row * m_yuv_mat.cols;
    for (std::size_t pos = 0; pos < data_buffer.size(); ++pos) {
      m_yuv_mat.at<uchar>(start_pos + pos) = data_buffer[pos];
    }

    // Clear buffer
    memset(m_buffer, 0, m_buff_size);
  }

  // std::cout << "Whole frame " << m_frame_info.frame_id << " parsed" << std::endl;
}

void FrameReceiverServer::readBGR()
{
  const int buffer_size = m_frame_info.width + PAYLOAD_HEADER_SIZE;
  std::uint8_t *buffer = new std::uint8_t[buffer_size];

  for (int c = 0; c < m_bgr_mat.channels(); ++c) {
    int n_rows = 0;
    while (n_rows < m_bgr_mat.rows) {
      int len = read(m_listening_sockfd, buffer, buffer_size);
      if (len > 0) {
        // Copy buffer
        std::vector<uint8_t> data_buffer(buffer + PAYLOAD_HEADER_SIZE,
                                         buffer + buffer_size + PAYLOAD_HEADER_SIZE);
        // Clear buffer
        memset(buffer, 0, buffer_size);

        debugDataBytes("Reading line", data_buffer);

        std::lock_guard<std::mutex> lk(m_lock);
        // Get row data
        auto start_pos = n_rows * m_bgr_mat.cols;
        for (std::size_t pos = 0; pos < len; ++pos) {
          m_bgr_mat.at<cv::Vec3b>(start_pos + pos)[c] = data_buffer[pos];
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
  const int buffer_size = m_frame_info.width + PAYLOAD_HEADER_SIZE;
  std::uint8_t *buffer = new std::uint8_t[buffer_size];

  int n_rows = 0;
  while (n_rows < m_bgr_mat.rows) {
    int len = read(m_listening_sockfd, buffer, buffer_size);
    if (len > 0) {
      // Copy buffer
      std::vector<uint8_t> data_buffer(buffer + PAYLOAD_HEADER_SIZE,
                                       buffer + buffer_size + PAYLOAD_HEADER_SIZE);
      // Clear buffer
      memset(buffer, 0, buffer_size);

      debugDataBytes("Reading line", data_buffer);

      std::lock_guard<std::mutex> lk(m_lock);
      // Get row data
      auto start_pos = n_rows * m_bgr_mat.cols;
      for (std::size_t pos = 0; pos < len; ++pos) {
        const auto val = data_buffer[pos + PAYLOAD_HEADER_SIZE];
        m_bgr_mat.at<cv::Vec3b>(start_pos + pos) = cv::Vec3b{val, val, val};
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

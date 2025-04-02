#include "streaming_udp.h"

#include "../decoder/avdecoder.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <iostream>
#include <thread>
#include <unistd.h>

#define MAX_PRIORITY 6                  // For SO_PRIORITY, higher numbers represent higher priority
#define RECV_TOS_PRIORITY_TRAFFIC 0x10; // TOS = 16 (low delay)
#define RECV_BUFFER_SIZE 212992;        // Taille du tampon de réception

#define FRAME_BUFFER_MTU 1280 // Max size of UDP packet
#define HEADER_BUFFER_SIZE 32

#define ACCELEROMETER_BUFFER_SIZE 64 // Max size of UDP packet

#define DEBUG_DATA 0

StreamingUdp::StreamingUdp() {}

auto StreamingUdp::stream() -> void
{
  int sockfd;
  struct sockaddr_in serverAddr, clientAddr;
  socklen_t addrLen = sizeof(clientAddr);

  char buffer[FRAME_BUFFER_MTU];
  memset(buffer, 0, FRAME_BUFFER_MTU);

  // Initialize FFmpeg network components
  avformat_network_init();

  // Create UDP socket
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("socket creation failed");
    return;
  }

  // Bind the socket to an IP address and port
  memset(&serverAddr, 0, sizeof(serverAddr));
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = INADDR_ANY;
  serverAddr.sin_port = htons(m_listen_port);

  if (bind(sockfd, (const struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
    perror("bind failed");
    close(sockfd);
    return;
  }

  // Set the maximum Type of Service (ToS) for high priority traffic (0x10 for minimum delay)
  int ToS = RECV_TOS_PRIORITY_TRAFFIC;
  if (setsockopt(sockfd, IPPROTO_IP, IP_TOS, &ToS, sizeof(ToS)) < 0) {
    std::cerr << "Error setting IP_TOS." << std::endl;
  }
  else {
    std::cout << "Set IP_TOS to " << ToS << " for low delay." << std::endl;
  }

  // Set the socket priority (SO_PRIORITY)
  int priority = MAX_PRIORITY; // Maximum priority for Linux (6 is max priority for SO_PRIORITY)
  if (setsockopt(sockfd, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) < 0) {
    std::cerr << "Error setting SO_PRIORITY." << std::endl;
  }
  else {
    std::cout << "Set SO_PRIORITY to " << MAX_PRIORITY << " (maximum priority)." << std::endl;
  }

  // Configurer la taille du tampon de réception
  int buffer_size = RECV_BUFFER_SIZE;
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0) {
    std::cerr << "Erreur lors de la configuration du tampon de réception." << std::endl;
    close(sockfd);
    return;
  }

  // Init accelero data
  double linearAccX, linearAccY, linearAccZ;

  // Find the H.264 codec
  AVDecoder *decoder_ptr = nullptr;
  try {
    decoder_ptr = new AVDecoder();
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    close(sockfd);
    return;
  }

  std::cout << "Listening on port " << m_listen_port << " for UDP packets...\n";

  std::cerr << "Looking for header..." << std::endl;
  bool header_ok = false;
  ssize_t receive_len;
  FrameHeader fh;
  do {
    while (!fh.isValidEcc()) {
      receive_len = recvfrom(
          sockfd, buffer, HEADER_BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrLen);
      if (receive_len < 0) {
        perror("recvfrom failed");
        break;
      }
      readHeader(buffer, fh);
    }
    fh.unvalidate();

    // I got the header so I try to read SPS & PPS

    // SPS part
    receive_len =
        recvfrom(sockfd, buffer, HEADER_BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrLen);
    if (receive_len < 0) {
      perror("recvfrom failed");
      continue;
    }
    std::vector<uint8_t> sps_copy_buffer(buffer, buffer + receive_len);
    memset(buffer, 0, HEADER_BUFFER_SIZE);

    // PPS part
    receive_len =
        recvfrom(sockfd, buffer, HEADER_BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrLen);
    if (receive_len < 0) {
      perror("recvfrom failed");
      continue;
    }
    std::vector<uint8_t> pps_copy_buffer(buffer, buffer + receive_len);
    memset(buffer, 0, HEADER_BUFFER_SIZE);

    // Decoding SPS & PPS
    try {
      decoder_ptr->readSPSandPPS(sps_copy_buffer, pps_copy_buffer);
    }
    catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
      continue;
    }

    // I got the header & SPS & PPS so I try to read accelerometer data
    receive_len = recvfrom(
        sockfd, buffer, ACCELEROMETER_BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrLen);
    if (receive_len < 0) {
      perror("recvfrom failed");
      continue;
    }
    std::vector<uint8_t> tmp_accelero_buffer(buffer, buffer + receive_len);
    memset(buffer, 0, HEADER_BUFFER_SIZE);
    if (!readAccelerometerData(linearAccX, linearAccY, linearAccZ, tmp_accelero_buffer)) {
      std::cerr << "Erreur lors de la reception des données gyroscopique" << std::endl;
      continue;
    }

    // Get the full frame until the next header
    std::vector<uint8_t> data_buffer;
    ssize_t total = 0;
    do {
      receive_len =
          recvfrom(sockfd, buffer, FRAME_BUFFER_MTU, 0, (struct sockaddr *) &clientAddr, &addrLen);
      if (receive_len < 0) {
        perror("recvfrom failed");
        break;
      }
      std::vector<uint8_t> tmp_buffer(buffer, buffer + receive_len);
      data_buffer.insert(data_buffer.end(), tmp_buffer.begin(), tmp_buffer.end());
      total += receive_len;
    } while (total < fh.encoded_total_size);

    // std::cout << "Total receive " << total << "/" << fh.encoded_total_size << std::endl;

    debugDataBytes("New Frame", data_buffer);

    bool frame_received{false};
    try {
      frame_received = decoder_ptr->receiveFrameBuffer(data_buffer);
    }
    catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
    }

    if (frame_received) {
      // The is only 1 plane on BGR frame
      std::vector<std::uint8_t *> dest(1, nullptr);
      std::vector<int> linesize(1, 0);

      m_img = cv::Mat(fh.height, fh.width, CV_8UC3);

      dest[0] = m_img.data;
      linesize[0] = m_img.step1();

      decoder_ptr->scaleFrame(dest, linesize);

      cv::putText(m_img,
                  std::string("Gravity: " + std::to_string(linearAccX) +
                              " - Roll: " + std::to_string(linearAccY) +
                              " - Pitch: " + std::to_string(linearAccZ)),
                  cv::Point2i(30, 30),
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.5,
                  cv::Scalar(0, 255, 0),
                  1);
    }
  } while (true);

  // Cleanup
  if (decoder_ptr) {
    decoder_ptr->cleanup();
    delete decoder_ptr;
    decoder_ptr = nullptr;
  }

  close(sockfd);
}

auto StreamingUdp::readHeader(char *buffer, FrameHeader &fh) -> void
{
  // Copy buffer
  std::vector<std::uint8_t> header_buffer(buffer, buffer + HEADER_BUFFER_SIZE / sizeof(char));

  // Example:
  // [begin_code] [encoding] [width] [height] [frame_id] [encoded_size] [ecc]
  // b8           0080       0280    01e0     0023       XXXXXXXX       XX

  std::size_t i = 0;
  std::uint8_t code = header_buffer[i++];

  // begin of frame code is 0xb8
  if (code == fh.beginCode()) {
    // Deduce frame format encoding

    // get frame encoding
    fh.frame_encoding = header_buffer[i++] << 8;
    fh.frame_encoding += header_buffer[i++];

    // get width
    fh.width = header_buffer[i++] << 8;
    fh.width += header_buffer[i++];

    // get height
    fh.height = header_buffer[i++] << 8;
    fh.height += header_buffer[i++];

    // get frame id
    fh.frame_id = header_buffer[i++] << 8;
    fh.frame_id += header_buffer[i++];

    // get encoded frame total size
    fh.encoded_total_size = header_buffer[i++] << 24;
    fh.encoded_total_size += header_buffer[i++] << 16;
    fh.encoded_total_size += header_buffer[i++] << 8;
    fh.encoded_total_size += header_buffer[i++];

    // get validation ecc code
    std::uint8_t ecc = header_buffer[i++];

    fh.validateECC(ecc);
  }
}

auto StreamingUdp::vectorToDouble(std::vector<uint8_t> byte_vector) -> std::pair<bool, double>
{
  std::reverse(byte_vector.begin(), byte_vector.end());

  double value = 0.;
  // Copy the bytes into the double variable
  if (byte_vector.size() == sizeof(double)) {
    std::memcpy(&value, byte_vector.data(), sizeof(double));
  }
  else if (byte_vector.size() == sizeof(float)) {
    float value_f = 0.F;
    std::memcpy(&value_f, byte_vector.data(), sizeof(float));
    value = static_cast<double>(value_f);
  }
  else {
    std::cout << "byteVector size (" << std::to_string(byte_vector.size())
              << ") does not match with double (" << std::to_string(sizeof(double)) << ")or float ("
              << std::to_string(sizeof(float)) + ")" << std::endl;
    return std::make_pair(false, value);
  }

  return std::make_pair(true, value);
}

auto StreamingUdp::readAccelerometerData(double &linearAccelerationX,
                                         double &linearAccelerationY,
                                         double &linearAccelerationZ,
                                         const std::vector<uint8_t> &accelero_data) -> bool
{
  if (accelero_data.empty()) {
    return false;
  }

  auto unit_size = accelero_data.size() / 3;
  std::vector<uint8_t> x_acceleration_buffer(accelero_data.begin(),
                                             accelero_data.begin() + unit_size);
  std::vector<uint8_t> y_acceleration_buffer(accelero_data.begin() + unit_size,
                                             accelero_data.begin() + 2 * unit_size);
  std::vector<uint8_t> z_acceleration_buffer(accelero_data.begin() + 2 * unit_size,
                                             accelero_data.end());

  auto x = vectorToDouble(x_acceleration_buffer);
  auto y = vectorToDouble(y_acceleration_buffer);
  auto z = vectorToDouble(z_acceleration_buffer);
  if (x.first && y.first && z.first) {
    linearAccelerationX = x.second;
    linearAccelerationY = y.second;
    linearAccelerationZ = z.second;
    return true;
  }

  return false;
}

auto StreamingUdp::setListenPort(std::uint16_t listen_port) -> void
{
  m_listen_port = listen_port;
}

auto StreamingUdp::img() const -> const cv::Mat &
{
  return m_img;
}

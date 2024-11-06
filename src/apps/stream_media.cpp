#include "../decoder/avdecoder.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>

#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>

#define MAX_PRIORITY 6                  // For SO_PRIORITY, higher numbers represent higher priority
#define RECV_TOS_PRIORITY_TRAFFIC 0x10; // TOS = 16 (low delay)
#define RECV_BUFFER_SIZE 212992;        // Taille du tampon de réception
#define FRAME_PORT 50000
#define FRAME_BUFFER_MTU 1280 // Max size of UDP packet
#define HEADER_BUFFER_SIZE 32

#define ACCELEROMETER_BUFFER_SIZE 64 // Max size of UDP packet

#define DEBUG_DATA 0

struct FrameHeader {
 private:
  bool is_valid = false;
  std::uint8_t code = 0xb8;

 public:
  std::uint16_t frame_encoding = 0;
  std::uint16_t width = 0;
  std::uint16_t height = 0;
  std::uint16_t frame_id = 0;
  std::uint32_t encoded_total_size = 0;

 public:
  void unvalidate() { is_valid = false; }
  void validateECC(const uint8_t ecc_to_test)
  {
    int iframe_encoding = static_cast<int>(frame_encoding);
    int iwidth = static_cast<int>(width);
    int iheight = static_cast<int>(height);
    int iframe_id = static_cast<int>(frame_id);
    int iencoded_total_size = static_cast<int>(encoded_total_size);

    uint8_t ecc = code; // Start with the least significant byte of 'code'

    // XOR each byte of the fields to accumulate parity
    ecc ^= static_cast<uint8_t>(iframe_encoding & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_encoding >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_encoding >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_encoding >> 24) & 0xFF);

    ecc ^= static_cast<uint8_t>(iwidth & 0xFF);
    ecc ^= static_cast<uint8_t>((iwidth >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iwidth >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iwidth >> 24) & 0xFF);

    ecc ^= static_cast<uint8_t>(iheight & 0xFF);
    ecc ^= static_cast<uint8_t>((iheight >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iheight >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iheight >> 24) & 0xFF);

    ecc ^= static_cast<uint8_t>(iframe_id & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_id >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_id >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iframe_id >> 24) & 0xFF);

    ecc ^= static_cast<uint8_t>(iencoded_total_size & 0xFF);
    ecc ^= static_cast<uint8_t>((iencoded_total_size >> 8) & 0xFF);
    ecc ^= static_cast<uint8_t>((iencoded_total_size >> 16) & 0xFF);
    ecc ^= static_cast<uint8_t>((iencoded_total_size >> 24) & 0xFF);

    is_valid = true; // = (ecc_to_test == ecc);
  }

 public:
  inline bool isValidEcc() const { return is_valid; }
  inline std::uint8_t beginCode() const { return code; }
};

auto readHeader(char *buffer, FrameHeader &fh) -> void
{
  // Copy buffer
  std::vector<std::uint8_t> header_buffer(buffer, buffer + HEADER_BUFFER_SIZE / sizeof(char));

  // Example:
  // [begin_code] [encoding] [width] [height] [frame_id] [ecc]
  // b8           0080       0280    01e0     0023       XXXX

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

void debugDataBytes(const std::string &prefix, const std::vector<uint8_t> &bytes)
{
#if DEBUG_DATA
  std::cout << prefix << " [";
  for (auto byte : bytes) {
    std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
  }
  std::cout << "]" << std::endl;
#endif
}

double vectorToDouble(std::vector<uint8_t> byteVector)
{
  std::reverse(byteVector.begin(), byteVector.end());

  double value = 0.;
  // Copy the bytes into the double variable
  if (byteVector.size() == sizeof(double)) {
    std::memcpy(&value, byteVector.data(), sizeof(double));
  }
  else if (byteVector.size() == sizeof(float)) {
    float value_f = 0.F;
    std::memcpy(&value_f, byteVector.data(), sizeof(float));
    value = static_cast<double>(value_f);
  }
  else {
    std::cout << "byteVector size (" << std::to_string(byteVector.size())
              << ") does not match with double (" << std::to_string(sizeof(double)) << ")or float ("
              << std::to_string(sizeof(float)) + ")" << std::endl;
  }

  return value;
}

void readAccelerometerData(double &linearAccelerationX,
                           double &linearAccelerationY,
                           double &linearAccelerationZ,
                           const std::vector<uint8_t> &accelero_data)
{
  if (accelero_data.empty()) {
    return;
  }

  auto unit_size = accelero_data.size() / 3;
  std::vector<uint8_t> x_acceleration_buffer(accelero_data.begin(),
                                             accelero_data.begin() + unit_size);
  std::vector<uint8_t> y_acceleration_buffer(accelero_data.begin() + unit_size,
                                             accelero_data.begin() + 2 * unit_size);
  std::vector<uint8_t> z_acceleration_buffer(accelero_data.begin() + 2 * unit_size,
                                             accelero_data.end());

  linearAccelerationX = vectorToDouble(x_acceleration_buffer);
  linearAccelerationY = vectorToDouble(y_acceleration_buffer);
  linearAccelerationZ = vectorToDouble(z_acceleration_buffer);
}

int main()
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
    return -1;
  }

  // Bind the socket to an IP address and port
  memset(&serverAddr, 0, sizeof(serverAddr));
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = INADDR_ANY;
  serverAddr.sin_port = htons(FRAME_PORT);

  if (bind(sockfd, (const struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
    perror("bind failed");
    close(sockfd);
    return -1;
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
    return -1;
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
    return -1;
  }

  std::cout << "Listening on port " << FRAME_PORT << " for UDP packets...\n";

  // OpenCV window to display the frames
  cv::namedWindow("H264 Stream", cv::WINDOW_AUTOSIZE);

  std::cerr << "Looking for header..." << std::endl;
  bool header_ok = false;
  ssize_t receive_len;
  FrameHeader fh;
  do {
    while (fh.isValidEcc()) {
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
      if (!decoder_ptr->readSPSandPPS(sps_copy_buffer, pps_copy_buffer)) {
        std::cerr << "Erreur lors de l'envoi du paquet SPS ou PPS au décodeur" << std::endl;
      }
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
    readAccelerometerData(linearAccX, linearAccY, linearAccZ, tmp_accelero_buffer);

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

    debugDataBytes("New Frame", data_buffer);

    bool frame_received{false};
    try {
      frame_received = decoder_ptr->receiveFrameBuffer(data_buffer);
    }
    catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
    }

    if (frame_received) {
      auto frame_ptr = decoder_ptr->frame();

      // The is only 1 plane on BGR frame
      std::vector<std::uint8_t *> dest(1, nullptr);
      std::vector<int> linesize(1, 0);

      cv::Mat img(frame_ptr->height, frame_ptr->width, CV_8UC3);

      dest[0] = img.data;
      linesize[0] = img.step1();

      decoder_ptr->scaleFrame(dest, linesize);

      cv::putText(img,
                  std::string("Gravity: " + std::to_string(linearAccX) +
                              " - Roll: " + std::to_string(linearAccY) +
                              " - Pitch: " + std::to_string(linearAccZ)),
                  cv::Point2i(30, 30),
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.5,
                  cv::Scalar(0, 255, 0),
                  1);

      // Display the frame using OpenCV
      cv::imshow("H264 Stream", img);
      if (cv::waitKey(1) == 27)
        break; // Exit on 'ESC' key
    }
  } while (true);

  // Cleanup
  decoder_ptr->cleanup();

  close(sockfd);
  cv::destroyWindow("H264 Stream");

  return 0;
}

#include <arpa/inet.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>

#include <iostream>
#include <thread>
#include <unistd.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#define MAX_PRIORITY 6                  // For SO_PRIORITY, higher numbers represent higher priority
#define RECV_TOS_PRIORITY_TRAFFIC 0x10; // TOS = 16 (low delay)
#define RECV_BUFFER_SIZE 212992;        // Taille du tampon de réception
#define FRAME_PORT 50000
#define FRAME_BUFFER_MTU 1200 // Max size of UDP packet
#define HEADER_BUFFER_SIZE 32

#define ACCELEROMETER_PORT 50002
#define ACCELEROMETER_BUFFER_MTU 64 // Max size of UDP packet

#define DEBUG_DATA 0

bool g_sps_and_pps_read{false};

void readSPSandPPS(AVCodecContext *codecContext, std::vector<uint8_t> sps, std::vector<uint8_t> pps)
{
  if (g_sps_and_pps_read) {
    return;
  }

  // Encapsuler SPS dans un AVPacket
  AVPacket *spsPacket = av_packet_alloc();
  spsPacket->data = sps.data();
  spsPacket->size = sps.size();

  // Envoyer le SPS au décodeur
  if (avcodec_send_packet(codecContext, spsPacket) < 0) {
    std::cerr << "Erreur lors de l'envoi du paquet SPS au décodeur" << std::endl;
  }
  av_packet_free(&spsPacket);

  // Encapsuler PPS dans un AVPacket
  AVPacket *ppsPacket = av_packet_alloc();
  ppsPacket->data = pps.data();
  ppsPacket->size = pps.size();

  // Envoyer le PPS au décodeur
  if (avcodec_send_packet(codecContext, ppsPacket) < 0) {
    std::cerr << "Erreur lors de l'envoi du paquet PPS au décodeur" << std::endl;
  }
  av_packet_free(&ppsPacket);

  g_sps_and_pps_read = true;
}

auto readHeader(char *buffer) -> bool
{
  // Copy buffer
  std::vector<std::uint8_t> header_buffer(buffer, buffer + HEADER_BUFFER_SIZE / sizeof(char));

  // Example:
  // [begin_code] [encoding] [width] [height] [frame_id] [ecc]
  // b8           0080       0280    01e0     0023       XXXX

  std::size_t i = 0;
  std::uint8_t code = header_buffer[i++];

  // begin of frame code is 0xb8
  if (code == 0xb8) {
    // Deduce frame format encoding
    uint16_t frame_encoding = 0;
    uint16_t width = 0;
    uint16_t height = 0;
    uint16_t frame_id = 0;
    uint16_t ecc = 0;

    // get frame encoding
    frame_encoding = header_buffer[i++] << 8;
    frame_encoding += header_buffer[i++];

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
      /*
      std::cout << "Header of frame " << frame_id << " validation successfull: " << ecc
                << std::endl;
      */
      return true;
    }
    else {
      std::cerr << "Failed to validate ECC: " << ecc << std::endl;
    }
  }

  return false;
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
    throw std::out_of_range("byteVector size (" + std::to_string(byteVector.size()) +
                            ") does not match with double (" + std::to_string(sizeof(double)) +
                            ")or float (" + std::to_string(sizeof(float)) + ")");
  }

  return value;
}

void receiveAccelerometerData(double &linearAccelerationX,
                              double &linearAccelerationY,
                              double &linearAccelerationZ)
{
  int sockfd;
  struct sockaddr_in serverAddr, clientAddr;
  socklen_t addrLen = sizeof(clientAddr);

  char buffer[ACCELEROMETER_BUFFER_MTU];
  memset(buffer, 0, ACCELEROMETER_BUFFER_MTU);

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
  serverAddr.sin_port = htons(ACCELEROMETER_PORT);

  if (bind(sockfd, (const struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
    perror("bind failed");
    close(sockfd);
    return;
  }

  while (true) {
    ssize_t recvLen = recvfrom(
        sockfd, buffer, ACCELEROMETER_BUFFER_MTU, 0, (struct sockaddr *) &clientAddr, &addrLen);
    if (recvLen < 0) {
      perror("recvfrom failed");
      continue;
    }
    std::vector<uint8_t> tmp_accelerometer_buffer(buffer, buffer + recvLen);
    memset(buffer, 0, ACCELEROMETER_BUFFER_MTU);

    auto unit_size = recvLen / 3;
    std::vector<uint8_t> x_acceleration_buffer(tmp_accelerometer_buffer.begin(),
                                               tmp_accelerometer_buffer.begin() + unit_size);
    std::vector<uint8_t> y_acceleration_buffer(tmp_accelerometer_buffer.begin() + unit_size,
                                               tmp_accelerometer_buffer.begin() + 2 * unit_size);
    std::vector<uint8_t> z_acceleration_buffer(tmp_accelerometer_buffer.begin() + 2 * unit_size,
                                               tmp_accelerometer_buffer.end());

    linearAccelerationX = vectorToDouble(x_acceleration_buffer);
    linearAccelerationY = vectorToDouble(y_acceleration_buffer);
    linearAccelerationZ = vectorToDouble(z_acceleration_buffer);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
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

  double linearAccX, linearAccY, linearAccZ;
  std::thread accThread([&linearAccX, &linearAccY, &linearAccZ] {
    receiveAccelerometerData(linearAccX, linearAccY, linearAccZ);
  });

  // Find the H.264 codec
  AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) {
    std::cerr << "Codec not found!" << std::endl;
    close(sockfd);
    return -1;
  }

  // Allocate codec context
  AVCodecContext *codecContext = avcodec_alloc_context3(codec);
  if (!codecContext) {
    std::cerr << "Could not allocate codec context!" << std::endl;
    close(sockfd);
    return -1;
  }

  // Open the codec
  if (avcodec_open2(codecContext, codec, nullptr) < 0) {
    std::cerr << "Could not open codec!" << std::endl;
    avcodec_free_context(&codecContext);
    close(sockfd);
    return -1;
  }

  // Allocate an AVFrame for decoding
  AVFrame *frame = av_frame_alloc();
  if (!frame) {
    std::cerr << "Could not allocate frame!" << std::endl;
    avcodec_free_context(&codecContext);
    close(sockfd);
    return -1;
  }

  std::cout << "Listening on port " << FRAME_PORT << " for UDP packets...\n";

  // OpenCV window to display the frames
  cv::namedWindow("H264 Stream", cv::WINDOW_AUTOSIZE);

  std::cerr << "Looking for header..." << std::endl;
  bool header_ok = false;
  do {
    while (!header_ok) {
      ssize_t recvLen = recvfrom(
          sockfd, buffer, HEADER_BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrLen);
      if (recvLen < 0) {
        perror("recvfrom failed");
        break;
      }
      header_ok = readHeader(buffer);
    }

    // I got the header so I try to read SPS & PPS
    ssize_t recvLen =
        recvfrom(sockfd, buffer, HEADER_BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrLen);
    if (recvLen < 0) {
      perror("recvfrom failed");
      continue;
    }
    std::vector<uint8_t> tmp_sps_buffer(buffer, buffer + recvLen);
    memset(buffer, 0, HEADER_BUFFER_SIZE);

    recvLen =
        recvfrom(sockfd, buffer, HEADER_BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrLen);
    if (recvLen < 0) {
      perror("recvfrom failed");
      continue;
    }
    std::vector<uint8_t> tmp_pps_buffer(buffer, buffer + recvLen);
    memset(buffer, 0, HEADER_BUFFER_SIZE);

    readSPSandPPS(codecContext, tmp_sps_buffer, tmp_pps_buffer);

    // Get the full frame until the next header
    // std::cerr << "Getting h264 frame data..." << std::endl;
    std::vector<uint8_t> data_buffer;
    do {
      ssize_t recvLen =
          recvfrom(sockfd, buffer, FRAME_BUFFER_MTU, 0, (struct sockaddr *) &clientAddr, &addrLen);
      if (recvLen < 0) {
        perror("recvfrom failed");
        break;
      }
      if (readHeader(buffer)) {
        break;
      }
      else {
        std::vector<uint8_t> tmp_buffer(buffer, buffer + recvLen);
        data_buffer.insert(data_buffer.end(), tmp_buffer.begin(), tmp_buffer.end());
      }
    } while (true);

    debugDataBytes("New Frame", data_buffer);

    // std::cerr << "End of frame data..." << std::endl;

    // Create an AVPacket and initialize it
    AVPacket *packet = av_packet_alloc();
    if (!packet) {
      std::cerr << "Could not allocate AVPacket!" << std::endl;
      break;
    }

    // Set data and size for the packet
    packet->data = data_buffer.data();
    packet->size = data_buffer.size();

    // Send the packet to the decoder
    if (avcodec_send_packet(codecContext, packet) < 0) {
      std::cerr << "Error sending packet for decoding\n";
      av_packet_free(&packet); // Free packet if send failed
      continue;
    }

    // Receive the decoded frame
    if (avcodec_receive_frame(codecContext, frame) == 0) {
      // std::cout << "avcodec_receive_frame: " << frame->width << "x" << frame->height <<
      // std::endl; Convert the decoded frame to a format suitable for OpenCV (BGR24)
      SwsContext *swsCtx = sws_getContext(frame->width,
                                          frame->height,
                                          codecContext->pix_fmt,
                                          frame->width,
                                          frame->height,
                                          AV_PIX_FMT_BGR24,
                                          SWS_BILINEAR,
                                          nullptr,
                                          nullptr,
                                          nullptr);

      uint8_t *dest[4] = {nullptr};
      int linesize[4] = {0};
      cv::Mat img(frame->height, frame->width, CV_8UC3);
      dest[0] = img.data;
      linesize[0] = img.step1();

      sws_scale(swsCtx, frame->data, frame->linesize, 0, frame->height, dest, linesize);
      sws_freeContext(swsCtx);

      cv::putText(img,
                  std::string("X: " + std::to_string(linearAccX) + " - Y: " +
                              std::to_string(linearAccY) + " - Z: " + std::to_string(linearAccZ)),
                  cv::Point2i(30, 30),
                  cv::FONT_HERSHEY_SIMPLEX,
                  1,
                  cv::Scalar(0, 255, 0),
                  2);

      // Display the frame using OpenCV
      cv::imshow("H264 Stream", img);
      if (cv::waitKey(1) == 27)
        break; // Exit on 'ESC' key
    }

    // Free the AVPacket
    av_packet_free(&packet);

  } while (true);

  // Cleanup
  av_frame_free(&frame);
  avcodec_free_context(&codecContext);
  close(sockfd);
  cv::destroyWindow("H264 Stream");

  return 0;
}

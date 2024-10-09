#include <arpa/inet.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>

#include <iostream>
#include <unistd.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#define PORT 50000
#define BUFFER_SIZE 1200 // Max size of UDP packet

auto readHeader(char *buffer) -> bool
{
  // Copy buffer
  std::vector<std::uint8_t> header_buffer(buffer, buffer + BUFFER_SIZE / sizeof(char));

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
      std::cout << "Header of frame " << frame_id << " validation successfull: " << ecc
                << std::endl;
      return true;
    }
    else {
      std::cerr << "Failed to validate ECC: " << ecc << std::endl;
    }
  }

  return false;
}

int main()
{
  int sockfd;
  struct sockaddr_in serverAddr, clientAddr;
  char buffer[BUFFER_SIZE];
  socklen_t addrLen = sizeof(clientAddr);

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
  serverAddr.sin_port = htons(PORT);

  if (bind(sockfd, (const struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
    perror("bind failed");
    close(sockfd);
    return -1;
  }

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

  std::cout << "Listening on port " << PORT << " for UDP packets...\n";

  // OpenCV window to display the frames
  cv::namedWindow("H264 Stream", cv::WINDOW_AUTOSIZE);

  do {
    std::cerr << "Looking for header..." << std::endl;
    bool header_ok = false;
    do {
      ssize_t recvLen =
          recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrLen);
      if (recvLen < 0) {
        perror("recvfrom failed");
        break;
      }
      header_ok = readHeader(buffer);
    } while (!header_ok);

    // Get the full frame until the next header
    std::cerr << "Getting h264 frame data..." << std::endl;
    std::vector<uint8_t> data_buffer;
    do {
      ssize_t recvLen =
          recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr *) &clientAddr, &addrLen);
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

    std::cerr << "End of frame data..." << std::endl;

    // Create an AVPacket and initialize it
    AVPacket *packet = av_packet_alloc();
    if (!packet) {
      std::cerr << "Could not allocate AVPacket!" << std::endl;
      break;
    }

    // Set data and size for the packet
    packet->data = &data_buffer[0];
    packet->size = data_buffer.size();

    // Send the packet to the decoder
    avcodec_send_packet(codecContext, packet);
    if (avcodec_send_packet(codecContext, packet) < 0) {
      std::cerr << "Error sending packet for decoding\n";
      av_packet_free(&packet); // Free packet if send failed
      continue;
    }

    // Receive the decoded frame
    if (avcodec_receive_frame(codecContext, frame) == 0) {
      std::cout << "avcodec_receive_frame: " << frame->width << "x" << frame->height << std::endl;
      // Convert the decoded frame to a format suitable for OpenCV (BGR24)
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

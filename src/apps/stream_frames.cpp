#include "../tcp/server.hpp"
#include "../udp/server.hpp"

#include <arpa/inet.h>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>

#include <chrono> // Pour std::chrono::seconds
#include <cstring>
#include <fstream>
#include <iostream>
#include <thread> // Pour std::this_thread::sleep_for
#include <unistd.h>

// sudo apt-get install qttools5-dev-tools

int main(int argc, char *argv[])
{
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <IP> <Port> <Output file>" << std::endl;
    return 1;
  }

  std::string address = argv[1];
  int port = std::stoi(argv[2]);
#if 1
  udp::FrameReceiverServer frame_listener(address, port);

  std::cout << "Waiting a client connection" << std::endl;

  while (!frame_listener.connection()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    std::cout << "Waiting for new connection" << std::endl;
  }

  std::cout << "Server and client are connected" << std::endl;

  frame_listener.listen();

  std::thread preview = std::thread([&frame_listener]() {
    std::cerr << "Pending preview thread" << std::endl;
    int key = 0;
    while (key != 27) {
      cv::imshow("Preview", frame_listener.getFrame());
      key = cv::waitKey(40);
    }
  });
#else
  tcp::FrameReceiverServer tcp_frame_listener(address, port);

  std::cout << "Waiting a client connection" << std::endl;

  while (!tcp_frame_listener.connection()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    std::cout << "Waiting for new connection" << std::endl;
  }

  std::cout << "Server and client are connected" << std::endl;

  tcp_frame_listener.readFrames();

  std::thread preview = std::thread([&tcp_frame_listener]() {
    std::cerr << "Pending preview thread" << std::endl;
    int key = 0;
    while (key != 27) {
      cv::imshow("Preview", tcp_frame_listener.getFrame());
      key = cv::waitKey(40);
    }
  });
#endif

  return 0;
}

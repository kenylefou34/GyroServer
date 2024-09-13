#include "../udp/client.hpp"
#include "../ui/mainwindow.h"

#include <arpa/inet.h>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>

#include <QApplication>
#include <QLocale>
#include <QTranslator>
#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

// sudo apt-get install qttools5-dev-tools

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  QTranslator translator;
  const QStringList uiLanguages = QLocale::system().uiLanguages();
  for (const QString &locale : uiLanguages) {
    const QString baseName = "GyroClient_" + QLocale(locale).name();
    if (translator.load(":/i18n/" + baseName)) {
      a.installTranslator(&translator);
      break;
    }
  }

  if (argc != 6) {
    std::cerr << "Usage: " << argv[0] << " <IP> <Port> <Output file>" << std::endl;
    return 1;
  }

  std::string address = argv[1];
  int port = std::stoi(argv[2]);
  std::string dest_address = argv[3];
  int dest_port = std::stoi(argv[4]);
  std::string output_file = argv[5];

  udp::FrameClient udp_frame_listener(address, port, dest_address, dest_port, output_file);
  udp_frame_listener.read();

  std::thread preview = std::thread([&udp_frame_listener]() {
    // std::this_thread::sleep_for(std::chrono::milliseconds{500});
    std::cerr << "Pending thread" << std::endl;
    int key = 0;
    while (key != 27) {
      cv::imshow("Preview", udp_frame_listener.getFrame());
      key = cv::waitKey(40);
    }
  });

  MainWindow w;
  w.show();
  return a.exec();
}

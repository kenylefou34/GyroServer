#include "../stream/streaming_udp.h"
// #include "gl/CustomOpenGLWidget.h"
#include "gl/glwidget.h"
#include "gl/window.h"

#include <Qt3DCore/QEntity>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QDiffuseMapMaterial>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QFrameGraphNode>
#include <Qt3DRender/QParameter>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DRender/QShaderProgram>
#include <Qt3DRender/QTexture>
#include <Qt3DRender/QTextureImage>

#include <QApplication>
#include <QImage>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QSurfaceFormat>
#include <QTimer>
#include <QWidget>
#include <thread>

void setupBackgroundUpdater(CustomOpenGLWidget *oglWidget)
{
  QTimer *timer = new QTimer(oglWidget);
  QObject::connect(timer, &QTimer::timeout, oglWidget, [=]() {
    QImage image(512, 512, QImage::Format_RGBA8888);
    image.fill(Qt::blue); // Replace with dynamically updated content

    // Update OpenGL texture with the new QImage
    oglWidget->updateTexture(image);
  });
  timer->start(50); // Update every 50 ms
}

void setupBackgroundEntity(Qt3DCore::QEntity *rootEntity, QOpenGLTexture *oglTexture)
{
  // Background quad (plane) entity
  Qt3DCore::QEntity *backgroundEntity = new Qt3DCore::QEntity(rootEntity);

  // Set up a 2D plane mesh for the background
  Qt3DExtras::QPlaneMesh *planeMesh = new Qt3DExtras::QPlaneMesh();
  planeMesh->setWidth(10.0f);
  planeMesh->setHeight(10.0f);
  backgroundEntity->addComponent(planeMesh);

  // Placeholder Qt 3D texture
  Qt3DRender::QTexture2D *texture = new Qt3DRender::QTexture2D();
  texture->setGenerateMipMaps(false);

  // Diffuse material that will use the texture
  Qt3DExtras::QDiffuseMapMaterial *material = new Qt3DExtras::QDiffuseMapMaterial();
  material->setDiffuse(texture);
  backgroundEntity->addComponent(material);

  // Use OpenGL to bind the texture later in a rendering loop
  oglTexture->setFormat(QOpenGLTexture::RGBA8_UNorm);
  oglTexture->setSize(512, 512);
  oglTexture->allocateStorage();
}

QImage cvMatToQImage(const cv::Mat &mat)
{
  if (mat.type() == CV_8UC3) { // 3-channel image
    return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).rgbSwapped();
  }
  else if (mat.type() == CV_8UC1) { // Grayscale image
    return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
  }
  else {
    // Unsupported format
    return QImage();
  }
}

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  // Step 1: Set up the 3D window
  Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
  view->defaultFrameGraph()->setClearColor(Qt::black);

  // Create a container widget for the Qt3DWindow
  QWidget *container = QWidget::createWindowContainer(view);
  container->setMinimumSize(QSize(800, 600));
  container->setMaximumSize(QSize(1024, 768));

  // Step 2: Create the root entity for the 3D scene
  Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();

  // Step 3: Initialize the OpenGL texture for the background
  QOpenGLTexture *oglTexture = new QOpenGLTexture(QOpenGLTexture::Target2D);

  // Set up the background entity to display the texture
  setupBackgroundEntity(rootEntity, oglTexture);

  // Set the root entity in the 3D view
  view->setRootEntity(rootEntity);

  // Step 4: Create a CustomOpenGLWidget to manage OpenGL texture updates
  CustomOpenGLWidget *oglWidget = new CustomOpenGLWidget();
  setupBackgroundUpdater(oglWidget); // Set up the timer to update the texture

  // Display the Qt3D container
  container->show();

  StreamingUdp streaming;
  if (argc > 1) {
    streaming.setListenPort(std::atoi(argv[1]));
  }
  std::thread streaming_thread = std::thread([&streaming]() { streaming.stream(); });

  /*
  std::thread preview_thread = std::thread([&streaming]() {
    std::cerr << "Pending preview thread" << std::endl;
    // OpenCV window to display the frames
    cv::namedWindow("H264 Stream", cv::WINDOW_AUTOSIZE);
    while (true) {
      // Display the frame using OpenCV
      const auto &frame = streaming.img();
      if (frame.empty()) {
        continue;
      }
      cv::imshow("H264 Stream", frame);
      cv::waitKey(0);
    }
    cv::destroyWindow("H264 Stream");
  });*/

  return app.exec();
}

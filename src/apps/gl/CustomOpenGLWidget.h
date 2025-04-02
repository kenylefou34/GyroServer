#ifndef GLWIDGET_CUSTOMOPENGLWIDGET_H
#define GLWIDGET_CUSTOMOPENGLWIDGET_H

#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QWidget>

class CustomOpenGLWidget : public QOpenGLWidget {
  Q_OBJECT
 public:
  CustomOpenGLWidget(QWidget *parent = nullptr);
  ~CustomOpenGLWidget();

 protected:
  void initializeGL() override { oglTexture->create(); }

 public:
  void updateTexture(const QImage &image);

  QOpenGLTexture *oglTexture;
};

#endif // GLWIDGET_CUSTOMOPENGLWIDGET_H

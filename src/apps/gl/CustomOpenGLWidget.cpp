#include "CustomOpenGLWidget.h"

#include <QtOpenGLWidgets/QOpenGLWidget>

CustomOpenGLWidget::CustomOpenGLWidget(QWidget *parent) : QOpenGLWidget(parent)
{
  oglTexture = new QOpenGLTexture(QOpenGLTexture::Target2D);
}

void CustomOpenGLWidget::updateTexture(const QImage &image)
{
  makeCurrent();
  if (oglTexture->isCreated()) {
    oglTexture->bind();
    oglTexture->setData(image.mirrored()); // Upload the QImage data
    oglTexture->release();
  }
  doneCurrent();
}

#pragma once
#include <QMainWindow>
#include "openglwidget.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
private:
    OpenGLWidget* ogl_ = nullptr;
}; 
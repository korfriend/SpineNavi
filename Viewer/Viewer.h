#pragma once

#include <QtWidgets/QMainWindow>
#include <QHBoxLayout>
#include <QListWidget>
#include <qstring.h>
#include <qicon.h>
#include <qdebug.h>
#include <qpushbutton.h>
#include <qlabel.h>

#include "ui_Viewer.h"

#include<filesystem>

class Viewer : public QMainWindow
{
    Q_OBJECT

public:
    Viewer(QWidget *parent = nullptr);
    ~Viewer();

    void inline setImagePath(QString path) { image_path = std::filesystem::path(path.toStdString()); };
    void setImages(); // 저장된 path로 부터 image들 item에 저장하는 함수.
    void setThumbnailItems(); // 저장된 item을 thumbnail에 등록
    void setLeftRightImg(); // left right image viewer 에 setting size 맞춰서.

protected:
    virtual void resizeEvent(QResizeEvent* event); // resize시

private:
    Ui::ViewerClass ui;

    std::vector<QListWidgetItem*> items; // thumbnail 표시 할 image 들 저장
    std::filesystem::path image_path;// image들 저장된 경로 저장
    QListWidget* qlistwidget; // thumnail 표시할 widget
    QImage leftImg; // 크게 표시될 두개의 img
    QImage rightImg;
    QLabel* leftView;
    QLabel* rightView;
    int num_imgs;
};

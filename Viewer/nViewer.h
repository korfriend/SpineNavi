//#pragma once
//#pragma once
//
//
//#include <QMainWindow>
//#include <QHBoxLayout>
//#include <QListWidget>
//#include <qstring.h>
//#include <qicon.h>
//#include <qdebug.h>
//#include <qpushbutton.h>
//#include <qlabel.h>
//#include <qtimer.h>
//#include <QMouseEvent>
//#include <QKeyEvent>
//
//#include "ui_Viewer.h"
//
//#include "ViewMgr.h"
//
//#include<filesystem>
//#include <iostream>
//#include <memory>
//
//class Q_DECL_EXPORT Viewer : public QMainWindow
//{
//    Q_OBJECT
//
//public:
//    Viewer(QWidget* parent = nullptr);
//    ~Viewer();
//
//    void inline setImagePath(QString path) { image_path = std::filesystem::path(path.toStdString()); };
//    void setImages(); // ����� path�� ���� image�� item�� �����ϴ� �Լ�.
//    void setThumbnailItems(); // ����� item�� thumbnail�� ���
//    void setLeftRightImg(); // left right image viewer �� setting size ���缭.
//
//
//protected:
//    virtual void resizeEvent(QResizeEvent* event); // resize��
//
//    void mousePressEvent(QMouseEvent* event) override;
//    void mouseMoveEvent(QMouseEvent* event) override;
//    void mouseReleaseEvent(QMouseEvent* event) override;
//
//    void keyPressEvent(QKeyEvent* event) override;
//private:
//    Ui::ViewerClass ui;
//
//
//    ViewMgr* m_ViewMgr;
//
//    int num_imgs;
//    std::vector<Image2D> m_imgList;
//    std::vector<QString> image_paths;
//
//    std::filesystem::path image_path;// image�� ����� ��� ����
//
//    QTimer* m_qtimer;
//
//
//
//private slots:
//    void clickedImage();
//    void TimerProc();
//};


//TODO : timer ��� ���� ������ Ȯ��.
// thread �����ϴ��� Ȯ��.
// �̹��� �޾ƿ��� ����.

// layout.

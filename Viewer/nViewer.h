#pragma once

#include "Engine.h"
#include "ui_Viewer.h"
#include "ViewMgr.h"
#include "ViewLayout.h"


#include <QMainWindow>
#include <QHBoxLayout>
#include <QListWidget>
#include <qstring.h>
#include <qicon.h>
#include <qdebug.h>
#include <qpushbutton.h>
#include <qlabel.h>
#include <qtimer.h>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QPalette>


#include <filesystem>
#include <iostream>
#include <memory>




class Q_DECL_EXPORT nViewer : public QMainWindow
{
    Q_OBJECT

public:
    nViewer(QWidget* parent = nullptr);
    ~nViewer();

    void initLayout();

private:
    Ui::ViewerClass ui;

    ViewMgr* m_ViewMgr;

    int num_imgs;
    std::vector<Image2D> m_imgList;
    std::vector<QString> image_paths;

    QTimer* m_qtimer;

    Engine* m_engine;

    QString m_folder_data;

    ViewLayout* m_mainFrame;

    QListWidget* m_thumbnails;
};


//TODO : image setting만 되면 끝.

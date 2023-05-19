#include "Viewer.h"

Viewer::Viewer(QWidget *parent)
    : QMainWindow(parent)
{
    
    ui.setupUi(this);

    num_imgs = 0;

    QHBoxLayout* hboxLayout = new QHBoxLayout(this->centralWidget());
    qlistwidget = new QListWidget(); // thumbnail view
    leftView = new myLabel();
    leftView->setStyleSheet(
        "color: #4D69E8; border-style: solid; border-width: 5px; border-color: #54A0FF; border-radius: 10px; ");
    selectedView = 0;
    leftView->setFocused(true);

    rightView = new myLabel();
    rightView->setFocused(false);
    
    qlistwidget->setViewMode(QListWidget::IconMode);
    qlistwidget->setIconSize(QSize(200, 150));
    qlistwidget->setResizeMode(QListWidget::Adjust);

    setImagePath("../data/c-arm 2023-04-19");

    hboxLayout->addWidget(leftView, 40);
    hboxLayout->addWidget(rightView, 40);
    hboxLayout->addWidget(qlistwidget, 20);
    setImages();
    //ui.centralWidget->setLayout(hboxLayout);

    connect(qlistwidget, SIGNAL(itemDoubleClicked(QListWidgetItem*)),this, SLOT(clickedImage()));
    connect(leftView, SIGNAL(clicked()), this, SLOT(leftClicked()));
    connect(rightView, SIGNAL(clicked()), this, SLOT(rightClicked()));
}

Viewer::~Viewer()
{}

void Viewer::leftClicked()
{
    leftView->setFocused(true);
    leftView->setStyleSheet(
        "color: #4D69E8; border-style: solid; border-width: 5px; border-color: #54A0FF; border-radius: 10px; ");

    rightView->setFocused(false);
    rightView->setStyleSheet("");
    selectedView = 0;
}
void Viewer::rightClicked()
{
    rightView->setFocused(false);
    rightView->setStyleSheet(
        "color: #4D69E8; border-style: solid; border-width: 5px; border-color: #54A0FF; border-radius: 10px; ");

    leftView->setFocused(false);
    leftView->setStyleSheet("");
    selectedView = 1;
}
void Viewer::setImages()
{
    for (auto const& dir_entry : std::filesystem::directory_iterator{ image_path })
    {
        qDebug() << dir_entry.path().u8string() << '\n';
        if (dir_entry.path().extension() == ".png")
        {
            QListWidgetItem* item = new QListWidgetItem(QIcon(dir_entry.path().u8string().c_str()), QString(dir_entry.path().filename().u8string().c_str()));
            image_paths.push_back(dir_entry.path().u8string().c_str());
            items.push_back(item);
            if (num_imgs == 0)
            {
                leftImg = QImage(dir_entry.path().u8string().c_str());
            }
            if (num_imgs == 1)
            {
                rightImg = QImage(dir_entry.path().u8string().c_str());
            }
            num_imgs++;
        }
    }    
    setThumbnailItems();
    setLeftRightImg();
}

void Viewer::setLeftRightImg()
{
    int w = leftView->width();
    int h = leftView->height();
    leftView->setPixmap(QPixmap::fromImage(leftImg).scaled(w, h, Qt::KeepAspectRatio));

    w = rightView->width();
    h = rightView->height();
    rightView->setPixmap(QPixmap::fromImage(rightImg).scaled(w, h, Qt::KeepAspectRatio));
}

void Viewer::resizeEvent(QResizeEvent* event)
{
    updateGeometry();
    setLeftRightImg();
}

void Viewer::setThumbnailItems()
{
    for (auto i : items)
    {
        qlistwidget->addItem(i);
    }

}

void Viewer::clickedImage()
{
    QModelIndex index = qlistwidget->currentIndex();
    int inx = index.row();

    if(selectedView == 0)
        leftImg = QImage(image_paths[inx]);
    else
        rightImg = QImage(image_paths[inx]);

    setLeftRightImg();


}
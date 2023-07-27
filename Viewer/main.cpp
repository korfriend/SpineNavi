#include "Viewer.h"
#include "nViewer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    nViewer w;
    w.show();
    return a.exec();
}

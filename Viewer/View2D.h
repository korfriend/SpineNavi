#pragma once

#include "Image2D.h"


#include <QtWidgets>
#include <qlabel.h>
#include <qapplication.h>

class Q_DECL_EXPORT View2D : public QGraphicsView
{
	Q_OBJECT


public:
	View2D(int Id, QWidget* pParent = 0);
	~View2D();

	void setImage(QImage img) { m_image->setImage(img); }; // set member image
	void display(); // display image to view scene

private:
	Image2D* m_image;
	QGraphicsPixmapItem* m_pixmap;
};
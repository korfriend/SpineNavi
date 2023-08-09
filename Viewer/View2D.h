#pragma once

#include "Image2D.h"
#include "Enums.h"
#include "Defines.h"

#include <QtWidgets>
#include <qlabel.h>
#include <qapplication.h>

class Q_DECL_EXPORT View2D : public QGraphicsView
{
	Q_OBJECT


public:
	View2D(VIEW_TYPE type, QWidget* pParent = 0);
	~View2D();

	void setImage(Image2D img) { m_image = img; }; // set member image
	void display(); // display image to view scene

	bool isNull() { return m_image.isNull(); };

protected:
	virtual void resizeEvent(QResizeEvent* pEvent) override;

private:
	Image2D m_image;

	QGraphicsPixmapItem* m_renderPixmap;

	VIEW_TYPE m_type;

	int m_width;
	int m_height;
};
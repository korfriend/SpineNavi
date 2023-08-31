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

	virtual void wheelEvent(QWheelEvent* pEvent) override;
	virtual void mouseMoveEvent(QMouseEvent* pEvent) override;
	virtual void mousePressEvent(QMouseEvent* pEvent) override;
	virtual void mouseReleaseEvent(QMouseEvent* pEvent) override;
	virtual void keyPressEvent(QKeyEvent* pEvent) override;
	virtual void keyReleaseEvent(QKeyEvent* pEvent) override;
	virtual void leaveEvent(QEvent* pEvent) override;
	virtual void mouseDoubleClickEvent(QMouseEvent* pEvent) override;

private:
	Image2D m_image;

	QGraphicsPixmapItem* m_renderPixmap;

	VIEW_TYPE m_type;

	int m_width;
	int m_height;
};
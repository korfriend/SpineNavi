#pragma once

#include "View2D.h"
#include "Defines.h"
#include "Image2D.h"

#include <QWidget>
#include <QVBoxLayout>
#include <QImage>

class Q_DECL_EXPORT WindowCArm : public QWidget
{
	Q_OBJECT
public:
	WindowCArm(VIEW_TYPE type, QWidget* parent = 0);
	~WindowCArm();

	void setView(View2D view);

	inline View2D* getView() { return m_view; }

	inline void setImage(Image2D img) { m_view->setImage(img); }
	inline void display(void) { m_view->display(); }

signals:
	void sigEvent(QEvent*);

private slots:
	void slotEvent(QEvent* pEvent);

private:

	View2D* m_view;

	VIEW_TYPE m_type;


};
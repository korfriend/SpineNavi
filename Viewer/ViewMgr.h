#pragma once


#include "ViewLayout.h"
#include "WindowCArm.h"
#include "View2D.h"
#include "Enums.h"
#include "Defines.h"

#include <qwidget.h>

class Q_DECL_EXPORT ViewMgr : public QWidget
{
	Q_OBJECT
public:
	ViewMgr(ViewLayout* layout, QWidget* parent = 0);
	~ViewMgr();

public:
	void setAPImg(Image2D img) {
		m_windowAP->setImage(img);
		m_windowAP->display();
	};
	void setLateralImg(Image2D img) { 
		m_windowLateral->setImage(img); 
		m_windowLateral->display();
	};

private:

	// AP View.
	WindowCArm* m_windowAP;
	
	// Lateral View
	WindowCArm* m_windowLateral;

	// Layout classes.
	ViewLayout* m_mainLayout;
	LAYOUT_TYPE			m_CurrentLayout;
};


#pragma once

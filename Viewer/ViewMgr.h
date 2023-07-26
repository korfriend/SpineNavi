#pragma once


#include "ViewLayout.h"
#include "View2D.h"

#include <qwidget.h>

class Q_DECL_EXPORT ViewMgr : public QWidget
{
	Q_OBJECT
public:
	ViewMgr(ViewLayout* layout, QWidget* pParent = 0);
	~ViewMgr();

public:

private:

	// AP View.
	View2D* m_AView;
	
	// Lateral View
	View2D* m_LView;

	// Layout classes.
	ViewLayout* m_Layout;
	LAYOUT_TYPE			m_CurrentLayout;
};


#pragma once

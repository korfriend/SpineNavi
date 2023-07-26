#pragma once

#include <QWidget>
#include <qgraphicsview.h>
#include <qlist.h>
#include <qgridlayout.h>

#include "Enums.h"

class Q_DECL_EXPORT ViewLayout : public QWidget
{
	Q_OBJECT

public:
	ViewLayout(QWidget* parent = nullptr);
	~ViewLayout();

	QLayout* getViewLayout(void) { return m_viewLayout; }

public:
	bool setSingleLayout(QWidget* view);
	bool setDoubleLayout(QWidget* view1, QWidget* view2);
	
private:
	void setViewLayout(QLayout* layout);

private:
	QLayout* m_viewLayout;
};

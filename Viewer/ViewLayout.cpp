#include "ViewLayout.h"

ViewLayout::ViewLayout(QWidget* parent)
	: QWidget(parent)
{
	m_viewLayout = nullptr;

	QVBoxLayout* layout = new QVBoxLayout;
	this->setLayout(layout);

	QSizePolicy expandingPolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	expandingPolicy.setHorizontalStretch(0);
	expandingPolicy.setVerticalStretch(0);
	setSizePolicy(expandingPolicy);

	this->layout()->setContentsMargins(QMargins(0, 0, 0, 0));

	setMouseTracking(true);
}

ViewLayout::~ViewLayout()
{
}

bool ViewLayout::setSingleLayout(QWidget* view)
{
	if (view == nullptr) return false;

	QGridLayout* gLayout = new QGridLayout;
	gLayout->setSpacing(0);

	gLayout->setColumnMinimumWidth(0, 300);
	gLayout->setColumnStretch(0, 1);
	gLayout->setRowMinimumHeight(0, 300);
	gLayout->setRowStretch(0, 1);
	gLayout->addWidget(view, 0, 0);

	setViewLayout(gLayout);

	return true;
}

bool ViewLayout::setDoubleLayout(QWidget* view1, QWidget* view2)
{
	if (view1 == nullptr || view2 == nullptr) return false;

	QHBoxLayout* LRLayout = new QHBoxLayout;
	LRLayout->setSpacing(0);
	LRLayout->addWidget(view1);
	LRLayout->setStretch(0, 1);
	LRLayout->addWidget(view2);
	LRLayout->setStretch(1, 1);


	setViewLayout(LRLayout);

	return true;
}

void ViewLayout::setViewLayout(QLayout* layout)
{
	QVBoxLayout* tabLayout = (QVBoxLayout*)this->layout();

	if (m_viewLayout)
	{
		tabLayout->removeItem(m_viewLayout);
		delete m_viewLayout;
	}

	m_viewLayout = layout;
	tabLayout->addLayout(m_viewLayout);
}

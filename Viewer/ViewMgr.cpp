#include "ViewMgr.h"


ViewMgr::ViewMgr(ViewLayout* layout, QWidget* parent)
	: QWidget(parent)
{
	m_mainLayout = layout;

	m_windowAP = new WindowCArm(VIEW_TYPE::AP);
	m_windowLateral = new WindowCArm(VIEW_TYPE::LATERAL);

	m_mainLayout->setDoubleLayout(m_windowAP, m_windowLateral);
}
ViewMgr::~ViewMgr()
{

	SAFE_DELETE_OBJECT(m_windowAP);
	SAFE_DELETE_OBJECT(m_windowLateral);
}

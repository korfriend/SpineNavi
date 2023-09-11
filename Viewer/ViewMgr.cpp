#include "ViewMgr.h"


ViewMgr::ViewMgr(ViewLayout* layout, RenderEngine* renderer, QWidget* parent)
	: QWidget(parent)
{
	m_mainLayout = layout;

	m_windowAP = new WindowCArm(VIEW_TYPE::AP);
	m_windowLateral = new WindowCArm(VIEW_TYPE::LATERAL);
	m_windowCalib = new WindowCArm(VIEW_TYPE::CALIB);

	//m_mainLayout->setSingleLayout(m_windowCalib);
	m_mainLayout->setDoubleLayout(m_windowAP, m_windowLateral);
	g_renderer = renderer;

	connect(m_windowCalib, SIGNAL(sigEvent(QEvent*)), g_renderer, SLOT(slotEventHandler(QEvent*)));
}
ViewMgr::~ViewMgr()
{

	SAFE_DELETE_OBJECT(m_windowAP);
	SAFE_DELETE_OBJECT(m_windowLateral);
	SAFE_DELETE_OBJECT(m_windowCalib);
}

void ViewMgr::setCalibMode()
{
	m_windowAP->setVisible(false);
	m_windowLateral->setVisible(false);
	m_mainLayout->setSingleLayout(m_windowCalib);
}

void ViewMgr::setNaviMode()
{
	m_windowCalib->setVisible(false);
	m_mainLayout->setDoubleLayout(m_windowAP, m_windowLateral);
}

void ViewMgr::slotCalibMousePress(glm::ivec2 pos)
{
	
}
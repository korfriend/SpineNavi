#include "WindowCArm.h"

WindowCArm::WindowCArm(VIEW_TYPE type, QWidget* parent)
	: QWidget(parent)
{
	m_type = type;
	m_view = new View2D(type, this);


	QVBoxLayout* layout = new QVBoxLayout();

	QSizePolicy expandingPolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	expandingPolicy.setHeightForWidth(true);
	m_view->setSizePolicy(expandingPolicy);

	layout->addWidget(m_view);
	this->setLayout(layout);
	this->layout()->setContentsMargins(QMargins(0, 0, 0, 0));

	this->setVisible(true);

	connect(m_view, SIGNAL(sigMousepress(QEvent*)), this, SLOT(slotEvent(QEvent*)));
}

WindowCArm::~WindowCArm()
{
	SAFE_DELETE_OBJECT(m_view);
}

void WindowCArm::slotEvent(QEvent* pEvent)
{
	emit sigEvent(pEvent);
}
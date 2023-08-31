#include "View2D.h"


View2D::View2D(VIEW_TYPE type, QWidget* parent)
	: QGraphicsView(parent)
{
	m_type = type;
	//this->setStyleSheet("QGraphicsView{ border: 1px solid white; }");

	m_renderPixmap = new QGraphicsPixmapItem(QPixmap());
	m_renderPixmap->setTransformationMode(Qt::SmoothTransformation);
	this->setRenderHint(QPainter::Antialiasing, true);

	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

	

	setContentsMargins(QMargins(0, 0, 0, 0));

	QGraphicsScene* pScene = new QGraphicsScene(this);
	this->setScene(pScene);
	
	this->scene()->setSceneRect(0, 0, 1296, 1296); //check
	this->scene()->addItem(m_renderPixmap);

	m_width = this->width();
	m_height = this->height();
}

View2D::~View2D()
{
	SAFE_DELETE_OBJECT(m_renderPixmap);
	// TODO
	// image release 되는 곳 체크.
	// image 주고 받는 방법 pointer, copy 체크
}

void View2D::resizeEvent(QResizeEvent* pEvent)
{
	QSize size = pEvent->size();
	float ratio = (float)size.width() / (float)m_width;

	this->scale(ratio, ratio);

	m_width = size.width();
	m_height = size.height();
	QGraphicsView::resizeEvent(pEvent);
	//std::cout << size.width() << " , " << size.height() << std::endl;


}

void View2D::wheelEvent(QWheelEvent* pEvent)
{
}

void View2D::mouseMoveEvent(QMouseEvent* pEvent)
{
}

void View2D::mousePressEvent(QMouseEvent* pEvent)
{
}

void View2D::mouseReleaseEvent(QMouseEvent* pEvent)
{
}

void View2D::keyPressEvent(QKeyEvent* pEvent)
{
}

void View2D::keyReleaseEvent(QKeyEvent* pEvent)
{
}

void View2D::leaveEvent(QEvent* pEvent)
{
}

void View2D::mouseDoubleClickEvent(QMouseEvent* pEvent)
{
}

void View2D::display()
{
	if (!m_image.isNull()) {
		//qDebug() << "VIEW2D display() " << m_image.getQImage().size();
		//qDebug() << this->width() << this->height();

		
		//qDebug() << this->width() << this->height();
		m_renderPixmap->setPixmap(QPixmap::fromImage(m_image.getQImage()));
		this->fitInView(0, 0, m_image.width(), m_image.height(), Qt::AspectRatioMode::KeepAspectRatio);
		m_width = this->width();
		m_height = this->height();
	}
	//TODO image setting 하기.
}

#include "clickableLabel.h"

myLabel::myLabel(QWidget* parent)
	:QLabel(parent)
{
	connect(this, SIGNAL(clicked()), this, SLOT(slotClicked()));
	is_focused = true;
}

void myLabel::slotClicked()
{
	//qDebug() << "Clicked";
}

void myLabel::mousePressEvent(QMouseEvent* event)
{
	emit clicked();
}

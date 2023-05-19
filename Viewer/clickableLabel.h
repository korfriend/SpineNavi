#pragma once

#include <qlabel.h>


class myLabel : public QLabel
{
	Q_OBJECT
public:
	myLabel(QWidget* parent = 0);
	~myLabel() {}

	void setFocused(bool f) { is_focused = f; };
	bool getFocused() { return is_focused; };
signals:
	void clicked();

public slots:
	void slotClicked();

protected:
	void mousePressEvent(QMouseEvent* event);
	bool is_focused;

};
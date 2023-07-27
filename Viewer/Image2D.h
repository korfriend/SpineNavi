#pragma once

#include <QImage>

class Q_DECL_EXPORT Image2D
{

public:
	Image2D(int id, int type, QImage img);
	~Image2D();


	void setImage(QImage img) { m_img = img; }

private:

	int m_id;
	int m_type; //AP or Lateral

	QImage m_img;
};
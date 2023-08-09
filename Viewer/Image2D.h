#pragma once
#include "Enums.h"

#include <QImage>

class Q_DECL_EXPORT Image2D
{

public:
	Image2D();
	Image2D(int id, VIEW_TYPE type, QImage img);
	
	~Image2D();


	void setImage(QImage img) { m_img = img; };
	bool isNull() { return m_img.isNull(); };
	int width() { return m_img.width(); };
	int height() { return m_img.height(); };

	QImage getQImage() { return m_img; };

private:

	int m_id;
	VIEW_TYPE m_type; //AP or Lateral

	QImage m_img;
};
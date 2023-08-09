#include "Image2D.h"

Image2D::Image2D()
{
}

Image2D::Image2D(int id, VIEW_TYPE type, QImage img)
{
	m_id = id;
	m_type = type;
	m_img = img;
}

Image2D::~Image2D()
{
}

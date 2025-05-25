#include "bitmapdata.h"
#include "flash/geom/matrix.h"
#include "flash/geom/geomrectangle.h"
#include <QImage>
#include <QPainter>
#include <QBuffer>
#include <emmintrin.h>
#include "flash/geom/colortransform.h"



static uint32_t hard_light(uint32_t color_32, uint32_t m_32)
{
	union Color
	{
		struct
		{
			uint8_t b;
			uint8_t g;
			uint8_t r;
			uint8_t a;
		};
		uint32_t u32;
	};

	Color color;
	color.u32 = color_32;

	Color m;
	m.u32 = m_32;

	const uint8_t bytes_1[8] = {color.b, 0x00, color.g, 0x00, color.r, 0x00, 0xFF, 0x00};
	const uint8_t bytes_2[8] = {m.b, 0x00, m.g, 0x00, m.r, 0x00, 0xFF, 0x00};

	__m64 mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7;

	mm6 = _m_from_int(0);
	mm6 = _m_pcmpeqw(mm6, mm6);
	mm6 = _m_psrlw(mm6, _m_from_int(15));

	mm7 = _m_from_int(0);
	mm7 = _m_pcmpeqw(mm7, mm7);
	mm7 = _m_psrlw(mm7, _m_from_int(8));

	mm0 = *(__m64*)bytes_1;
	mm1 = *(__m64*)bytes_2;

	mm5 = _m_pshufw(mm1, -1);
	mm2 = _m_pxor(mm5, mm7);
	mm2 = _m_pmullw(mm2, mm0);
	mm3 = _m_pshufw(mm0, -1);
	mm3 = _m_pxor(mm3, mm7);
	mm4 = _m_psllw(mm0, _m_from_int(1));
	mm3 = _m_paddw(mm3, mm4);
	mm3 = _m_pmullw(mm3, mm1);
	mm2 = _m_paddw(mm2, mm3);
	mm3 = _m_psrlw(mm2, _m_from_int(8));
	mm2 = _m_paddw(mm2, mm3);
	mm2 = _m_paddw(mm2, mm6);
	mm2 = _m_psrlw(mm2, _m_from_int(8));
	mm3 = _m_psubw(mm5, mm1);
	mm4 = _m_pshufw(mm0, -1);
	mm3 = _m_pmullw(mm3, mm4);
	mm4 = _m_psllw(mm1, _m_from_int(1));
	mm4 = _m_psubw(mm4, mm5);
	mm4 = _m_pmullw(mm4, mm0);
	mm3 = _m_paddw(mm3, mm4);
	mm3 = _m_paddw(mm3, mm7);
	mm4 = _m_psrlw(mm3, _m_from_int(8));
	mm3 = _m_paddw(mm3, mm4);
	mm3 = _m_paddw(mm3, mm6);
	mm3 = _m_psrlw(mm3, _m_from_int(8));
	mm0 = _m_paddw(mm0, mm1);
	mm0 = _m_psubw(mm0, mm3);
	mm4 = _m_pshufw(mm1, -1);
	mm4 = _m_psrlw(mm4, _m_from_int(1));
	mm5 = _m_paddw(mm4, mm6);
	mm5 = _m_pcmpgtw(mm5, mm1);
	mm2 = _m_pand(mm2, mm5);
	mm5 = _m_pandn(mm5, mm0);
	mm2 = _m_por(mm2, mm5);

	union U
	{
		__m64 m;
		uint8_t b[8];
	};
	U u;
	u.m = mm2;

	color.r = u.b[4];
	color.g = u.b[2];
	color.b = u.b[0];

	_m_empty();

	return color.u32;
}




BitmapData::BitmapData()
{
	this->_width = 0;
	this->_height = 0;
	this->_transparent = false;
	this->image = new QImage();
}

BitmapData::BitmapData(uint32_t width, uint32_t height, bool transparent, uint32_t fillColor)
{
	this->_width = width;
	this->_height = height;
	this->_transparent = transparent;

	QImage::Format format = transparent ? QImage::Format_ARGB32_Premultiplied : QImage::Format_RGB32;
	this->image = new QImage(width, height, format);
	this->image->fill(fillColor);
}

BitmapData::~BitmapData()
{
	delete this->image;
}

void BitmapData::fill(uint32_t color)
{
	this->image->fill(color);
}

void BitmapData::beginBitmapFill(const BitmapData *src)
{
	QPainter p(this->image);

	for (uint32_t x = 0; x < this->_width;)
	{
		for (uint32_t y = 0; y < this->_height;)
		{
			p.drawImage(x, y, *src->image);
			y += src->_height;
		}

		x += src->_width;
	}
	p.end();
}

bool BitmapData::loadFromData(const QByteArray &data)
{
	bool result = this->image->loadFromData(data);

	if (result == false)
	{
		this->_width = false;
		this->_height = false;
		this->_transparent = false;
		return false;
	}

	this->_width =  this->image->width();
	this->_height =  this->image->height();

	QImage::Format format = this->image->format();

	if (format == QImage::Format_ARGB32)
	{
		this->image->convertTo(QImage::Format_ARGB32_Premultiplied);
		this->_transparent = true;
		return true;
	}

	if (format == QImage::Format_RGB32)
	{
		this->_transparent = false;
		return true;
	}

	if (format == QImage::Format_Indexed8)
	{
		this->image->convertTo(QImage::Format_ARGB32_Premultiplied);
		this->_transparent = true;
		return true;
	}

	return false;
}

QByteArray BitmapData::toData()
{
	QByteArray data;
	QBuffer buffer(&data);
	buffer.open(QIODevice::WriteOnly);
	this->image->save(&buffer, "png");
	return data;
}

uint32_t* BitmapData::data()
{
	return (uint32_t*)this->image->bits();
}

uint32_t BitmapData::size()
{
	return 0;
}

void BitmapData::copy(const BitmapData *src)
{
	*this->image = *src->image;
	this->_width =  this->image->width();
	this->_height =  this->image->height();
	this->_transparent = src->_transparent;
}

void BitmapData::scaled(int w, int h)
{
	*this->image = this->image->scaled(QSize(w, h), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	this->_width =  this->image->width();
	this->_height =  this->image->height();
}


int BitmapData::width() const
{
	return _width;
}

int BitmapData::height() const
{
	return _height;
}

bool BitmapData::transparent()
{
	return _transparent;
}

void BitmapData::dispose()
{

}

void BitmapData::getRect(GeomRectangle *rectangle)
{
	rectangle->x = 0;
	rectangle->y = 0;
	rectangle->width = _width;
	rectangle->height = _height;
}

void BitmapData::draw(IBitmapDrawable *source, Matrix *matrix, ColorTransform *colorTransform,
					  BlendMode blendMode, GeomRectangle *clipRect, bool smoothing)
{
	(void)blendMode;
	(void)clipRect;
	(void)smoothing;

	BitmapData *_source = (BitmapData*)source;

	if (blendMode == BlendMode::HARDLIGHT)
	{
		int width = _source->width();
		int height = _source->height();

		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				QRgb rgb = image->pixel(x, y);
				QRgb rgb_src = _source->image->pixel(x, y);

				rgb = hard_light(rgb, rgb_src);

				image->setPixel(x, y, rgb);
			}
		}
		return;
	}


	int v1, v2, v3, v4;

	if (clipRect != nullptr)
	{
		v1 = clipRect->x;
		v2 = clipRect->y;
		v3 = clipRect->width;
		v4 = clipRect->height;
	}
	else
	{
		v1 = 0;
		v2 = 0;
		v3 = _source->width();
		v4 = _source->height();
	}


	QImage image2;

	int xx = 0;
	int yy = 0;

	if (matrix != nullptr)
	{
		QTransform transform(matrix->a, matrix->b, matrix->c, matrix->d, matrix->tx, matrix->ty);
		image2 = _source->image->transformed(transform, Qt::SmoothTransformation);
		xx = matrix->tx;
		yy = matrix->ty;
	}
	else
	{
		image2 = *_source->image;
	}

	if (colorTransform != nullptr)
	{
		int count = image2.sizeInBytes() / 4;
		uint32_t *p = (uint32_t*)image2.bits();
		for (int i = 0; i < count; i++)
			p[i] = colorTransform->get(p[i]);
	}


	QPainter p;
	p.begin(this->image);
	p.drawImage(xx, yy, image2, v1, v2, v3, v4);
	p.end();
}



void BitmapData::copyPixels(const BitmapData *sourceBitmapData,
							const GeomRectangle *sourceRect,
							const Point *destPoint,
							BitmapData *alphaBitmapData,
							Point *alphaPoint,
							bool mergeAlpha)
{
	(void)alphaBitmapData;
	(void)alphaPoint;
	(void)mergeAlpha;

	QPainter p(this->image);
	p.drawImage(destPoint->x, destPoint->y, *sourceBitmapData->image, sourceRect->x, sourceRect->y, sourceRect->width, sourceRect->height);
	p.end();
}

void BitmapData::setPixel(int x, int y, uint32_t color)
{
	this->image->setPixel(x, y, color);
}

uint32_t BitmapData::getPixel(int x, int y)
{
	return this->image->pixel(x, y);
}


void BitmapData::colorTransform(const ColorTransform *colorTransform)
{
	(void)colorTransform;
}

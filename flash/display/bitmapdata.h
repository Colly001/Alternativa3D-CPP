#ifndef BITMAPDATA_H
#define BITMAPDATA_H

#include <stdint.h>
#include "flash/display/IBitmapDrawable.h"
#include "flash/display/BlendMode.h"


class Matrix;
class ColorTransform;
class QString;
class GeomRectangle;
class IBitmapDrawable;
class Point;
class QImage;
class QByteArray;


class BitmapData : public IBitmapDrawable
{
	uint32_t _width;
	uint32_t _height;
	bool _transparent;
	QImage *image;
public:
	BitmapData();
	BitmapData(uint32_t width, uint32_t height, bool transparent = true, uint32_t fillColor = 0xFFFFFFFF);
	~BitmapData();

	void fill(uint32_t color);

	void beginBitmapFill(const BitmapData *src);

	bool loadFromData(const QByteArray &data);
	QByteArray toData();

	QImage* qimage() { return image; }


	uint32_t* data();
	uint32_t size();

	void copy(const BitmapData *src);

	int width() const;
	int height() const;
	bool transparent();

	void dispose();

	//[только для чтения] Прямоугольник, определяющий размер и расположение растрового изображения.
	void getRect(GeomRectangle *rectangle);



	//Отображает исходный экранный объект source поверх растрового изображения с помощью векторного средства визуализации среды выполнения Flash.
	void draw(IBitmapDrawable *source,
			  Matrix *matrix = nullptr,
			  ColorTransform *colorTransform = nullptr,
			  BlendMode blendMode = BlendMode::NONE,
			  GeomRectangle *clipRect = nullptr,
			  bool smoothing = false);


	//Представляет собой процедуру быстрой обработки точек изображений без растяжения, поворота и цветовых эффектов.
	void copyPixels(const BitmapData *sourceBitmapData,
					const GeomRectangle *sourceRect,
					const Point *destPoint,
					BitmapData *alphaBitmapData = nullptr,
					Point *alphaPoint = nullptr,
					bool mergeAlpha = false);


	void setPixel(int x, int y, uint32_t color);
	uint32_t getPixel(int x, int y);
	void scaled(int w, int h);
	void colorTransform(const ColorTransform *colorTransform);
};

#endif // BITMAPDATA_H

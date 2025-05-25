#ifndef COLORTRANSFORM_H
#define COLORTRANSFORM_H

#include <stdint.h>
class QString;


class ColorTransform
{
public:

	double alphaMultiplier; //Десятичное значение, умножаемое на значение канала альфа-прозрачности.
	double alphaOffset; //Число от -255 до 255, добавляемое к значению канала альфа-прозрачности после умножения на значение alphaMultiplier.

	double blueMultiplier; //Десятичное значение, умножаемое на значение синего канала.
	double blueOffset; //Число от -255 до 255, добавляемое к значению синего канала после умножения на значение blueMultiplier.

	double greenMultiplier; //Десятичное значение, умножаемое на значение зеленого канала.
	double greenOffset; //Число от -255 до 255, добавляемое к значению зеленого канала после умножения на значение greenMultiplier.

	double redMultiplier; //Десятичное значение, умножаемое на значение красного канала.
	double redOffset; //Число от -255 до 255, добавляемое к значению красного канала после умножения на значение redMultiplier.

	//Создает объект ColorTransform для экранного объекта в заданными значениями цветовых каналов и значениями альфа-канала.
	ColorTransform(double redMultiplier = 1.0,
				   double greenMultiplier = 1.0,
				   double blueMultiplier = 1.0,
				   double alphaMultiplier = 1.0,
				   double redOffset = 0,
				   double greenOffset = 0,
				   double blueOffset = 0,
				   double alphaOffset = 0)
	{
		this->redMultiplier = redMultiplier;
		this->greenMultiplier = greenMultiplier;
		this->blueMultiplier = blueMultiplier;
		this->alphaMultiplier = alphaMultiplier;
		this->redOffset = redOffset;
		this->greenOffset = greenOffset;
		this->blueOffset = blueOffset;
		this->alphaOffset = alphaOffset;
	}

	void concat(const ColorTransform*);

	uint32_t get(uint32_t color) const;
};

#endif // COLORTRANSFORM_H

#include "boundbox.h"
#include <QString>
#include "_global.h"

/*
BoundBox::BoundBox()
{
	_function_name("BoundBox::BoundBox");
	//this->infinity();
}
*/

void BoundBox::setSize(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
{
	_function_name("BoundBox::setSize");
	this->minX = minX;
	this->minY = minY;
	this->minZ = minZ;
	this->maxX = maxX;
	this->maxY = maxY;
	this->maxZ = maxZ;
}

void BoundBox::addBoundBox(const BoundBox *boundBox)
{
	_function_name("BoundBox::addBoundBox");
	this->minX = (boundBox->minX < this->minX) ? boundBox->minX : this->minX;
	this->minY = (boundBox->minY < this->minY) ? boundBox->minY : this->minY;
	this->minZ = (boundBox->minZ < this->minZ) ? boundBox->minZ : this->minZ;
	this->maxX = (boundBox->maxX > this->maxX) ? boundBox->maxX : this->maxX;
	this->maxY = (boundBox->maxY > this->maxY) ? boundBox->maxY : this->maxY;
	this->maxZ = (boundBox->maxZ > this->maxZ) ? boundBox->maxZ : this->maxZ;
}

void BoundBox::addPoint(double x, double y, double z)
{
	_function_name("BoundBox::addPoint");

	if (x < this->minX)
	{
		this->minX = x;
	}
	if (x > this->maxX)
	{
		this->maxX = x;
	}
	if (y < this->minY)
	{
		this->minY = y;
	}
	if (y > this->maxY)
	{
		this->maxY = y;
	}
	if (z < this->minZ)
	{
		this->minZ = z;
	}
	if (z > this->maxZ)
	{
		this->maxZ = z;
	}
}

void BoundBox::infinity()
{
	_function_name("BoundBox::infinity");

	this->minX = 1E308;
	this->minY = 1E308;
	this->minZ = 1E308;
	this->maxX = -1E308;
	this->maxY = -1E308;
	this->maxZ = -1E308;
}

bool BoundBox::intersects(const BoundBox *bb, double epsilon) const
{
	_function_name("BoundBox::intersects");

	return !((((((this->minX > (bb->maxX + epsilon)) || (this->maxX < (bb->minX - epsilon))) || (this->minY > (bb->maxY + epsilon))) || (this->maxY < (bb->minY - epsilon))) || (this->minZ > (bb->maxZ + epsilon))) || (this->maxZ < (bb->minZ - epsilon)));
}

void BoundBox::copyFrom(const BoundBox *boundBox)
{
	_function_name("BoundBox::copyFrom");

	this->minX = boundBox->minX;
	this->minY = boundBox->minY;
	this->minZ = boundBox->minZ;
	this->maxX = boundBox->maxX;
	this->maxY = boundBox->maxY;
	this->maxZ = boundBox->maxZ;
}


QString BoundBox::toString() const
{
	_function_name("BoundBox::toString");

	return QString::number(this->minX, 'g', 20) + " " +
			QString::number(this->minY, 'g', 20) + " " +
			QString::number(this->minZ, 'g', 20) + " " +

			QString::number(this->maxX, 'g', 20) + " " +
			QString::number(this->maxY, 'g', 20) + " " +
			QString::number(this->maxZ, 'g', 20);
}


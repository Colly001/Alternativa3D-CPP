#include "uvmatrixprovider.h"

UVMatrixProvider::UVMatrixProvider()
{

}

double *UVMatrixProvider::getValues()
{
	Matrix *matrix = this->getMatrix();
	this->matrixValues[0] = matrix->a;
	this->matrixValues[1] = matrix->b;
	this->matrixValues[2] = matrix->tx;
	this->matrixValues[3] = 0;
	this->matrixValues[4] = matrix->c;
	this->matrixValues[5] = matrix->d;
	this->matrixValues[6] = matrix->ty;
	this->matrixValues[7] = 0;
	return this->matrixValues;
}

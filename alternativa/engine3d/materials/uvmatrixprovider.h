#ifndef UVMATRIXPROVIDER_H
#define UVMATRIXPROVIDER_H

#include "flash/geom/matrix.h"

class UVMatrixProvider
{
	double matrixValues[8];
	Matrix matrix;

public:
	UVMatrixProvider();

	Matrix* getMatrix()
	{
		return &this->matrix;
	}

	double* getValues();
};

#endif // UVMATRIXPROVIDER_H

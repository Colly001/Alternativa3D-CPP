#include "matrix3d.h"

//4x4

const double* Matrix3D::rawData() const
{
	return data;
}

double* Matrix3D::rawData()
{
	return data;
}

void Matrix3D::transformVectors_1(const double in[3], double out[3])
{
	out[0] = (this->data[0] * in[0]) + (this->data[4] * in[1]) + (this->data[8] * in[2]) + this->data[12];
	out[1] = (this->data[1] * in[0]) + (this->data[5] * in[1]) + (this->data[9] * in[2]) + this->data[13];
	out[2] = (this->data[2] * in[0]) + (this->data[6] * in[1]) + (this->data[10] * in[2]) + this->data[14];
}


void Matrix3D::transformVectors_2(const double in[6], double out[6])
{
	out[0] = (this->data[0] * in[0]) + (this->data[4] * in[1]) + (this->data[8] * in[2]) + this->data[12];
	out[1] = (this->data[1] * in[0]) + (this->data[5] * in[1]) + (this->data[9] * in[2]) + this->data[13];
	out[2] = (this->data[2] * in[0]) + (this->data[6] * in[1]) + (this->data[10] * in[2]) + this->data[14];

	out[3] = (this->data[0] * in[3]) + (this->data[4] * in[4]) + (this->data[7] * in[5]) + this->data[12];
	out[4] = (this->data[1] * in[3]) + (this->data[5] * in[4]) + (this->data[8] * in[5]) + this->data[13];
	out[5] = (this->data[2] * in[3]) + (this->data[6] * in[4]) + (this->data[9] * in[5]) + this->data[14];
}

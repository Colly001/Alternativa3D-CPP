#ifndef MATRIX3D_H
#define MATRIX3D_H


class Matrix3D
{
	double data[16];
public:
	void set(double a1,
			 double a2,
			 double a3,
			 double a4,
			 double a5,
			 double a6,
			 double a7,
			 double a8,
			 double a9,
			 double a10,
			 double a11,
			 double a12,
			 double a13,
			 double a14,
			 double a15,
			 double a16)
	{
		data[0] = a1;
		data[1] = a2;
		data[2] = a3;
		data[3] = a4;

		data[4] = a5;
		data[5] = a6;
		data[6] = a7;
		data[7] = a8;

		data[8] = a9;
		data[9] = a10;
		data[10] = a11;
		data[11] = a12;

		data[12] = a13;
		data[13] = a14;
		data[14] = a15;
		data[15] = a16;
	}

	const double* rawData() const;
	double* rawData();
	void transformVectors_1(const double in[3], double out[3]);
	void transformVectors_2(const double in[6], double out[6]);
};

#endif // MATRIX3D_H

#ifndef VECTOR3_H
#define VECTOR3_H

class Matrix4;
class Matrix3;
class QString;


class Vector3
{

public:
	double x;
	double y;
	double z;

	static const Vector3 ZERO;
	static const Vector3 X_AXIS;
	static const Vector3 Y_AXIS;
	static const Vector3 Z_AXIS;
	static const Vector3 DOWN;

	Vector3(const Vector3&) = delete;
	void operator=(const Vector3&) = delete;

	Vector3() {}
	Vector3(double X, double Y, double Z);
	void reverse();
	double dot(const Vector3*) const;
	void reset(double = 0, double = 0, double = 0);
	double lengthSqr() const;
	double length() const;
	void setLength(double);
	void copy(const Vector3*);
	void cross2(const Vector3*, const Vector3*);
	void normalize();
	bool isFiniteVector() const;
	void scale(double);
	void setLengthAlongDirection(const Vector3*, double);
	void add(const Vector3*);
	void remove(const Vector3 *vector3);
	void cross(const Vector3*);
	void transform4(const Matrix4*);
	void diff(const Vector3*, const Vector3*);
	void subtract(const Vector3*);
	double distanceTo(const Vector3*) const;
	double distanceToSquared(const Vector3*) const;
	void transformTransposed3(const Matrix3*);
	void addScaled(double, const Vector3*);
	void deltaTransform4(const Matrix4*);
	void sum(const Vector3*, const Vector3*);
	void transform3(const Matrix3*);
	void transformBy3(const Matrix3 *m);
	void transformBy3Tr(const Matrix3 *m);
	void transformBy4(const Matrix4 *m);
	void interpolate(double, const Vector3*, const Vector3*);
	double cosAngle(const Vector3*) const;
	static double distanceBetween(const Vector3 *a, const Vector3 *b);
	QString toString() const;
};

#endif // VECTOR3_H

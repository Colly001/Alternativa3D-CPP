#ifndef QUATERNION_H
#define QUATERNION_H

class Vector3;
class Matrix3;
class Matrix4;
class QString;

class Quaternion
{
public:
	double w, x, y, z;
	void getZAxis(Vector3*) const;
	void getYAxis(Vector3*) const;
	void toMatrix3(Matrix3*) const;
	void addScaledVector(const Vector3*, double);
	void copy(const Quaternion*);
	bool isFiniteQuaternion() const;
	void setFromEulerAnglesXYZ(double x, double y, double z);
	void normalize();
	void toMatrix4(Matrix4*) const;
	void getEulerAngles(Vector3*) const;
	void setFromAxisAngleComponents(double, double, double, double);
	void setFromAxisAngle(const Vector3 *axis, double angle);
	void append(const Quaternion*);
	void setFromEulerAngles(const Vector3*);
	void slerp(const Quaternion*, const Quaternion*, double);
	QString toString() const;

	void reset(double w = 1, double x = 0, double y = 0, double z = 0)
	{
		this->w = w;
		this->x = x;
		this->y = y;
		this->z = z;
	}
};

#endif // QUATERNION_H

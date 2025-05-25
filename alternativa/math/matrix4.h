#ifndef MATRIX4_H
#define MATRIX4_H

class Matrix3;
class Vector3;
class QString;

class Matrix4
{
public:

    union {
        struct {
            double m00;
            double m01;
            double m02;
            double m03;

            double m10;
            double m11;
            double m12;
            double m13;

            double m20;
            double m21;
            double m22;
            double m23;
        };

        struct {
            double ma;
            double mb;
            double mc;
            double md;

            double me;
            double mf;
            double mg;
            double mh;

            double mi;
            double mj;
            double mk;
            double ml;
        };

		struct {
			double a;
			double b;
			double c;
			double d;

			double e;
			double f;
			double g;
			double h;

			double i;
			double j;
			double k;
			double l;
		};

		struct {
			double x_x;
			double x_y;
			double x_z;
			double x_w;

			double y_x;
			double y_y;
			double y_z;
			double y_w;

			double z_x;
			double z_y;
			double z_z;
			double z_w;
		};
    };

    void toIdentity();
	void transformVector(const Vector3*, Vector3*) const;
	void setPosition(const Vector3*);
	void getPosition(Vector3*) const;
	void fromAxisAngle(const Vector3*, double);
	void getAxis(int, Vector3*) const;
	void transformVectorInverse(Vector3*, Vector3*) const;
    void setMatrix(double x, double y, double z, double rotationX, double rotationY, double rotationZ);
	void setMatrix(const Vector3 *position, const Vector3 *rotation);
	void prepend(Matrix4*);
	void append(const Matrix4*);
	void getEulerAngles(Vector3*) const;
	void transformVectorsN(const Vector3 **arrin, Vector3 **arrout, int len) const;
	void transformVectorsInverseN(const Vector3 **arrin, Vector3 **arrout, int len) const;
	void invert();
	void setFromMatrix3(const Matrix3*, const Vector3*);
	void copy(const Matrix4 *m);
	void setRotationMatrix(double, double, double);
	QString toString() const;
	void init(double v1, double v2, double v3, double v4, double v5, double v6, double v7, double v8, double v9, double v10, double v11, double v12);
};

#endif // MATRIX4_H

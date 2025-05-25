#ifndef BOUNDBOX_H
#define BOUNDBOX_H

class QString;

class BoundBox
{
public:

	union
	{
		struct
		{
			double minX;
			double minY;
			double minZ;
			double maxX;
			double maxY;
			double maxZ;
		};

		struct
		{
			double MinX;
			double MinY;
			double MinZ;
			double MaxX;
			double MaxY;
			double MaxZ;
		};
	};

	BoundBox() {}
	BoundBox(const BoundBox&) = delete;
	void operator=(const BoundBox&) = delete;

	void setSize(double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
	void addBoundBox(const BoundBox *boundBox);
	void addPoint(double x, double y, double z);
	void infinity();
	bool intersects(const BoundBox *bb, double epsilon) const;
	void copyFrom(const BoundBox *boundBox);
	QString toString() const;
};

#endif // BOUNDBOX_H

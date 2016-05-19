#ifndef DJI_EDGE_POLYGON_EGDER_H
#define DJI_EDGE_POLYGON_EGDER_H
#include "djiPoint.h"

#define DJI_EDGE_FARMLAND 0

#define DJI_EDGE_NSZ_AROUND 1

#define DJI_EDGE_NSZ_OVER 2

#define DJI_EDGE_NSZ_HEIGHT 3

#define DJI_EDGE_NSZ_AROUND_OUTERSIDE 4

#define DJI_LINE_CHARGING 5

#define RESIZE_TYPE_INFLATE 0
#define RESIZE_TYPE_DEFLATE 1


class DjiRect
{
public:
	double minX;
	double maxX;
	double minY;
	double maxY;
	DjiRect()
	{
		maxY=minY = maxX = minX = 0.0;
	}
	DjiRect(double _minX, double _maxX, double _minY, double _maxY)
	{
		minX = _minX;
		maxX = _maxX;
		minY = _minY;
		maxY = _maxY;
	}
};
class DjiEdge
{
public:
	vector<DjiPoint> edge;

	DjiRect outerRect;
	//Minimum enclosing rectangle
	//DjiPoint outerRect[4];

	//Maxmum inscribed rectangle
	//DjiPoint innerRect[4];

	bool _isUpdateRect;
public:
	DjiEdge();
	DjiEdge(vector<DjiPoint> edge);

	void calOuterRect();

	bool isUpdateRect() const
	{
		return _isUpdateRect;
	}

	bool isOutsidePolygonRect(DjiPoint pt) const
	{
		return !(pt.x >= outerRect.minX && pt.x<=outerRect.maxX 
			&& pt.y >= outerRect.minY && pt.y <= outerRect.maxY);
	}

	bool isOutsidePolygonRect(DjiLineSeq seq) const
	{
		return isOutsidePolygonRect(seq.s) && isOutsidePolygonRect(seq.e);
	}

	void push_back(DjiPoint pt);

	/*edge points anti-clockwise
	*/
	void inflate(double offset);

	/*edge points anti-clockwise
	*/
	void deflate(double offset);

	unsigned size() const;


	DjiPoint& operator[](const unsigned int index);

	const DjiPoint& operator[](const unsigned int index) const;

private:
	void resize(double offset, int type);

};

#endif // !DJI_EDGE_POLYGON

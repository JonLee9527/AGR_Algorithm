#include "stdafx.h"

#include "djiPoint.h"


DjiPoint::DjiPoint()
{
	x = 0.0;
	y = 0.0;
	s = 0;
	h = 0.0;
}

DjiPoint::DjiPoint(const DjiPoint& p)
{
	x = p.x;
	y = p.y;
	s = p.s;
	h = p.h;
}

DjiPoint::DjiPoint(const DjiPoint& p, int _state)
{
	x = p.x;
	y = p.y;
	s = _state;

	h = p.h;
}
DjiPoint::DjiPoint(double _x, double _y)
{
	x = _x;
	y = _y;

	s = 0;
	h = 0.0;
}

// DjiPoint::DjiPoint(double _x, double _y, double _h, int _s)
// {
// 	x = _x;
// 	y = _y;
// 	h = _h;
// 
// 	s = _s;
// 	
// }
DjiPoint::DjiPoint(double _x, double _y,int _state)
{
	x = _x;
	y = _y;
	s = _state;

	h = 0.0;
}



DjiPoint DjiPoint::operator+(const DjiPoint &pt)
{
	return DjiPoint(x + pt.x, y + pt.y);
}

DjiPoint DjiPoint::operator-(const DjiPoint &pt)
{
	return DjiPoint(x - pt.x, y - pt.y);
}
DjiPoint DjiPoint::operator * (const double s)
{
	return DjiPoint(x*s, y*s);
}
DjiPoint DjiPoint::operator / (const double s)
{
	return DjiPoint(x/s, y/s);
}

double DjiPoint::dot(const DjiPoint &pt) const
{
	return x*pt.x + y*pt.y;
}

double cross(const DjiVec &vec1, const DjiVec &vec2)
{
	return vec1.x*vec2.y - vec2.x*vec1.y;
}
double DjiPoint::cross2(const DjiPoint &vec) const
{
	return x*vec.y - vec.x*y;
}

bool DjiPoint::xyEqual(DjiPoint pt)
{
	return abs(this->x - pt.x) < ZERO_ABS  &&  abs(this->y - pt.y) < ZERO_ABS;
}
bool DjiPoint::operator == (const DjiPoint &pt)
{
	return x == pt.x && y == pt.y && s == pt.s;
}

//operator
DjiPoint operator* (double s, const DjiPoint &pt)
{
	return DjiPoint(s*pt.x, s*pt.y);
}

DjiPoint operator- (const DjiPoint &pt)
{
	return DjiPoint(-pt.x, -pt.y);
}

DjiPoint operator - (const DjiPoint &pt1, const DjiPoint &pt2)
{
	return DjiPoint(pt1.x-pt2.x, pt1.y-pt2.y);
}
DjiPoint operator + (const DjiPoint &pt1, const DjiPoint &pt2)
{
	return DjiPoint(pt1.x + pt2.x, pt1.y + pt2.y);
}



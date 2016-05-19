#ifndef DJIPOINT_2DIMENSION_DOUBLE
#define DJIPOINT_2DIMENSION_DOUBLE

#include<vector>
#include "djiConst.h"

using namespace std;

#define POINT_NO_SPRAY  0
#define POINT_SPRAY  1
#define POINT_CHANGE_BATTERY  2

class DjiPoint
{
public:
	double x;        //latitude
	double y;        //longitude
	double h;  //height

	int s;           //sign to defferent uses

public:
	DjiPoint();
	DjiPoint(double _x, double _y);
	DjiPoint(double _x, double _y, int _state);
	//DjiPoint(double _x, double _y, double _h,int _state);
	DjiPoint(const DjiPoint& p);
	DjiPoint(const DjiPoint& p, int _state);

	DjiPoint operator + (const DjiPoint &pt);
	DjiPoint operator - (const DjiPoint &pt);
	DjiPoint operator * (const double s);
	DjiPoint operator / (const double s);
	bool operator == (const DjiPoint &pt);

	double dot(const DjiPoint &pt) const;
	double cross2(const DjiPoint &vec) const;
	bool xyEqual(DjiPoint pt);
};
typedef DjiPoint   DjiVec;

//operator
DjiPoint operator* (double s, const DjiPoint &pt);
DjiPoint operator- (const DjiPoint &pt);
DjiPoint operator - (const DjiPoint &pt1, const DjiPoint &pt2);
DjiPoint operator + (const DjiPoint &pt1, const DjiPoint &pt2);


/*
if a£¨x1£¬y1£©£¬b£¨x2£¬y2£©£¬a¡Áb=£¨x1y2-x2y1£©
return>0 vec2 on the anti-clockwise direction of vec1 (define  from X axis to Y axis is anti-clockwise)
return<0 vec2 on the clockwise direction of vec1
return=0 vec2 and vec1 one same line
*/
double djiCross(const DjiVec &vec1, const DjiVec &vec2);

class DjiLine
{
public:
	DjiPoint pt;
	DjiVec vec;
	DjiLine(DjiPoint _pt, DjiVec _vec)
	{
		pt = _pt;
		vec = _vec;
	}

};

static double pointDistance(DjiPoint start, DjiPoint end)
{
	DjiVec vec = end - start;
	return sqrt(vec.dot(vec));
}


class DjiLineSeq
{
public:
	DjiPoint s;
	DjiPoint e;

public:
	DjiLineSeq() { }
	DjiLineSeq(DjiPoint _s, DjiPoint _e)
	{
		s = _s;
		e = _e;
	}

	DjiLineSeq(const DjiLineSeq& _seq)
	{ 
		s = _seq.s;   e = _seq.e; 
	}
	DjiLineSeq(vector<DjiPoint> _seq)
	{ 
		s = _seq[0];   e = _seq[1]; 
	}

	void reverse()
	{
		DjiPoint tem = s;
		s = e;
		e = tem;
	}

	double length()
	{
		return pointDistance(s, e);
	}

	
};


/*@brief legal detection of latitude_longtitude
*/
static bool isLegalLatLong(DjiPoint latLong)
{
	return latLong.x >= -90 && latLong.x <= 90 && latLong.y >= -180 && latLong.y <= 180;
}

static double molMat(const DjiVec vec)
{
	return sqrt(vec.dot(vec));
}


static bool isSamePoint(const DjiPoint& pt1, const DjiPoint& pt2)
{
	return molMat(pt1 - pt2) < ZERO_ABS;
}

static bool isOnLineseq(const DjiPoint& pt0, const DjiPoint& pt1, const DjiPoint& testPt)
{
	return abs(molMat(testPt - pt0) + molMat(pt1 - testPt) - molMat(pt1 - pt0)) < ZERO_ABS;
}

#endif // !DJIPOINT_2DIMENSION_DOUBLE

#include "stdafx.h"
#include "edgeGenaration.h"
#include "math.h"
#include "PPETask.h"

#include "djiUtils.h"

void calOneDirection(const vector<DjiPoint>& _polylinePts, double _radius, vector<DjiPoint>& resPts)
{
	DjiVec forwardVec;
	DjiVec postive90;
	DjiVec vec1;
	DjiVec vec2;
	DjiVec offset;
	double sita;

	for (unsigned i = 0; i < _polylinePts.size(); i++)
	{
		if (i == 0)
		{
			forwardVec = _polylinePts[1] - _polylinePts[0];
			forwardVec = forwardVec / molMat(forwardVec);
			postive90.x = forwardVec.y;
			postive90.y = -forwardVec.x;
			resPts.push_back(_polylinePts[0] - forwardVec*_radius / cos(PI / 6));
			resPts.push_back(_polylinePts[0] + postive90*_radius);
		}
		else if (i == _polylinePts.size() - 1)
		{
			forwardVec = _polylinePts[_polylinePts.size() - 1] - _polylinePts[_polylinePts.size() - 2];
			forwardVec = forwardVec / molMat(forwardVec);
			postive90.x = forwardVec.y;
			postive90.y = -forwardVec.x;
			resPts.push_back(_polylinePts[_polylinePts.size() - 1] + postive90*_radius);
		}
		else
		{
			forwardVec = _polylinePts[i] - _polylinePts[i - 1];
			forwardVec = forwardVec / molMat(forwardVec);
			postive90.x = forwardVec.y;
			postive90.y = -forwardVec.x;

			vec1 = _polylinePts[i - 1] - _polylinePts[i];
			vec1 = vec1 / molMat(vec1);
			vec2 = _polylinePts[i + 1] - _polylinePts[i];
			vec2 = vec2 / molMat(vec2);
			sita = acos(vec1.dot(vec2));
			offset = vec1 + vec2;
			offset = offset / molMat(offset);

			if (offset.dot(postive90) < 0)
			{
				offset = -offset;
			}
			resPts.push_back(_polylinePts[i] + offset*_radius / sin(sita / 2));
		}
	}

}
vector<DjiPoint> initWithSiglePoint(DjiPoint p, double radius)
{ 
	vector<DjiPoint> edge = vector<DjiPoint>(6);
	double x = p.x;
	double y = p.y;
	edge[0] = DjiPoint(x - tan(PI / 6)*radius, y + radius);
	edge[1] = DjiPoint(x + tan(PI / 6)*radius, y + radius);
	edge[2] = DjiPoint(x + radius / cos(PI / 6), y);
	edge[3] = DjiPoint(x + tan(PI / 6)*radius, y - radius);
	edge[4] = DjiPoint(x - tan(PI / 6)*radius, y - radius);
	edge[5] = DjiPoint(x - radius / cos(PI / 6), y);

	return edge;
}
vector<DjiPoint> getEdge(DjiPoint p, double radius)
{
	CoordTransform tran = CoordTransform(p);
	DjiPoint pt_p = tran.latlong2plane(p);
	vector<DjiPoint> pts = initWithSiglePoint(pt_p, radius);
	return tran.plane2latlong(pts);
}
vector<DjiPoint> getEdge(vector<DjiPoint> polylinePts, double radius)
{
	//data pre handle
	deleteCollinearPoint(polylinePts);
	if (polylinePts.size() < 1)
		return vector<DjiPoint>();
	if (polylinePts.size() == 1)
	{
		return getEdge(polylinePts[0], radius);
	}
	
	//transform to plane coordinate
	CoordTransform tran = CoordTransform(polylinePts[0]);
	polylinePts = tran.latlong2plane(polylinePts);

	vector<DjiPoint> resPts;
	calOneDirection(polylinePts, radius, resPts);
	
	vector<DjiPoint> temp(polylinePts.size());
	for (int i = 0; i < (int)temp.size(); i++)
	{
		temp[i] = polylinePts[polylinePts.size() - 1 - i];
	}

	calOneDirection(temp, radius, resPts);
	return tran.plane2latlong(resPts);
}
#include "stdafx.h"

#include "DjiEdge.h"

#include "djiUtils.h"

#include "debugIO.h"

DjiEdge::DjiEdge()
{
	edge = vector<DjiPoint>();
	_isUpdateRect = false;
}

DjiEdge::DjiEdge(vector<DjiPoint> _edge)
{
	edge = _edge;
	_isUpdateRect = false;
}

unsigned DjiEdge::size() const
{
	return edge.size();
}

void DjiEdge::push_back(DjiPoint pt)
{
	edge.push_back(pt);
	_isUpdateRect = false;
}

void DjiEdge::calOuterRect()
{
	if (this->size() == 0)
		return;

	double& minX = outerRect.minX;
	double& maxX = outerRect.maxX;

	double& minY = outerRect.minY;
	double& maxY = outerRect.maxY;

	minX = maxX = edge[0].x;

	minY = maxY = edge[0].y;

	for (int i = 0; i < (int)edge.size(); i++)
	{
		minX = edge[i].x<minX ? edge[i].x : minX;
		maxX = edge[i].x>maxX ? edge[i].x : maxX;

		minY = edge[i].y < minY ? edge[i].y : minY;
		maxY = edge[i].y>maxY ? edge[i].y : maxY;
	}
	_isUpdateRect = true;
}

DjiPoint& DjiEdge::operator[](const unsigned int index)
{
	return edge[index];

}

const DjiPoint& DjiEdge::operator[](const unsigned int index) const
{
	return edge[index];

}

void DjiEdge::resize(double offset, int type)
{
	if (type==RESIZE_TYPE_DEFLATE)
		offset = -offset;

	vector<DjiPoint> edgeTem;

	DjiVec vec1;//pts[i] - pts[i - 1];
	DjiVec vec2;//pts[i + 1] - pts[i];
	DjiVec clockwise90;//vec2 rotate  clockwise 90 degree
	DjiVec offsetVec;//vec1 + vec2;
	double sita;//angle between vec1 and vec2

	for (unsigned i = 0; i < edge.size(); i++)
	{
		vec1 = edge[i] - edge[(i - 1 + edge.size()) % edge.size()];
		vec1 = -vec1 / molMat(vec1);

		vec2 = edge[(i + 1) % edge.size()] - edge[i];
		vec2 = vec2 / molMat(vec2);

		clockwise90.x = vec2.y;
		clockwise90.y = -vec2.x;

		sita = acos(vec1.dot(vec2));
		offsetVec = vec1 + vec2;
		offsetVec = offsetVec / molMat(offsetVec);

		// the angle between -vec1 and vec2 less than 180 degree
		offsetVec=offsetVec.dot(clockwise90) < 0 ? -offsetVec : offsetVec;
		edgeTem.push_back(edge[i] + offsetVec*offset / sin(sita / 2));
	}

	edge = edgeTem;

}

void DjiEdge::inflate(double offset)
{
	if (!isAntiClockwise(edge))
		printLog("polygon is not anti-clockwise", true);

	resize(offset, RESIZE_TYPE_INFLATE);

	_isUpdateRect = false;
}

void DjiEdge::deflate(double offset)
{
	if (!isAntiClockwise(edge))
		printLog("polygon is not anti-clockwise", true);

	resize(offset, RESIZE_TYPE_DEFLATE);

	_isUpdateRect = false;
}


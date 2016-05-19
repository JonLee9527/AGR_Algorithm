#include "stdafx.h"

#include "djiUtils.h"

#include "debugIO.h"

#include "geometry.h"

#include "DjiEdge.h"

void deleteClosedPoint(vector<DjiPoint>& pts, double minDist)
{
	if (pts.size() < 2)
		return;
	vector<DjiPoint> tem;
	tem.push_back(pts[0]);
	DjiPoint last = pts[0];

	for (int i = 1; i < (int)pts.size(); i++)
	{
		if (pointDistance(pts[i], last)>minDist)
		{
			tem.push_back(pts[i]);
			last = pts[i];
		}
	}
	pts = tem;
}

void deleteCollinearPoint(vector<DjiPoint>& pts)
{
	vector<DjiPoint>  _pts = pts;
	vector<bool> flags(pts.size(), 1);
	bool isSameLine = false;
	for (int i = 0; i < (int)pts.size(); i++)
	{
		if (isOnLineseq(pts[(i - 1 + pts.size()) % pts.size()], pts[(i + 1) % pts.size()], pts[i]))
		{
			flags[i] = 0;
			isSameLine = true;
		}
	}
	if (isSameLine)
	{
		pts.clear();
		for (int i = 0; i < (int)_pts.size(); i++)
		{
			if (flags[i])
			{
				pts.push_back(_pts[i]);
			}
		}
	}
}

bool isIntersectLineseq(DjiLineSeq first, DjiLineSeq second)
{
	return intersect_A(first, second);
}

bool isIntersection(DjiLineSeq lineseq, const DjiEdge& edge)
{
	if (!edge.isUpdateRect())
		printLog("!edge.isUpdateRect(), in isIntersection");
	if (edge.isOutsidePolygonRect(lineseq))
		return false;
	for (int i = 0; i < (int)edge.size(); i++)
	{
		if (isIntersectLineseq(lineseq, DjiLineSeq(edge[i], edge[(i + 1) % edge.size()])))
			return true;
	}
	return false;
}

bool isIntersection(const DjiEdge& edge1, const DjiEdge& edge2)
{
	if (!(edge1.isUpdateRect() && edge2.isUpdateRect()))
		printLog("!edge.isUpdateRect(), in isIntersection",true);

	//排斥试验
	DjiRect outerRect1 = edge1.outerRect;
	DjiRect outerRect2 = edge2.outerRect;
	if (outerRect1.minX>outerRect2.maxX || outerRect1.maxX<outerRect2.minX
		|| outerRect1.minY>outerRect2.maxY || outerRect1.maxY<outerRect2.minY)
	{
		return false;
	}

	for (int i = 0; i < (int)edge1.size(); i++)
	{
		for (int j = 0; j < (int)edge2.size(); j++)
		{
			if (isIntersectLineseq(DjiLineSeq(edge1[i], edge1[(i + 1) % edge1.size()]),
				DjiLineSeq(edge2[j], edge2[(j + 1) % edge2.size()])))
				return true;
		}
	}
	return false;
}

bool isSelfIntersect(const vector<DjiPoint>& pts)
{
	if (pts.size() <= 3)
		return false;
	POINT* _pts = new POINT[pts.size()];
	for (int i = 0; i < (int)pts.size(); i++)
	{
		_pts[i] = pts[i];
	}

	bool isSim= issimple(pts.size(),_pts);

	delete[] _pts;
	return !isSim;

	///////*****************my code**********//////


// 	vector<DjiLineSeq> seqs;
// 	for (int i = 0; i < pts.size();i++)
// 	{
// 		seqs.push_back(DjiLineSeq(pts[i], pts[(i + 1) % pts.size()]));
// 	}
// 	for (int i = 0; i < seqs.size(); i++)
// 	{
// 		for (int j = i+1; j < seqs.size();j++)
// 		{
// 			if (isIntersectLineseq(seqs[i], seqs[j]))
// 				return true;
// 		}
// 	}
// 	return false;

}
bool isAntiClockwise(const vector<DjiPoint>& pts)
{
	if (pts.size() <3)
		return false;
	POINT* _pts = new POINT[pts.size()];
	for (int i = 0; i < (int)pts.size(); i++)
	{
		_pts[i] = pts[i];
	}

	bool isSim = isconterclock(pts.size(), _pts);
	//bool isSim = isccwize(pts.size(), _pts);

	delete[] _pts;
	return isSim;

}

vector<DjiPoint> connectPoints(vector<DjiPoint> head, vector<DjiPoint> tail)
{
	head.insert(head.end(), tail.begin(), tail.end());
	return head;
}

vector<DjiPoint> connectPoints(DjiPoint headPt, vector<DjiPoint> tail)
{
	vector<DjiPoint> head;
	head.push_back(headPt);
	head.insert(head.end(), tail.begin(), tail.end());
	return head;
}

vector<DjiPoint> reversePoints(const vector<DjiPoint>& pts)
{
	vector<DjiPoint> res = vector<DjiPoint>(pts.size());
	for (int i = 0; i < (int)res.size(); i++)
	{
		res[i] = pts[pts.size() - 1 - i];
	}
	return res;
}

vector<DjiPoint> getRange(const vector<DjiPoint>& pts, int start, int end)
{
	vector<DjiPoint> res;
	for (int i = start; i <= end && i < (int)pts.size(); i++)
	{
		res.push_back(pts[i]);
	}
	return res;
}

double calPolygonArea(const vector<DjiPoint> &points)
{
	int point_num = points.size();
	if (point_num < 3)return 0.0;
	double s = points[0].y * (points[point_num - 1].x - points[1].x);
	for (int i = 1; i < point_num; ++i)
		s += points[i].y * (points[i - 1].x - points[(i + 1) % point_num].x);
	return fabs(s / 2.0);
}


double calPolylineDist(const vector<DjiPoint>& pts)
{
	double pathCost = 0;
	for (int i = 0; i < (int)pts.size() - 1; i++)
	{
		pathCost += pointDistance(pts[i], pts[i + 1]);
	}
	return pathCost;
}


void sortWithVec(vector<DjiPoint>& interSec, DjiPoint forwardVec)
{
	double* a = new double[interSec.size()];
	for (int i = 0; i < (int)interSec.size(); i++)
	{
		a[i] = interSec[i].dot(forwardVec);
	}

	int i, j;
	double key;
	DjiPoint keyPoint;

	for (i = 1; i< (int)interSec.size(); i++)
	{
		key = a[i];
		keyPoint = interSec[i];

		for (j = i; j>0 && a[j - 1] > key; j--)
		{
			a[j] = a[j - 1];
			interSec[j] = interSec[j - 1];
		}
		a[j] = key;
		interSec[j] = keyPoint;
	}
	delete[] a;
}

void getMinMaxPt(vector<DjiPoint> farmEdge, DjiVec _forwardVec, DjiPoint &minPt, DjiPoint & maxPt)
{
	//forwardVec=(n1,n2)   直线方程n1*y-n2*x=0，所有区域标记点带入直线方程求得最大值和最小值
	//与forward垂直的向量为sideVec（n2,-n1)
	int minPtIndex = 0;
	int maxPtIndex = 0;
	double minTemp = _forwardVec.x*farmEdge[0].y - _forwardVec.y*farmEdge[0].x;
	double maxTemp = minTemp;
	for (int i = 1; i < (int)farmEdge.size(); i++)
	{
		if (minTemp>_forwardVec.x*farmEdge[i].y - _forwardVec.y*farmEdge[i].x)
		{
			minPtIndex = i;
			minTemp = _forwardVec.x*farmEdge[i].y - _forwardVec.y*farmEdge[i].x;
		}
		if (maxTemp < _forwardVec.x*farmEdge[i].y - _forwardVec.y*farmEdge[i].x)
		{
			maxPtIndex = i;
			maxTemp = _forwardVec.x*farmEdge[i].y - _forwardVec.y*farmEdge[i].x;
		}
	}
	//离home点近的定义为minPt
	minPt = farmEdge[minPtIndex];
	maxPt = farmEdge[maxPtIndex];
}

vector<DjiPoint> dispersePolyline(vector<DjiPoint>polyline, double pieceLength)
{
	if (pieceLength < 0 || polyline.size() < 2)
	{
		printLog("input error in dividLineSeq", true);
	}

	vector<DjiPoint> pieceLine;
	DjiVec normal;
	double length = 0;
	for (int i = 1; i < (int)polyline.size(); i++)
	{
		length = pointDistance(polyline[i - 1], polyline[i]);
		normal = polyline[i] - polyline[i - 1];
		normal = normal / molMat(normal);
		for (int j = 0; j*pieceLength < length; j++)
		{
			pieceLine.push_back(polyline[i - 1] + j*pieceLength*normal);
		}
	}
	pieceLine.push_back(polyline[polyline.size() - 1]);
	return pieceLine;
}

vector<DjiPoint> disperseFlightLineseq(DjiPoint pStart, DjiPoint pEnd, double pieceLength)
{
	if (pieceLength < 0 || (pStart.x == pEnd.x&&pStart.y == pEnd.y))
	{
		printLog("input error in dividLineSeq", true);
	}
	vector<DjiPoint> pieceLine;
	double length = pointDistance(pStart, pEnd);
	DjiVec normal = pEnd - pStart;
	normal = normal / molMat(normal);

	for (int j = 0; j*pieceLength < length; j++)
	{
		pieceLine.push_back(pStart + j*pieceLength*normal);
	}
	pieceLine.push_back(pEnd);
	return pieceLine;
}


static double computeDistance(DjiPoint startlatLong, DjiPoint endLatLong)
{

	double startLatitude = startlatLong.x;
	double startLongitude = startlatLong.y;
	double endLatitude = endLatLong.x;
	double endLongitude = endLatLong.y;

	int MAXITERS = 20;
	// Convert lat/long to radians
	startLatitude *= PI / 180.0;
	endLatitude *= PI / 180.0;
	startLongitude *= PI / 180.0;
	endLongitude *= PI / 180.0;

	double a = 6378137.0; // WGS84 major axis
	double b = 6356752.3142; // WGS84 semi-major axis
	double f = (a - b) / a;
	double aSqMinusBSqOverBSq = (a * a - b * b) / (b * b);

	double L = endLongitude - startLongitude;
	double A = 0.0;
	double U1 = atan((1.0 - f) * tan(startLatitude));
	double U2 = atan((1.0 - f) * tan(endLatitude));

	double cosU1 = cos(U1);
	double cosU2 = cos(U2);
	double sinU1 = sin(U1);
	double sinU2 = sin(U2);
	double cosU1cosU2 = cosU1 * cosU2;
	double sinU1sinU2 = sinU1 * sinU2;

	double sigma = 0.0;
	double deltaSigma = 0.0;
	double cosSqAlpha = 0.0;
	double cos2SM = 0.0;
	double cosSigma = 0.0;
	double sinSigma = 0.0;
	double cosLambda = 0.0;
	double sinLambda = 0.0;

	double lambda = L; // initial guess
	for (int iter = 0; iter < MAXITERS; iter++) {
		double lambdaOrig = lambda;
		cosLambda = cos(lambda);
		sinLambda = sin(lambda);
		double t1 = cosU2 * sinLambda;
		double t2 = cosU1 * sinU2 - sinU1 * cosU2 * cosLambda;
		double sinSqSigma = t1 * t1 + t2 * t2; // (14)
		sinSigma = sqrt(sinSqSigma);
		cosSigma = sinU1sinU2 + cosU1cosU2 * cosLambda; // (15)
		sigma = atan2(sinSigma, cosSigma); // (16)
		double sinAlpha = (sinSigma == 0) ? 0.0 :
			cosU1cosU2 * sinLambda / sinSigma; // (17)
		cosSqAlpha = 1.0 - sinAlpha * sinAlpha;
		cos2SM = (cosSqAlpha == 0) ? 0.0 :
			cosSigma - 2.0 * sinU1sinU2 / cosSqAlpha; // (18)

		double uSquared = cosSqAlpha * aSqMinusBSqOverBSq; // defn
		A = 1 + (uSquared / 16384.0) * // (3)
			(4096.0 + uSquared *
			(-768 + uSquared * (320.0 - 175.0 * uSquared)));
		double B = (uSquared / 1024.0) * // (4)
			(256.0 + uSquared *
			(-128.0 + uSquared * (74.0 - 47.0 * uSquared)));
		double C = (f / 16.0) *
			cosSqAlpha *
			(4.0 + f * (4.0 - 3.0 * cosSqAlpha)); // (10)
		double cos2SMSq = cos2SM * cos2SM;
		deltaSigma = B * sinSigma * // (6)
			(cos2SM + (B / 4.0) *
			(cosSigma * (-1.0 + 2.0 * cos2SMSq) -
			(B / 6.0) * cos2SM *
			(-3.0 + 4.0 * sinSigma * sinSigma) *
			(-3.0 + 4.0 * cos2SMSq)));

		lambda = L +
			(1.0 - C) * f * sinAlpha *
			(sigma + C * sinSigma *
			(cos2SM + C * cosSigma *
			(-1.0 + 2.0 * cos2SM * cos2SM))); // (11)

		double delta = (lambda - lambdaOrig) / lambda;
		if (abs(delta) < 1.0e-12) {
			break;
		}
	}
	return (b * A * (sigma - deltaSigma));
}



double findNearestOnLineseq(DjiPoint pos, DjiLineSeq seq, DjiPoint& nearest)
{
	POINT tem;

	double dist=ptolinesegdist(pos, seq, tem);
	nearest = DjiPoint(tem.x, tem.y);
	return dist;

// 	DjiVec vec1 = seq.s - pos;
// 	DjiVec vec2 = seq.e - pos;
// 	DjiVec vecLine = seq.e - seq.s;
// 	DjiVec vecLineNormal = vecLine / molMat(vecLine);
// 
// 	double dist = 0;
// 	double tem1 = vec1.dot(vecLine);
// 	double tem2 = vec2.dot(vecLine);
// 	if (vec1.dot(vecLine)*vec2.dot(vecLine) >= 0)
// 	{
// 		if (pointDistance(pos, seq.s) < pointDistance(pos, seq.e))
// 		{
// 			nearest = seq.s;
// 			dist = pointDistance(pos, seq.s);
// 		}
// 		else
// 		{
// 			nearest = seq.e;
// 			dist = pointDistance(pos, seq.e);
// 		}
// 	}
// 	else
// 	{
// 		nearest = seq.s + abs(vec1.dot(vecLineNormal))*vecLineNormal;
// 		dist = pointDistance(pos, nearest);
// 	}
// 	return dist;
}


bool lineIntersection(DjiLine l1, DjiLine l2, DjiPoint& intersec)
{
	POINT p;
	bool flag=lineintersect(l1, l2, p);
	intersec = DjiPoint(p.x,p.y);
	return flag;
}


string int2string(int a)
{
	char cStr[200];
	sprintf(cStr, "%d", a);
	return cStr;

}


string double2string(double a)
{
	char cStr[200];
	sprintf(cStr, "%lf", a);
	return cStr;
}


void lineInterSectionMarked(const vector<DjiPoint>& edge, const DjiLine line, vector<DjiPoint>& interSec, int markType)
{
	if (edge.size() < 3)
	{
		printLog("Pts.size()<3, in lineInterSectionMarked", true);
	}
	if (!(markType == INTERSEC_MARKED_WITH_POINT_INDEX || markType == INTERSEC_MARKED_WITH_POINT_S))
		printLog("input Error, in lineInterSectionMarked", true);

	DjiPoint pt0 = line.pt;
	DjiVec vec = line.vec;
	//直线方程    ny*(x - x0)-nx*(y - y0)   = 0;
	double x0 = pt0.x;
	double y0 = pt0.y;
	double nx = vec.x;
	double ny = vec.y;

	double x1 = 0;
	double y1 = 0;
	double x2 = 0;
	double y2 = 0;

	double result = 0;
	double res_next = 0;
	double res_back = 0;
	for (int i = 0; i < (int)edge.size(); i++)
	{
		result = ny*(edge[i].x - x0) - nx*(edge[i].y - y0);
		res_next = ny*(edge[(i + 1) % edge.size()].x - x0) - nx*(edge[(i + 1) % edge.size()].y - y0);
		res_back = ny*(edge[(i - 1 + edge.size()) % edge.size()].x - x0)
			- nx*(edge[(i - 1 + edge.size()) % edge.size()].y - y0);

		if (abs(result)<ZERO_ABS)
		{
			if (abs(res_back)>ZERO_ABS && abs(res_next)>ZERO_ABS && res_back*res_next < 0)
			{
				if (markType == INTERSEC_MARKED_WITH_POINT_S)
				{
					interSec.push_back(edge[i]);
				}
				if (markType == INTERSEC_MARKED_WITH_POINT_INDEX)
				{
					interSec.push_back(DjiPoint(edge[i].x, edge[i].y, i));
				}
			}
		}
		else  //第i点不在直线上
		{
			if (abs(res_next) < ZERO_ABS)
				continue;
			//第i点和第i+1点都不在直线上
			if (result*res_next < 0)
			{
				DjiPoint intersec;
				if (!lineIntersection(line, DjiLine(edge[i], edge[(i + 1) % edge.size()] - edge[i]), intersec))
					printLog("no intersection, in lineInterSectionMarked", true);
				if (markType == INTERSEC_MARKED_WITH_POINT_S)
				{

					interSec.push_back(DjiPoint(intersec.x, intersec.y, edge[(i + 1) % edge.size()].s));
				}
				if (markType == INTERSEC_MARKED_WITH_POINT_INDEX)
				{
					interSec.push_back(DjiPoint(intersec.x, intersec.y, (i + 1) % edge.size()));
				}
			}
		}
	}
	//判断
	if (interSec.size() % 2 != 0)
	{
		// 		drawPoint(mapImg, interSec[0], COL_RED, 1);
		// 		drawPoint(mapImg, line.pt, COL_RED, 0);
		// 		drawPoint(mapImg, line.pt+line.vec, COL_RED, 1);
		printLog("interSec.size() % 2 != 0, in intersectLine", true);
	}
}


double findTwoPointsPath(const vector<DjiEdge >& allAroundObsEdgesPointer, DjiPoint ptStart, DjiPoint ptEnd, 
	vector<DjiPoint>& path, bool isKeepStartEndPoint, bool isClearPath)
{
	
	
	if (isSamePoint(ptStart, ptEnd))
	{
		printPoint("ptStart", ptStart);
		printPoint("ptEnd", ptEnd);
		printLog("ptStart is same ptEnd, in findTwoPointsPath");
		return 0;
	}
	vector<vector<DjiPoint> >  pathPtsSeq;

	//求与障碍物边缘交点
	for (int j = 0; j < (int)allAroundObsEdgesPointer.size(); j++)
	{
		addPathWihtAroundEdge(allAroundObsEdgesPointer[j], ptStart, ptEnd, pathPtsSeq);
	}
	//对pathPts 进行排序
	sortPathsWithForwardVec(pathPtsSeq, ptEnd - ptStart);


	vector<DjiPoint> pathWayPts;

	pathWayPts.push_back(ptStart);//起点

	for (int i = 0; i < (int)pathPtsSeq.size(); i++)
	{
		for (int j = 0; j < (int)pathPtsSeq[i].size(); j++)
		{
			pathWayPts.push_back(DjiPoint(pathPtsSeq[i][j].x, pathPtsSeq[i][j].y));
		}
	}
	pathWayPts.push_back(ptEnd);//终点

	if (pathWayPts.size() < 2)
		printLog("pathWayPts.size() < 2, in findPathFromTwoPts", true);
	double costDist= calPolylineDist(pathWayPts);

	if (!isKeepStartEndPoint)
	{
		pathWayPts.pop_back();
		pathWayPts.erase(pathWayPts.begin());
	}

	if (isClearPath)
	{
		path.clear();
	}

	if (pathWayPts.size() > 0)
	{
		path.insert(path.end(), pathWayPts.begin(), pathWayPts.end());
	}

	return costDist;
}


//寻路时求交
void addPathWihtAroundEdge(const DjiEdge& forbidEdge, const DjiPoint& pStart,
	const DjiPoint& pEnd, vector<vector<DjiPoint> >& pathPts)
{
	if (forbidEdge.size() < 3)
	{
		printLog("Pts.size() < 3, in addPathWihtForbidedge", true);
	}

	//判断在同一线段上
	for (int i = 0; i < (int)forbidEdge.size(); i++)
	{
		if (isOnLineseq(forbidEdge[i], forbidEdge[(i + 1) % forbidEdge.size()], pStart)
			&& isOnLineseq(forbidEdge[i], forbidEdge[(i + 1) % forbidEdge.size()], pEnd))
		{
			return;
		}
	}

	//如果与外接多边形无交点 不可能存在交点
	DjiPoint nearestTem;
	findNearestOnLineseq(DjiPoint((forbidEdge.outerRect.minX + forbidEdge.outerRect.maxX) / 2.0,
		(forbidEdge.outerRect.minY + forbidEdge.outerRect.maxY) / 2.0), DjiLineSeq(pStart, pEnd), nearestTem);

	if (forbidEdge.isOutsidePolygonRect(nearestTem))
		return;

	vector<DjiPoint> interSec;
	lineInterSectionMarked(forbidEdge.edge, DjiLine(pStart, pEnd - pStart), interSec, INTERSEC_MARKED_WITH_POINT_INDEX);
	sortWithVec(interSec, pEnd - pStart);

	double posiCost = 0;
	double negCost = 0;

	vector<int> flags(interSec.size(), 0);

	for (int i = 0; i < (int)interSec.size(); i++)
	{
		if (isOnLineseq(pStart, pEnd, interSec[i]))
			flags[i] = 1;
	}
	for (int i = 0; i < (int)flags.size(); i++)
	{
		if (flags[i] == 0)
		{
			if (i % 2 == 0)
			{
				flags[i + 1] = 0;
			}
			else
			{
				flags[i - 1] = 0;
			}
		}
	}

	if (interSec.size() % 2 != 0)
		printLog("interSec.size() % 2 != 0, in calInterSection2", true);

	vector<DjiPoint> interSecTem;
	for (int i = 0; i < (int)flags.size(); i++)
	{
		if (flags[i] == 1)
		{
			interSecTem.push_back(interSec[i]);
		}
	}
	interSec = interSecTem;

	if (interSec.size() == 0)
		return;



	vector<DjiPoint> shortPath;

	DjiPoint temp1;
	DjiPoint temp2;
	DjiPoint aPt;

	for (int i = 0; i < (int)interSec.size(); i = i + 2)
	{
		shortPath.clear();
		posiCost = 0;
		negCost = 0;
		//正向
		for (int j = (interSec[i].s + 1) % forbidEdge.size(); j != interSec[i + 1].s;)
		{
			temp1.x = forbidEdge[j% forbidEdge.size()].x;
			temp1.y = forbidEdge[j% forbidEdge.size()].y;
			temp2.x = forbidEdge[(j - 1 + forbidEdge.size()) % forbidEdge.size()].x;
			temp2.y = forbidEdge[(j - 1 + forbidEdge.size()) % forbidEdge.size()].y;
			posiCost += molMat(temp1 - temp2);
			j++;
			j = j%forbidEdge.size();
		}
		//反向
		for (int j = (interSec[i].s - 1 + forbidEdge.size()) % forbidEdge.size(); j != interSec[i + 1].s;)
		{

			temp1.x = forbidEdge[(j + forbidEdge.size()) % forbidEdge.size()].x;
			temp1.y = forbidEdge[(j + forbidEdge.size()) % forbidEdge.size()].y;
			temp2.x = forbidEdge[(j + 1 + forbidEdge.size()) % forbidEdge.size()].x;
			temp2.y = forbidEdge[(j + 1 + forbidEdge.size()) % forbidEdge.size()].y;
			negCost += molMat(temp1 - temp2);

			j--;
			j = (j + forbidEdge.size()) % forbidEdge.size();
		}
		shortPath.push_back(interSec[i]);
		if (posiCost < negCost)
		{
			for (int j = interSec[i].s; j != interSec[i + 1].s;)
			{
				aPt.x = forbidEdge[j].x;
				aPt.y = forbidEdge[j].y;
				shortPath.push_back(aPt);
				j++;
				j %= forbidEdge.size();
			}
		}
		else
		{
			for (int j = interSec[i].s; j != interSec[i + 1].s;)
			{
				aPt.x = forbidEdge[(j - 1 + forbidEdge.size()) % forbidEdge.size()].x;
				aPt.y = forbidEdge[(j - 1 + forbidEdge.size()) % forbidEdge.size()].y;
				shortPath.push_back(aPt);
				j--;
				j = (j + forbidEdge.size()) % forbidEdge.size();
			}
		}
		shortPath.push_back(interSec[i + 1]);

		if (shortPath.size())
		{
			pathPts.push_back(shortPath);
		}
	}
}

void sortPathsWithForwardVec(vector<vector<DjiPoint> >& pathPts, DjiPoint forwardVec)
{
	double* a = new double[pathPts.size()];
	for (int i = 0; i < (int)pathPts.size(); i++)
	{
		a[i] = pathPts[i][0].dot(forwardVec);
	}

	int i, j;
	double key;
	vector<DjiPoint> matKey;
	for (i = 1; i<(int)pathPts.size(); i++)
	{
		key = a[i];
		matKey = pathPts[i];

		for (j = i; j>0 && a[j - 1] > key; j--)
		{
			a[j] = a[j - 1];
			pathPts[j] = pathPts[j - 1];

		}
		a[j] = key;
		pathPts[j] = matKey;
	}
	delete[] a;
}


void adjustAntiClockWise(vector<DjiEdge *>  edges)
{
	for (int i = 0; i < (int)edges.size(); i++)
	{
		adjustAntiClockWise(edges[i]);
	}
}

void adjustAntiClockWise(DjiEdge * edge){

	deleteCollinearPoint(edge->edge);
	deleteClosedPoint(edge->edge, MIN_DIST_TO_DELETE);
	if (isSelfIntersect(edge->edge))
		printLog("error isSelfIntersect, in dataPreHandle", true);
	if (!isAntiClockwise(edge->edge))
	{
		edge->edge = reversePoints(edge->edge);
		if (!isAntiClockwise(edge->edge))
		{
			printLog("edge points size= " + int2string(edge->edge.size()));
			for (int j = 0; j < edge->edge.size(); j++)
			{
				printPoint(int2string(j), edge->edge[j]);
			}
			printLog("edge not anti-clockwise or clockwise, in dataPreHandle", true);
		}
	}

}





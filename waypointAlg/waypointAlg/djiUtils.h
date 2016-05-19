#ifndef DJI_WAYPOINT_VECTOR_UTILS_H
#define DJI_WAYPOINT_VECTOR_UTILS_H


#include "djiPoint.h"

#include "DjiEdge.h"

#include <string>


/*
*@brief A closed point will be deleted in distance of minDist
*/
void deleteClosedPoint(vector<DjiPoint>& pts, double minDist);

/*
*@brief If three adjacent points one the same line, the middle one will be deleted
*/
void deleteCollinearPoint(vector<DjiPoint>& pts);

/*
*
*/
bool isSelfIntersect(const vector<DjiPoint>& pts);

/*
*/
bool isAntiClockwise(const vector<DjiPoint>& pts);

/*@brief return =  head + tail
*/
vector<DjiPoint> connectPoints(vector<DjiPoint> head, vector<DjiPoint> tail);
vector<DjiPoint> connectPoints(DjiPoint headPt, vector<DjiPoint> tail);

/*@brief return included start and end
*/
vector<DjiPoint> getRange(const vector<DjiPoint>& pts, int startIndex, int endIdex);

/*
*/
vector<DjiPoint> reversePoints(const vector<DjiPoint>& pts);

/*@param points Is Anti-Clockwise order
*/
double calPolygonArea(const vector<DjiPoint> &points);

/*
*@biref Total distance of every two adjacent points
*/
double calPolylineDist(const vector<DjiPoint>& pts);

/*@brief Sort points with a vector
*/
void sortWithVec(vector<DjiPoint>& interSec, DjiPoint forwardVec);

/*@brief Get min and max points on forwardVec direction in pts
*/
void getMinMaxPt(vector<DjiPoint> pts, DjiVec forwardVec, DjiPoint &minPt, DjiPoint & maxPt);

vector<DjiPoint> dispersePolyline(vector<DjiPoint>polyline, double pieceLength);

/*@brief disperse lineSeq into pieces
@return include pStart and pEnd, all s in points is set 0!
*/
vector<DjiPoint> disperseFlightLineseq(DjiPoint pStart, DjiPoint pEnd, double pieceLength);

/*@brief Is there intersection between two line segments
*/
bool isIntersectLineseq(DjiLineSeq first, DjiLineSeq second);

/*@brief Is there intersection between two plygon edges
*/
bool isIntersection(const DjiEdge& edge1, const DjiEdge& edge2);

/*@brief Is there intersection between two line, if Yes return in parameter intersec
*/
bool lineIntersection(DjiLine l1, DjiLine l2, DjiPoint& intersec);

/*@brief Find the nearest point on line segment( --seq) to a point(--pos), return the nearest pionts in parameter nearest
@param pos Current position
@param line1 The first point of lineSeq
@param line2 The second point of lineSeq
@param nearest The the nearest point after calculation
*/
double findNearestOnLineseq(DjiPoint pos, DjiLineSeq seq, DjiPoint& nearest);


string int2string(int a);
string double2string(double b);

/*
@return interSec interSec.s is the intersection index in pts
*/
void lineInterSectionMarked(const vector<DjiPoint>& edge, const DjiLine line, vector<DjiPoint>& interSec, int markType);


/*@brief Find Path to around obstacles
@param pStart Start point
@param ptEnd End point
@param path Avoiding obstacle path
@param isKeepStartEndPoint True for keep ptStart & ptEnd in path
@param isClearPath True for clear path at fist of the function
@return The path distance
*/
double findTwoPointsPath(const vector<DjiEdge >& allAroundObsEdges, DjiPoint ptStart, DjiPoint ptEnd,
	vector<DjiPoint>& path, bool isKeepStartEndPoint=true, bool isClearPath=true);

void addPathWihtAroundEdge(const DjiEdge& around, const DjiPoint& pt0, const DjiPoint& pt1,
	vector<vector<DjiPoint> >& pathPts);

void sortPathsWithForwardVec(vector<vector<DjiPoint> >& pathPts, DjiPoint forwardVec);


void adjustAntiClockWise(vector<DjiEdge *>  edges);

void adjustAntiClockWise(DjiEdge * edges);

#endif // !DJI_WAYPOINT_VECTOR_UTILS_H

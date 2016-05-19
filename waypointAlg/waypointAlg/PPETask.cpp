#include "stdafx.h"

#include "djiConst.h"
#include "PPETask.h"
#include "debugIO.h"
#include "djiUtils.h"
#include "djiConst.h"

PPETask::PPETask()
{
	droneNum = 1;
	isUseCoordTransform = false;
}
PPETask::PPETask(DjiPoint _startSprayingPosition,double _sprayingDirection, double _sprayingWidth, 
	          int _droneNum,  bool _isUseCoordTransform)
{

	if (!(_sprayingDirection <= 360 && _sprayingDirection >= 0 &&
		_sprayingWidth>0 && _sprayingWidth<=MAX_SPRAY_WIDTH_METER))
	{
		printLog("WaypointTask init input error", true);
	}
	if (_isUseCoordTransform)
	{
		if (!isLegalLatLong(_startSprayingPosition))
		{
			printLog("_startSprayingPosition is illegal LatLong", true);
		}	
	}
	if (_droneNum<1)
	{
		printLog("inupt error droneNum<1", true);
	}

	isUseCoordTransform = _isUseCoordTransform;

	//all the latitude_longtitude data will be transformed into plane coordinate  (the unit is metre)
	tran = isUseCoordTransform ? CoordTransform(_startSprayingPosition) : CoordTransform();

	homePoint = isUseCoordTransform ? tran.latlong2plane(_startSprayingPosition) : _startSprayingPosition;

	droneNum = _droneNum;

	forwardVec = CoordTransform::dirSouthEastFromNorthAngle(_sprayingDirection);
	forwardVec = forwardVec / molMat(forwardVec);

	//rotate clockwise 90 degree
	sideVec.x = forwardVec.y;
	sideVec.y = -forwardVec.x;

	sprayWidth = _sprayingWidth;

	printLog(" ");
	printPoint("startSprayingPosition: ", _startSprayingPosition);
	printLog("sprayingDirection: " + double2string(_sprayingDirection));
	printLog("sprayingWidth: " + double2string(sprayWidth));
	printLog("droneNum: " + int2string(droneNum));

	initDefaultValues();
}

void PPETask::initDefaultValues()
{
	safeOffset = 5;

	sprayingOffset = 3;

	pieceLength = 5;

	maxGroundWalk = 500;

	landingDeviation = 25;

	flightSpeed = 3;

	timeOneBatteryS = 1000;

	maxFlightDist = flightSpeed*timeOneBatteryS;

	timeToChangeBattery = 120;

	minPowerPercent = 25;

/*	safeLandPercent = 10;*/
}

bool PPETask::addObsEdge(vector<DjiPoint>  edge, int type)
{
	if (edge.size() == 0)
		return false;
	if (isUseCoordTransform)
		edge = tran.latlong2plane(edge);
	printLog(" ");
	switch (type)
	{
	case DJI_EDGE_FARMLAND:
		printLog("add DJI_EDGE_FARMLAND edge size=" + int2string(edge.size()));
		farmEdge = edge;
		break;
	case DJI_EDGE_NSZ_AROUND:
		printLog("add DJI_EDGE_NSZ_AROUND edge size=" + int2string(edge.size()));
		nsz_around.push_back(edge);
		break;
	case DJI_EDGE_NSZ_OVER:
		printLog("add DJI_EDGE_NSZ_NOSPRAY edge size=" + int2string(edge.size()));
		nsz_blank.push_back(edge);
		printLog(" ");
		break;
	case DJI_EDGE_NSZ_HEIGHT:
		printLog("DJI_EDGE_CHANGE_HEIGHT handled as DJI_EDGE_FORBID, in addObsEdge");
		nsz_around.push_back(edge);
		//nsz_height.push_back(edge);
		break;
	case DJI_EDGE_NSZ_AROUND_OUTERSIDE:
		printLog("add DJI_EDGE_NSZ_AROUND_OUTERSIDE edge");
		obs_around_outside.push_back(edge);
		break;
	case DJI_LINE_CHARGING:
		printLog("add DJI_LINE_CHARGING polyline size=" + int2string(edge.size()));
		chargingLine_org = edge;
		break;
		
	default:
		printLog("input edge unknow type error, in addObsEdge",true);
		return false;
		break;
	}
	for (int i = 0; i < edge.size(); i++)
	{
		printLog(double2string(edge[i].x) + ", " + double2string(edge[i].y));
	}

	printLog(" ");
	return true;
}


bool PPETask::setSafeOffset(double safeOffset)
{
	if (safeOffset<0)
	{
		printLog("input error: safeOffset<0 in setSafeOffset()", true);
		return false;
	}
	else
	{
		printLog("setSafeOffset " + double2string(safeOffset));
	}
	this->safeOffset = safeOffset;
	return true;
}

bool PPETask::setSprayingOffset(double sprayingOffset)
{
	if (sprayingOffset < 0)
	{
		printLog("input error: sprayingOffset<0 in setSprayingOffset()", true);
		return false;
	}
	else
	{
		printLog("setSprayingOffset " + double2string(sprayingOffset));
	}
	this->sprayingOffset = sprayingOffset;
	return true;
}

bool PPETask::setPieceLength(double pieceLength)
{
	if (!(pieceLength >= MIN_CHARGING_DIVID_METER && pieceLength <= MAX_CHARGING_DIVID_METER))
	{
		printLog("input error : setPieceLength input error", true);
		return false;
	}
		
	this->pieceLength = pieceLength;
	return true;
}

bool PPETask::setMaxGroundWalk(double maxGroundWalk)
{
	if (maxGroundWalk < 0)
	{
		printLog("input error: maxGroundWalk<0 in setMaxGroundWalk()", true);
		return false;
	}
	else
	{
		printLog("setMaxGroundWalk " + double2string(maxGroundWalk));
	}
	this->maxGroundWalk = maxGroundWalk;
	return true;
}

bool PPETask::setLandingDeviation(double landingDeviation)
{
	if (landingDeviation < 0)
	{
		printLog("input error: landingDeviation<0 in setLandingDeviation()", true);
		return false;
	}
	else
	{
		printLog("setLandingDeviation " + double2string(landingDeviation));
	}
	this->landingDeviation = landingDeviation;
	return true;
}

bool PPETask::setFlightSpeed(double flightSpeed)
{
	if (flightSpeed < 0)
	{
		printLog("input error: flightSpeed<0 in setFlightSpeed()", true);
		return false;
	}
	else
	{
		printLog("setFlightSpeed " + double2string(flightSpeed));
	}
	this->flightSpeed = flightSpeed;
	this->maxFlightDist = flightSpeed*timeOneBatteryS;
	return true;
}

bool PPETask::setTimeOneBattery(double timeOneBattery)
{
	if (timeOneBattery < 0)
	{
		printLog("input error: timeOneBattery<0 in setTimeOneBattery()", true);
		return false;
	}
	else
	{
		printLog("setTimeOneBattery " + double2string(timeOneBattery));
	}
	this->timeOneBatteryS = timeOneBattery;
	this->maxFlightDist = flightSpeed*timeOneBattery;
	return true;
}

bool PPETask::setMinPowerPercent(double minPowerPercent)
{
	if (minPowerPercent < 0)
	{
		printLog("input error: minPowerPercent<0 in setMinPowerPercent()", true);
		return false;
	}
	else
	{
		printLog("setMinPowerPercent " + double2string(minPowerPercent));
	}
	this->minPowerPercent = minPowerPercent;
	return true;
}


bool PPETask::setTimeToChangeBattery(double timeToChangeBattery)
{
	if (timeToChangeBattery < 0)
	{
		printLog("input error: timeToChangeBattery<0 in setTimeToChangeBattery()", true);
		return false;
	}
	else
	{
		printLog("setTimeToChangeBattery " + double2string(timeToChangeBattery));
	}
	this->timeToChangeBattery = timeToChangeBattery;
	return true;
}


FarmSprayingCost PPETask::estimateFarmSprayCosts()
{
	/*if (waypoints.size() == 0)
		printLog("waypoint.size()==0", true);

	   //估计电池需要的总数，和任务消耗的时间  多机时间怎么计算？
	int changeBatteryTimes;
	double costTime;
	double reserve = maxFlightDist - pointDistance(waypoints[0], homePoint);
	unsigned nearCharge;
	vector<DjiPoint> tem;
	findNearestChargePoint(homePoint, 0, chargingLine.size() - 1, nearCharge, tem);
	getSupplyAreaWithDistance(nearCharge, maxGroundWalk, supplyStart, supplyEnd);
	batteryCheck(waypoints, reserve, supplyStart, supplyEnd, changeBatteryTimes);
	costTime = changeBatteryTimes*timeToChangeBattery + (changeBatteryTimes + 1)*timeOneBatteryS;
	*/

	double farmArea = calPolygonArea(farmEdge.edge);

	double obsArea = 0;
	for (int i = 0; i < (int)nsz_around.size(); i++)
	{
		obsArea += calPolygonArea(nsz_around[i].edge);
	}
	for (int i = 0; i < (int)nsz_blank.size(); i++)
	{
		obsArea += calPolygonArea(nsz_blank[i].edge);
	}
	for (int i = 0; i < (int)nsz_height.size(); i++)
	{
		obsArea += calPolygonArea(nsz_height[i].edge);
	}

	return FarmSprayingCost(/*changeBatteryTimes + 1, costTime, */farmArea, obsArea, farmArea - obsArea);
}

vector<DjiEdge* > PPETask::getAllAroundEdges()
{
	vector<DjiEdge* > ref;
	for (int i = 0; i < (int)nsz_around.size(); i++)
		ref.push_back(&nsz_around[i]);
	for (int i = 0; i < (int)obs_around_outside.size(); i++)
		ref.push_back(&obs_around_outside[i]);

	return ref;
}

vector<DjiEdge* > PPETask::getAllEdgesInFarmland()
{
	vector<DjiEdge* > ref;

	ref.push_back(&farmEdge);

	for (int i = 0; i < (int)nsz_around.size(); i++)
		ref.push_back(&nsz_around[i]);

	for (int i = 0; i < (int)nsz_blank.size(); i++)
		ref.push_back(&nsz_blank[i]);

	for (int i = 0; i < (int)nsz_height.size(); i++)
		ref.push_back(&nsz_height[i]);

	return ref;
}
vector<DjiEdge* > PPETask::getAllEdges()
{
	vector<DjiEdge* > ref = getAllEdgesInFarmland();
	for (int i = 0; i < (int)obs_around_outside.size(); i++)
		ref.push_back(&obs_around_outside[i]);
	return ref;
}



void PPETask::disperseChargingLine(vector<DjiPoint> _chargingline, double pieceLength)
{
	chargingLine.clear();

	if (_chargingline.size() == 0)
	{
		printLog("chargingline.size() == 0!  set homepoint as change battery point");
		chargingLine.push_back(homePoint);
		return;
	}
	if (pieceLength < 0)
	{
		printLog("pieceLength < 0 input error in disperseChargingLine()", true);
		return;
	}
	chargingLine = dispersePolyline(_chargingline, pieceLength);
}

void PPETask::dataPreHandle()
{
	//all edges
	adjustAntiClockWise(getAllEdges());

	if (farmEdge.size() < 3)
		printLog("farmEdge.size() < 3", true);
	if (farmEdge.size() == 0)
		printLog("farmEdge.size() == 0", true);

	//offsetSprayInflation
	farmEdge.inflate(sprayingOffset);
	for (int i = 0; i < (int)nsz_blank.size(); i++)
		nsz_blank[i].deflate(sprayingOffset);

	//offsetSprayInflation
	for (int i = 0; i < (int)nsz_around.size(); i++)
		nsz_around[i].inflate(sprayingOffset);
	for (int i = 0; i < (int)nsz_height.size(); i++)
		nsz_height[i].inflate(sprayingOffset);
	for (int i = 0; i < (int)obs_around_outside.size(); i++)
		obs_around_outside[i].inflate(sprayingOffset);

	//all edges
	vector<DjiEdge* > tem=getAllEdges();
	for (int i = 0; i < (int)tem.size(); i++)
	{
		tem[i]->calOuterRect();
	}

	tem = getAllEdges();
	for (int i = 0; i < (int)tem.size(); i++)
	{
		for (int j = i + 1; j < (int)tem.size(); j++)
		{
			if (isIntersection(*tem[i], *tem[j]))
				printLog("two edge intersected, in dataPreHandle",true);
		}
	}
}

void PPETask::drawMarkedPoints()
{
	drawImg(farmEdge.edge, COL_WHITE, CV_DRAW_CLOSED_POLYLINE, 0);
// 	for (int j = 0; j < farmEdge.edge.size(); j++)
// 	{
// 		drawPoint(mapImg, farmEdge.edge[j], COL_RED, 1);
// 	}
	for (int i = 0; i < (int)nsz_around.size(); i++)
	{
		drawImg(nsz_around[i].edge, COL_RED, CV_DRAW_CLOSED_POLYLINE, 0);
// 		for (int j = 0; j < obs_forbid[i].edge.size();j++)
// 		{
// 			drawPoint(mapImg, obs_forbid[i].edge[j], COL_GREEN, 1);
// 		}
	}
	for (int i = 0; i < (int)nsz_blank.size(); i++)
	{
		drawImg(nsz_blank[i].edge, COL_SLIVER, CV_DRAW_CLOSED_POLYLINE, 0);
// 		for (int j = 0; j < obs_noSpray[i].edge.size(); j++)
// 		{
// 			drawPoint(mapImg, obs_noSpray[i].edge[j], COL_RED, 1);
// 		}

	}
	for (int i = 0; i < (int)nsz_height.size(); i++)
	{
		drawImg(nsz_height[i].edge, COL_YELLOW, CV_DRAW_CLOSED_POLYLINE, 0);
// 		for (int j = 0; j < obs_change_height[i].edge.size(); j++)
// 		{
// 			drawPoint(mapImg, obs_change_height[i].edge[j], COL_RED, 1);
// 		}
	}
	drawImg(chargingLine, COL_YELLOW, CV_DRAW_POLYLINE, 1);
}


void PPETask::markOneEdge(vector<DjiPoint>& _pts, int & usedIndex)
{
	if (_pts.size() < 3)
	{
		printLog("_pts.size() < 3, in getMarkerIndex", true);
	}

	char markerIndex = 0;
	DjiPoint last;
	DjiPoint foucas;
	DjiPoint next;

	for (int i = 0; i < (int)_pts.size(); i++)
	{
		last = _pts[(i - 1 + _pts.size()) % _pts.size()];
		foucas = _pts[i];
		next = _pts[(i + 1) % _pts.size()];
		_pts[i].s = markerIndex;

		if (sideVec.dot(last - foucas)*sideVec.dot(next - foucas) >= 0)
		{
			markerIndex++;
		}
	}

	last = _pts[_pts.size() - 2];
	foucas = _pts[_pts.size() - 1];
	next = _pts[0];
	char tempIndex = _pts[_pts.size() - 1].s;
	double aaa = sideVec.dot(last - foucas)*sideVec.dot(next - foucas);
	if (sideVec.dot(last - foucas)*sideVec.dot(next - foucas) < 0)
	{
		for (int i = 0; i < (int)_pts.size(); i++)
		{
			if (_pts[i].s == (int)_pts[_pts.size() - 1].s)
			{
				_pts[i].s = 0;
			}
		}
	}
	for (int i = 0; i < (int)_pts.size(); i++)
	{
		_pts[i].s += usedIndex;
	}
	usedIndex += markerIndex;
}

int PPETask::markAllEdge()
{
	int usedIndex = 0;
	vector<DjiEdge* > allEdgesInFarmland = getAllEdgesInFarmland();
	for (int i = 0; i < (int)allEdgesInFarmland.size(); i++)
	{
		markOneEdge(allEdgesInFarmland[i]->edge, usedIndex);//mark divided area frags
	}
	return usedIndex;
}

vector<vector<DjiPoint> > PPETask::getOrderInterSections()
{
	vector<vector<DjiPoint> > inSec_seq;

	DjiPoint  minPt;
	DjiPoint  maxpt;
	getMinMaxPt(farmEdge.edge,forwardVec,minPt, maxpt);
	if (sideVec.dot(maxpt - minPt) < 0)
		sideVec = -sideVec;

	vector<DjiPoint>  interSecs;
	vector<DjiEdge* > allEdgesInFarmland = getAllEdgesInFarmland();
	for (int i = 1;; i++)
	{
		if (i*sprayWidth > sideVec.dot(maxpt - minPt))
			break;

		interSecs.clear();
		for (int j = 0; j < (int)allEdgesInFarmland.size(); j++)
		{
			lineInterSectionMarked(allEdgesInFarmland[j]->edge, DjiLine(minPt + i*sprayWidth*sideVec, forwardVec), interSecs,INTERSEC_MARKED_WITH_POINT_S);
		}

		sortWithVec(interSecs, forwardVec);

		if (interSecs.size() > 0)
			inSec_seq.push_back(interSecs);
	}

	printLog("getOrderInterSections complete");

	return inSec_seq;
}

vector<vector<DjiPoint> > PPETask::areaDivideMethod(vector<vector<DjiPoint> > inSec_seq, unsigned usedIndex)
{
	vector<vector<DjiPoint> > areaFrags;

    bool  isObstacle = false;
	int _start=0;
	int _end=0;

	//有障碍物区域和无障碍物区域分开
	vector<vector<DjiPoint> > areaFragsTem = vector<vector<DjiPoint> >(usedIndex);

	vector<DjiPoint> aFrags;
	for (int i = 0; i < (int)inSec_seq.size(); i++)
	{
		if (isObstacle == false && inSec_seq[i].size()>2 && i != (inSec_seq.size() - 1))//进入障碍物
		{
			_end = i;
			isObstacle = true;
		}
		else if (isObstacle == true && inSec_seq[i].size() <= 2 && i != (inSec_seq.size() - 1))//出障碍物区
		{
			_end = i;
			isObstacle = false;
		}
		else if (i == (inSec_seq.size() - 1))
		{
			_end = i+1;
			isObstacle = true;
		}
		else
			continue;

		for (int m = _start; m < _end; m++)
		{
			if (inSec_seq[m].size() >= 2)
			{
				for (int n = 0; n < (int)inSec_seq[m].size() - 1; n = n + 2)
				{
					areaFragsTem[inSec_seq[m][n].s].push_back(inSec_seq[m][n]);
					areaFragsTem[inSec_seq[m][n].s].push_back(inSec_seq[m][n + 1]);
				}
			}
		}
		_start = _end;

		char lastIndex = -1;
		vector<DjiPoint> block;
		for (int i = 0; i < (int)areaFragsTem.size(); i++)
		{
			block.clear();
			lastIndex = -1;
			for (int j = 0; j < (int)areaFragsTem[i].size()-1;j=j+2)
			{
				if (lastIndex==-1)
				{
					lastIndex = areaFragsTem[i][j + 1].s;
					block.clear();
					block.push_back(areaFragsTem[i][j]);
					block.push_back(areaFragsTem[i][j+1]);
				}
				else
				{
					if (lastIndex == areaFragsTem[i][j + 1].s)
					{
						block.push_back(areaFragsTem[i][j]);
						block.push_back(areaFragsTem[i][j + 1]);
						continue;
					}
					else
					{
						if (block.size())
						{
							areaFrags.push_back(block);
							block.clear();
						}
						block.push_back(areaFragsTem[i][j]);
						block.push_back(areaFragsTem[i][j + 1]);
						lastIndex = areaFragsTem[i][j + 1].s;
					}

				}
			}
			if (block.size())
			{
				areaFrags.push_back(block);
				block.clear();
			}
		}
		for (int i = 0; i < (int)areaFragsTem.size(); i++)
		{
			if (areaFragsTem[i].size() != 0)
			{
				areaFragsTem[i].clear();
			}
		}
	}


// 	for (int i = 0; i < (int)areaFrags.size();i++)
// 	{
// 		drawImg(areaFrags[i], COL_GREEN, CV_DRAW_POLYLINE, true);
// 	}

	//清除航线段的标记  统一标记为喷洒标记
	for (int i = 0; i < areaFrags.size(); i++)
	{
		for (int j = 0; j < areaFrags[i].size(); j++)
		{
			//areaFrags[i][j].s = P_SPRAY;
			areaFrags[i][j].s = POINT_NO_SPRAY;
		}
	}

	printLog("areaDivideMethod complete;  areaFrags.size()=" + int2string(areaFrags.size()));

	return areaFrags;
}


vector<vector<DjiPoint>> PPETask::getMultDroneMission()
{
	//mark defferent blocks, calculate edges outerRect
	dataPreHandle();
	printLog("dataPreHandle complete");

	disperseChargingLine(chargingLine_org, pieceLength);

	drawMarkedPoints();

	int usedIndex = markAllEdge();

	vector<vector<DjiPoint > > _areaFrags = areaDivideMethod(getOrderInterSections(), usedIndex);

	BlockManager bolockManager = BlockManager(_areaFrags, homePoint, droneNum, getAllAroundEdges());

	//get ground station waypoints  when point sing s==1, open sprayer for spraying pesticides otherwise close sprayer
	vector<vector<DjiPoint>> taskSet=bolockManager.calNearestPath();

	//去掉重复的航点
	vector<vector<DjiPoint>> taskSetCopy = vector<vector<DjiPoint>>(taskSet.size());
	for (int i = 0; i < taskSet.size(); i++)
	{
		for (int j = 0; j<taskSet[i].size() ; j++)
		{
			if (j>0 && taskSet[i][j].s != POINT_SPRAY && taskSet[i][j].x == taskSet[i][j - 1].x &&taskSet[i][j].y == taskSet[i][j - 1].y)
			{
				continue;
			}
			taskSetCopy[i].push_back(taskSet[i][j]);
		}
	}
	

	vector<SprayTask> sprayTaskSet;
	for (int i = 0; i < taskSet.size(); i++)
	{
		sprayTaskSet.push_back(SprayTask());
	}
	for (int i = 0; i < taskSet.size();i++)
	{
		taskSet[i] = isUseCoordTransform ? tran.plane2latlong(taskSet[i]) : taskSet[i];
		printPoints(taskSet[i]);
	}

	sprayingcost=estimateFarmSprayCosts();

	return taskSet;
}

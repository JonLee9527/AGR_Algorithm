#ifndef DJI_GET_WAYPOINT_PPETASK
#define DJI_GET_WAYPOINT_PPETASK

#include "math.h"

#include "djiPoint.h"
#include "CoordTransform.h"

#include "DjiClass.h"

#include "DjiEdge.h"

#include "BlockManager.h"

#include "SprayTask.h"

class PPETask
{
private:   
	//input data
	//farmland edge
	DjiEdge farmEdge;

	//obstacles edge
	vector<DjiEdge > nsz_around;

	//obstacles UAV needs raise or descend to avoid 
	vector<DjiEdge > nsz_height;

	//No Spray Area
	vector<DjiEdge > nsz_blank;

	//obstacle outside farmland
	vector<DjiEdge > obs_around_outside;


	//when test on windows  is 0 
	bool isUseCoordTransform;

	DjiPoint homePoint;

	int droneNum;

	//spray width range
	double sprayWidth;

	//to avoid foridObstacles
	double safeOffset;

	//ensure fully sparying farmland edge
	double sprayingOffset;

	vector<DjiPoint> chargingLine_org;

	//disperse charingline in pices 可充电的线段离散化单位，以及作为将返航前的航线段离散化单位  
	//也作为飞机即将没电 将最后的航线段离散化 可以飞更多路程
	double pieceLength;

	//battery parameters
	//max flight distance with full electricity
	double maxFlightDist;

	//the safe power left when landing 
	double minPowerPercent;


	// the max walk distance of the man on the ground who supply batterys
	double  maxGroundWalk;

	//progran would estimate next changing battery point where the man wait aircraft, detaWalk is a landing deviation
	double landingDeviation;

	//speed when spraying 
	double flightSpeed;

	//seconds, srpaying time for full battery
	double timeOneBatteryS;

	//seconds
	double timeToChangeBattery;


private:

	//coordinates transform between coord_latitude_longtitude and coord_plane
	CoordTransform tran;

	//available region to change battery, which is a curve
	vector<DjiPoint> chargingLine;

	//spray direction, decided by wind direction
	DjiVec  forwardVec;

	//vertical vector of forwardVec
	DjiVec  sideVec;

	FarmSprayingCost sprayingcost;


public:
	PPETask();

public:
	/***interfaces***/

	/*@param farmlandEdge Farmland edge points
	@param startSprayingPosition Drone homePoints generally
	@param sprayingDirection Anti-clockwise angle from North in North-East earth Map
	@param sprayingWith Spraying Width
	@param isUseCoordTransform Is true generally, false when testing on windows 
	*/
	PPETask(DjiPoint startSprayingPosition, double sprayingDirection, double sprayingWithv, int _droneNum, bool isUseCoordTransform);

	/*@brief add all kinds of obstacles
	*/
	bool addObsEdge(vector<DjiPoint> edge,int type);

	bool setSafeOffset(double safeOffset);

	bool setSprayingOffset(double sprayingOffset);

	//call befor setChargingline()
	bool setPieceLength(double pieceLength);

	bool setMaxGroundWalk(double maxGroundWalk);

	bool setLandingDeviation(double landingDeviation);

	bool setFlightSpeed(double flightSpeed);

	bool setTimeOneBattery(double timeOneBattery);

	bool setMinPowerPercent(double minPowerPercent);

	bool setTimeToChangeBattery(double timeToChangeBattery);

	/*@brief get groundstation wayoints after constructing WaypointTask
	@return groundstation waypoints
	*/
	vector<vector<DjiPoint> > getMultDroneMission();

	int getEstimatedBatteryNum()
	{
		return sprayingcost.batteryNumber;
	}
	double getEstimatedCostTime()
	{
		return sprayingcost.costTime;
	}
	double getEstimatedFarmArea()
	{
		return sprayingcost.farmArea;
	}
	double getEstimatedObsArea()
	{
		return sprayingcost.obstaclesArea;
	}
	double getEstimatedSprayingArea()
	{
		return sprayingcost.sprayingArea;
	}

public:  //use for windows test case

	
	vector<DjiPoint> getDisperatedChargingLine()
	{
		return chargingLine;
	}

	
	CoordTransform getTran()
	{
		return tran;
	}

	vector<DjiEdge* > getAllAroundEdges();

private:
	vector<DjiEdge* > getAllEdgesInFarmland();
	vector<DjiEdge* > getAllEdges();

	/*@brief test in windows, use image as map
	*/
	void drawMarkedPoints();


private:

	void initDefaultValues();

	/*@brief delete middle points when point number more than 2 on the same line
	@brief Divide marker 
	*/
	void dataPreHandle();

	void disperseChargingLine(vector<DjiPoint> chargingline, double pieceLength);

	/*@brief dividing spray lineSeqs in groups 
	*/
	void markOneEdge(vector<DjiPoint>& _pts, int & usedIndex);
	int markAllEdge();
	
	/*@brief 求交并按飞行方向排序， 返回交点  vector<DjiPoint>表示和某个边缘求交所得的所有 “交点对”
	*/
	vector<vector<DjiPoint> > getOrderInterSections();
	

	//区域分块，并将结果返回
	vector<vector<DjiPoint> > areaDivideMethod(vector<vector<DjiPoint> > inSec_seq, unsigned usedIndex);



	FarmSprayingCost estimateFarmSprayCosts();

public:
};
#endif // !DJI_GET_WAYPOINT_TASK

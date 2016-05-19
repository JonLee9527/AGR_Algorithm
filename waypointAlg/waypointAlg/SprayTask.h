#ifndef DJI_GET_WAYPOINT_SPRAYTASK
#define DJI_GET_WAYPOINT_SPRAYTASK

#include "djiPoint.h"
#include "DjiClass.h"
#include "CoordTransform.h"

#include "DjiEdge.h"

class SprayTask
{
private:
	//*************Output data*************//
	ChargingPath chargingPath;

	//********Input data for init*********//

	vector<DjiPoint> waypoints;

	vector<DjiPoint> chargingLine;
	
	vector<DjiEdge > allAroundObsEdgesObj;

	DjiPoint homePoint;

	//battery parameters
	//max flight distance with full electricity
	double maxFlightDist;

	//the safe power left when landing 
	double minPowerPercent;

	//data from test team
	//double safeLandPercent;

	// the max walk distance of the man on the ground who supply batterys
	double  maxGroundWalk;

	//progran would estimate next changing battery point where the man wait aircraft, detaWalk is a landing deviation
	double landingDeviation;

	//也作为飞机即将没电 将最后的航线段离散化 可以飞更多路程
	double pieceLength;


	//**************middle date*************//
	//actually flight parameters
	unsigned supplyStart;
	unsigned supplyEnd;


	//when test on windows  is false,  true on android
	bool isUseCoordTransform;

	//coordinates transform between coord_latitude_longtitude and coord_plane
	CoordTransform tran;

// 	//use for find charging path with the start and end point
// 	vector<DjiEdge * > allAroundObsEdges;


public:
	SprayTask(){};

	SprayTask(vector<DjiPoint> waypoints, vector<DjiPoint> chargingLine, vector< DjiEdge> allAroundObsEdgesObj
		, DjiPoint homePoint, double minPowerPercent,
		double maxFlightDist, double  maxGroundWalk,
		double landingDeviation, double pieceLength
		, bool isUseCoordTransform=true)
	{
		this->waypoints = waypoints;
		this->chargingLine = chargingLine;

		this->allAroundObsEdgesObj = allAroundObsEdgesObj;

		this->isUseCoordTransform = isUseCoordTransform;
		if (isUseCoordTransform)
		{
			this->tran = CoordTransform(homePoint);
		}
		this->homePoint = waypoints[0];  //第一个换电池的点参考 每一个子任务的第一个任务点

		this->maxFlightDist = maxFlightDist;
		this->minPowerPercent = minPowerPercent;
		this->maxGroundWalk = maxGroundWalk;

		this->landingDeviation = landingDeviation;
		this->pieceLength = pieceLength;

		getFirstChargingPoint();
	}

	/*@brief After getting waypoints, the function estimate the first charging point,
	the man can go the point when task begin to wait drone coming for changing battery
	*/
	void getFirstChargingPoint();

	/*@brief when drone arrived a way point, detect whether go to charging point
	@param waypointIndex
	*/
	bool askCharging(int waypointIndex, double power);

	vector<DjiPoint> getChangeBatteryPath(){
		return chargingPath.changeBatteryPath;
	}

	vector<DjiPoint> getResumeMissionTask(){
		return chargingPath.resumeMissionTask;
	}

	vector<DjiPoint> getBatterySupplyArea(){
		return chargingPath.batterySupplyArea;
	}

private:

	/*@brief path will be cleared first
	@brief no change any global varible
	@param pos UAV postion now
	@return distance cost
	*/
	double findNearestChargePoint(DjiPoint pos, unsigned chargeStart, unsigned chargeEnd, unsigned& nearest, vector<DjiPoint>& path);

	//根据计算获得的waypoints 预估一整条换电池路径，用于计算该task 所需要的电池数总量
	double batteryCheck(const vector<DjiPoint>& taskPoints, double& reserve, unsigned& chargeStart, 
		unsigned& chargeEnd, int& changeBatteryTimes);

	unsigned estimateNextChargingPoint(const vector<DjiPoint>& taskPoints, double reserve, unsigned chargeStart, unsigned chargeEnd);

	/*@brief with last charging index and maxGroundWalk,  get the index range could provide changing battry service
	@param lastIndex  the last index in chargingLine to changing battery
	@param maxWalk  maxGround walk in one-battery time
	@return chargeStart&chargeEnd the index range in chargingLine providing changing battry service
	*/
	void getSupplyAreaWithDistance(unsigned lastIndex, double maxWalk, unsigned& chargeStart, unsigned& chargeEnd);

	/*@brief used this->variable,  maxflightDist minPowerPercent safeLandPercent
	@param pStart the first waypoint
	@param pEnd the second waypoint
	@param distLeft
	@return the path includes spraying and goCharging
	*/
	void findMiddlePointToGetChargingPath(DjiPoint pStart, DjiPoint pEnd, double distLeft, int chargingStart, int chargingEnd,
		vector<DjiPoint>& goChargingPath, vector<DjiPoint>& continueTaskPath, unsigned& nearCharge);


};

#endif
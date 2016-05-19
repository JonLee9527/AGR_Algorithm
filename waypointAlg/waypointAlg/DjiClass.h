#ifndef DJI_CLASS_DATA_DEBUG
#define DJI_CLASS_DATA_DEBUG
#include "djiPoint.h"

class FarmSprayingCost
{
public:

	int batteryNumber;

	//time cost in the whole spraying process
	double costTime;

	double farmArea;

	double obstaclesArea;

	double sprayingArea;

	//Å©Ò©Á¿
	//double pesticides;

	FarmSprayingCost()
	{
	}

	FarmSprayingCost(int _batteryNumber, double _costTime, double _farmArea, double _obstaclesArea, double _sprayingArea)
	{
		batteryNumber = _batteryNumber;
		costTime = _costTime;
		farmArea = _farmArea;
		obstaclesArea = _obstaclesArea;
		sprayingArea = _sprayingArea;
	}

	FarmSprayingCost(double _farmArea, double _obstaclesArea, double _sprayingArea)
	{
		farmArea = _farmArea;
		obstaclesArea = _obstaclesArea;
		sprayingArea = _sprayingArea;
	}
	
};

class BatterySupplyArea
{
public:
	DjiPoint nextChargingPoint;
	vector<DjiPoint> batterySupplyArea;
	BatterySupplyArea()
	{
		nextChargingPoint = DjiPoint();
		batterySupplyArea = vector<DjiPoint>();
	}

	BatterySupplyArea(DjiPoint _nextChargingPoint, vector<DjiPoint> _area)
	{
		nextChargingPoint = _nextChargingPoint;
		batterySupplyArea = _area;
	}
};
class ChargingPath
{
public:
	vector<DjiPoint> changeBatteryPath;
	vector<DjiPoint> resumeMissionTask;
	vector<DjiPoint> batterySupplyArea;

	ChargingPath()
	{
		changeBatteryPath = vector<DjiPoint>();
		resumeMissionTask = vector<DjiPoint>();
		batterySupplyArea = vector<DjiPoint>();
	}
	//for the last waypoint
	ChargingPath(vector<DjiPoint> _goChargingPath)
	{
		this->changeBatteryPath = _goChargingPath;
		this->resumeMissionTask = vector<DjiPoint>();
		this->batterySupplyArea = vector<DjiPoint>();
	}
	ChargingPath(vector<DjiPoint> _goChargingPath, vector<DjiPoint> _continueTaskPath, vector<DjiPoint> _batterySupplyArea)
	{
		this->changeBatteryPath = _goChargingPath;
		this->resumeMissionTask = _continueTaskPath;
		this->batterySupplyArea = _batterySupplyArea;
	}
};

#endif // !DJI_CLASS_DATA_DEBUG

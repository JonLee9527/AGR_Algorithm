#include "stdafx.h"

#include "SprayTask.h"
#include "djiConst.h"
#include "debugIO.h"
#include "djiUtils.h"
#include "djiConst.h"


double SprayTask::batteryCheck(const vector<DjiPoint>& taskPoints, double& reserve,
	unsigned& chargeStart, unsigned& chargeEnd, int& changeBatteryTimes)
{
	double _r = reserve;
	double cost2next = 0;
	unsigned nearCharge;
	double cost = 0;
	double costSum = 0;

	double used = 0;
	int begainIndex = 0;
	vector<DjiPoint> pathTem;

	changeBatteryTimes = 0;

	//�ɻ���i-1λ����
	for (unsigned i = 1; i < taskPoints.size(); i++)
	{
		used += pointDistance(taskPoints[i - 1], taskPoints[i]);
		if (used >= _r)
		{
			used -= pointDistance(taskPoints[i - 1], taskPoints[i]);
			i = i - 1;
			for (int j = i; j >= begainIndex; j--)
			{
				cost = findNearestChargePoint(taskPoints[j], chargeStart, chargeEnd, nearCharge, pathTem);
				i = j;
				double temp = used + cost;
				if (used + cost < _r || j <= begainIndex)
				{
					if (j == begainIndex)
					{
						//��������������Է�����һ������ ���ش���
						if (j + 1 < (int)taskPoints.size())
						{
							if (maxFlightDist < cost + pointDistance(taskPoints[j], taskPoints[j + 1]) +
								findNearestChargePoint(taskPoints[j + 1], chargeStart, chargeEnd, nearCharge, pathTem))
								printLog("�����������Է���һ������, in batteryCheck", true);
						}
					}
					//taskPoints[j].s = P_CHARGE;//����ر��
					costSum += cost;//�ɻ������ط����ϴε������
					_r = maxFlightDist - cost;
					getSupplyAreaWithDistance(nearCharge, maxGroundWalk, chargeStart, chargeEnd);
					begainIndex = j;

					changeBatteryTimes++;

					//**//
					vector<DjiPoint> pts;
					pts.push_back(taskPoints[i - 1]);
					pts.push_back(chargingLine[nearCharge]);
					//drawImg(mapImg, pts, CV_RGB(255, 0, 0), CV_DRAW_POINTS);
					break;
				}
				else
				{
					used -= pointDistance(taskPoints[j - 1], taskPoints[j]);
				}

			}
			used = 0;
		}

	}
	reserve = _r - used;
	return costSum;
}

double SprayTask::findNearestChargePoint(DjiPoint pos, unsigned chargeStart, unsigned chargeEnd, unsigned& nearest, vector<DjiPoint>& path)
{
	if (chargeStart > chargeEnd || chargeStart < 0 || chargeEnd >= chargingLine.size())
		printLog("input error, in findNearestChargePoint", true);
	path.clear();

	vector<DjiPoint> pathTem;
	double distTem;
	double minDist = findTwoPointsPath(allAroundObsEdgesObj, pos, chargingLine[chargeStart], pathTem);
	path = pathTem;
	nearest = chargeStart;
	for (unsigned i = chargeStart + 1; i <= chargeEnd; i++)
	{
		distTem = findTwoPointsPath(allAroundObsEdgesObj, pos, chargingLine[i], pathTem);
		if (distTem < minDist)
		{
			minDist = distTem;
			nearest = i;
			path = pathTem;
		}
	}
	return minDist;
}
void SprayTask::getSupplyAreaWithDistance(unsigned lastIndex, double maxWalk,
	unsigned& chargeStart, unsigned& chargeEnd)
{
	chargeStart = lastIndex;
	chargeEnd = lastIndex;
	for (int i = lastIndex + 1; i < (int)chargingLine.size(); i++)
	{
		if (pointDistance(chargingLine[lastIndex], chargingLine[i])>maxWalk)
		{
			break;
		}
		chargeEnd = i;
	}
	for (int i = lastIndex - 1; i > 0; i--)
	{
		if (pointDistance(chargingLine[lastIndex], chargingLine[i]) > maxWalk)
		{
			break;
		}
		chargeStart = i;
	}
}

unsigned SprayTask::estimateNextChargingPoint(const vector<DjiPoint>& taskPoints, double reserve,
	unsigned chargeStart, unsigned chargeEnd)
{
	double _r = reserve;
	double cost2next = 0;
	unsigned nearCharge = 0;
	double cost = 0;
	double costSum = 0;

	double used = 0;
	int begainIndex = 0;
	vector<DjiPoint> pathTem;

	//�ɻ���i-1λ����
	for (unsigned i = 1; i < taskPoints.size(); i++)
	{
		used += pointDistance(taskPoints[i - 1], taskPoints[i]);
		if (used >= _r)
		{
			used -= pointDistance(taskPoints[i - 1], taskPoints[i]);
			i = i - 1;
			for (int j = i; j >= begainIndex; j--)
			{
				cost = findNearestChargePoint(taskPoints[j], chargeStart, chargeEnd, nearCharge, pathTem);
				i = j;
				double temp = used + cost;
				if (used + cost < _r || j <= begainIndex)
				{
					if (j == begainIndex)
					{
						//��������������Է�����һ������ ���ش���
						if (j + 1 < (int)taskPoints.size())
						{
							if (maxFlightDist < cost + pointDistance(taskPoints[j], taskPoints[j + 1]) +
								findNearestChargePoint(taskPoints[j + 1], chargeStart, chargeEnd, nearCharge, pathTem))
								printLog("�����������Է���һ������, in batteryCheck", true);
						}
					}
					//taskPoints[j].s = P_CHARGE;//����ر��
					costSum += cost;//�ɻ������ط����ϴε������
					_r = maxFlightDist - cost;
					getSupplyAreaWithDistance(nearCharge, maxGroundWalk, chargeStart, chargeEnd);
					begainIndex = j;

				}
				else
				{
					used -= pointDistance(taskPoints[j - 1], taskPoints[j]);
				}

			}
			used = 0;
			//once
			return nearCharge;
		}
	}

	reserve = _r - used;
	findNearestChargePoint(taskPoints[taskPoints.size() - 1], chargeStart, chargeEnd, nearCharge, pathTem);
	return nearCharge;
}


void SprayTask::getFirstChargingPoint()
{
	//��һ������صĵ�  ��Ϊ��home�������
	unsigned nearCharge;
	vector<DjiPoint> tem;
	findNearestChargePoint(homePoint, 0, chargingLine.size() - 1, nearCharge, tem);
	getSupplyAreaWithDistance(nearCharge, maxGroundWalk, supplyStart, supplyEnd);
	unsigned estimateNext = estimateNextChargingPoint(waypoints, maxFlightDist, supplyStart, supplyEnd);
	getSupplyAreaWithDistance(estimateNext, landingDeviation, supplyStart, supplyEnd);
	//return BatterySupplyArea(chargingLine[estimateNext], getRange(chargingLine, supplyStart, supplyEnd));

	//���ڻ�ȡ��һ�λ��������
	chargingPath = ChargingPath(vector<DjiPoint>(), vector<DjiPoint>(), getRange(chargingLine, supplyStart, supplyEnd));
}
bool SprayTask::askCharging(int waypointIndex, double powerPercent)
{

	chargingPath = ChargingPath();

	if (powerPercent > 100 || powerPercent < 0)
	{
		printLog("powerPercent out of range !", true);
		return false;
	}
	if (waypointIndex >= (int)waypoints.size() || waypointIndex < 0)
	{
		printLog("waypointIndex out of range !", true);
		return false;
	}

// 	if (powerPercent < safeLandPercent)
// 	{
// 		printLog("powerPercent < safeLandPercent!!!!!", true);
// 	}

	vector<DjiPoint> goChargingPath;
	vector<DjiPoint> continueTaskPath;
	int usedIndex = -1;  //ʹ���˵ĺ���index  continuepath��Ҫ��֤��index���浽index+1�ĺ��߶ζ��������
	unsigned nearCharge = 0;


	if (waypointIndex == waypoints.size() - 1)
	{
		printLog("the last waypoint,  task complete !!!!", false);

		findNearestChargePoint(waypoints[waypointIndex], supplyStart, supplyEnd, nearCharge, goChargingPath);

		if (isUseCoordTransform)
			goChargingPath = tran.plane2latlong(goChargingPath);
		//���һ������
		chargingPath = ChargingPath(goChargingPath);
		return true;
	}

	//���㵱ǰpower���ܷɶ�Զ
	double distLeft = maxFlightDist*(powerPercent - minPowerPercent) / (100.0 - minPowerPercent);

	//�Ƿ��ܹ�������һ��waypoint + gohome �ĵ���
	double gohomeCost = findNearestChargePoint(waypoints[waypointIndex + 1], supplyStart, supplyEnd, nearCharge, goChargingPath);
	double goNextCost = pointDistance(waypoints[waypointIndex], waypoints[waypointIndex + 1]);

	//����������
	if (gohomeCost + goNextCost < distLeft)
		return false;
	else//gohomeCost + goNextCost > distLeft
	{

		//********************* Print Log ***********************//

		printLog("minPowerPercent=" + double2string(minPowerPercent));

		printLog("waypointIndex=" + int2string(waypointIndex));

		printLog("powerPercent=" + double2string(powerPercent));

		printLog("maxFlightDist=" + double2string(maxFlightDist));

		printLog("distLeft =" + double2string(distLeft));

		printLog("goNextCost=" + double2string(goNextCost));

		printLog("gohomeCost= " + double2string(gohomeCost));

		printLog("goNextCost+gohomeCost =" + double2string(goNextCost + gohomeCost) + "   >   distLeft:" + double2string(distLeft));

		if (!(waypoints[waypointIndex].s == 0 || waypoints[waypointIndex].s == 1))
		{
			printLog("waypoints[waypointIndex] with erro s=" + int2string(waypoints[waypointIndex].s), true);
		}
		//********************* Print Log  End ***********************//

		//��һ�κ��߲�������, �ӵ�ǰwaypointIndex���������
		if (waypoints[waypointIndex].s == 0)
		{
			//ֱ�Ӵӵ�ǰ��ؼ�
			findNearestChargePoint(waypoints[waypointIndex], supplyStart, supplyEnd, nearCharge, goChargingPath);
			for (usedIndex = waypointIndex + 1; usedIndex < (int)waypoints.size(); usedIndex++)
			{
				//�ϵ�����ʱ�ɵ���һ����ʼ������λ��
				if (waypoints[usedIndex].s == 1)
				{
					findTwoPointsPath(allAroundObsEdgesObj, chargingLine[nearCharge], waypoints[usedIndex], continueTaskPath);
					break;
				}
			}
			//����������޶ϵ������ĵ�continueTaskPath
			if (continueTaskPath.size() == 0)
			{
				chargingPath = ChargingPath(isUseCoordTransform ? tran.plane2latlong(goChargingPath) : goChargingPath);
				return true;
			}
			//��֤usedIndex ��usedIndex֮��ĺ��߶η���
			if (usedIndex + 1 < (int)waypoints.size())
			{
				continueTaskPath.push_back(waypoints[usedIndex + 1]);
			}
		}
		//��һ��Ϊ�������߶Σ��ӵ�ǰwaypointIndex����������ֱ��ʣ������պù��ؼ�
		else if (waypoints[waypointIndex].s == 1)
		{
			findMiddlePointToGetChargingPath(waypoints[waypointIndex], waypoints[waypointIndex + 1],
				distLeft, supplyStart, supplyEnd, goChargingPath, continueTaskPath, nearCharge);
			usedIndex = waypointIndex;
		}

		continueTaskPath.insert(continueTaskPath.end(), waypoints.begin() + usedIndex + 1, waypoints.end());

		getSupplyAreaWithDistance(nearCharge, maxGroundWalk, supplyStart, supplyEnd);
		unsigned estimateNext = estimateNextChargingPoint(continueTaskPath, maxFlightDist, supplyStart, supplyEnd);
		getSupplyAreaWithDistance(estimateNext, landingDeviation, supplyStart, supplyEnd);

		DjiPoint nextChargingPoint = chargingLine[estimateNext];
		vector<DjiPoint> supplyArea = getRange(chargingLine, supplyStart, supplyEnd);

		//����waypoints
		waypoints = continueTaskPath;

		if (isUseCoordTransform)
		{
			goChargingPath = tran.plane2latlong(goChargingPath);
			continueTaskPath = tran.plane2latlong(continueTaskPath);
			nextChargingPoint = tran.plane2latlong(nextChargingPoint);
			supplyArea = tran.plane2latlong(supplyArea);
		}
		chargingPath = ChargingPath(goChargingPath, continueTaskPath, supplyArea);
		return true;
	}
	return false;
}

void SprayTask::findMiddlePointToGetChargingPath(DjiPoint pStart, DjiPoint pEnd, double distLeft,
	int chargingStart, int chargingEnd, vector<DjiPoint>& goChargingPath, vector<DjiPoint>& continueTaskPath, unsigned& nearCharge)
{
	if (pStart.s != 1)
		printLog("input error, in findMiddlePointToGetChargingPath", true);
	if (pStart.xyEqual(pEnd))
		printLog("start and end point is same, infindMiddlePointToGetChargingPath", true);

	pStart.s = 0;
	pEnd.s = 0;
	goChargingPath.clear();
	continueTaskPath.clear();

	vector<DjiPoint> pieceLine = disperseFlightLineseq(pStart, pEnd, pieceLength);
	if (pieceLine.size() < 2)
	{
		printLog("pieceLine.size()<2, in findMiddlePointToGetChargingPath", true);
	}

	double comePointCost;
	double gohomeCost;
	int index = pieceLine.size() - 1;
	for (index = pieceLine.size() - 1; index >= 0; index--)
	{
		comePointCost = pointDistance(pieceLine[0], pieceLine[index]);
		gohomeCost = findNearestChargePoint(pieceLine[index], chargingStart, chargingEnd, nearCharge, goChargingPath);
		if (distLeft > comePointCost + gohomeCost)
			break;
	}
	if (goChargingPath.size() < 2)
	{
		printLog("askChargingError, in findMiddlePointToGetChargingPath", true);
	}

	for (int i = (int)goChargingPath.size() - 1; i >= 0; i--)
	{
		continueTaskPath.push_back(goChargingPath[i]);
	}
	continueTaskPath[continueTaskPath.size() - 1].s = 1;
	continueTaskPath.push_back(pEnd);

	if (index > 0)
	{
		vector<DjiPoint> task;
		task.push_back(DjiPoint(pStart, 1));
		task.insert(task.end(), goChargingPath.begin(), goChargingPath.end());
		goChargingPath = task;
	}
}


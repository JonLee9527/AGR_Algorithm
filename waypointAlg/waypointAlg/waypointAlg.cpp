// waypointAlg.cpp : Defines the entry point for the console application.
#include "stdafx.h"

#include "math.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "PPETask.h"
#include "edgeGenaration.h"
#include "debugIO.h"
#include "CoordTransform.h"
#include "djiConst.h"

#include "DjiMatD.h"


using namespace cv;
using namespace std;

static vector<vector<DjiPoint>> _forbidSeqPts;

static Mat mapImg;// esimate the map with a image
vector<DjiPoint>  _farmEdge;
static vector<DjiPoint> ptsCtrl;
static vector<DjiPoint> ptsShift;
static vector<DjiPoint> ptsAlt;

static FILE* fPts_mark = NULL;
static FILE* fPts_ctrl = NULL;
static FILE* fPts_shift = NULL;
static FILE* fPts_alt = NULL;

bool  opened_mark = false;
bool  opened_ctrl = false;
bool  opened_shift = false;
bool  opened_alt = false;

void onmouse(int event, int x, int y, int flags, void* param);
void getTestData();


void testOnAnroid()
{
	
	PPETask task = PPETask(DjiPoint(32.547662, 113.697039), 25, 5,3, true);

	vector<DjiPoint> farmEdge;
	farmEdge.push_back(DjiPoint(32.5469390000, 113.6943350000));
	farmEdge.push_back(DjiPoint(32.5455660000, 113.6941870000));
	farmEdge.push_back(DjiPoint(32.5459040000, 113.6966300000));
	farmEdge.push_back(DjiPoint(32.5476620000, 113.6970390000));
	task.addObsEdge(farmEdge, DJI_EDGE_FARMLAND);


// 	vector<DjiPoint> chargingline;
// 	chargingline.push_back(DjiPoint(32.5469390000, 113.6943350000));
// 	chargingline.push_back(DjiPoint(32.5455660000, 113.6941870000));
// 	task.addObsEdge(chargingline, DJI_LINE_CHARGING);


	vector<vector<DjiPoint> >taskSet=task.getMultDroneMission();
	for (int i = 0; i < taskSet.size(); i++)
	{
		printPoints(taskSet[i]);
	}
	

	getchar();
}


void testTransform()
{
	DjiPoint home = DjiPoint(39.82404, 116.5302);
	DjiPoint p1 = DjiPoint(39.0, 116.8);
	printPoint("p1_latlon", p1);
	CoordTransform tran = CoordTransform(home);
	DjiPoint p1_p = tran.latlong2plane(p1);
	printPoint("p1_plane", p1_p);
	DjiPoint p1_latlon = tran.plane2latlong(p1_p);
	printPoint("p1_latlon", p1_latlon);
	getchar();
}

double simulatePowerCost(double dist, double maxFlightDist, double minPowerPercent)
{
	//return prePower - dist / maxFlightDist *(100.0 - safeLandPercent);
	return dist / maxFlightDist *(100.0 - minPowerPercent);
	//dist = maxFlightDist*(prePower - powerLeft) / (100.0 - safeLandPercent);
}


void testOnWindows()
{
	getTestData();

	Mat testImg = Mat::zeros(512, 512, CV_8UC3);
	Mat imgSave = testImg.clone();

	vector<DjiPoint> farmEdge = _farmEdge;
	vector<DjiPoint> nsz_forbid = ptsCtrl;
	vector<DjiPoint> nsz_noSpray = ptsShift;
	vector<DjiPoint> nsz_raise = ptsAlt;

	drawImg(&testImg, farmEdge, COL_WHITE, CV_DRAW_CLOSED_POLYLINE, 0);
	drawImg(&testImg, nsz_forbid, COL_RED, CV_DRAW_CLOSED_POLYLINE, 0);
	drawImg(&testImg, nsz_noSpray, COL_SLIVER, CV_DRAW_CLOSED_POLYLINE, 0);
	drawImg(&testImg, nsz_raise, COL_YELLOW, CV_DRAW_CLOSED_POLYLINE, 1);

	//parameters setting
	DjiPoint homePt = DjiPoint(0, 511);
	PPETask ppeTask = PPETask(homePt, 0, 5, 3, false);

	ppeTask.addObsEdge(farmEdge, DJI_EDGE_FARMLAND);
	ppeTask.addObsEdge(ptsCtrl, DJI_EDGE_NSZ_AROUND);
	ppeTask.addObsEdge(ptsShift, DJI_EDGE_NSZ_OVER);
	ppeTask.addObsEdge(ptsAlt, DJI_EDGE_NSZ_HEIGHT);

	ppeTask.addObsEdge(farmEdge, DJI_LINE_CHARGING);

	/************************optional parameter setting***************************/

	//offset setting to avoid obstacles
	double safeOffset = 5;
	ppeTask.setSafeOffset(safeOffset);

	//offset setting to ensure full spraying on the farmland egde
	double sprayingOffset = 3;
	ppeTask.setSprayingOffset(sprayingOffset);

	double pieceLength = 3;
	//call this function before setChargingLine() for dispersing
	ppeTask.setPieceLength(pieceLength);

	//the max walk distance on the ground 
	double maxGroundWalk = 100;
	ppeTask.setMaxGroundWalk(maxGroundWalk);

	double landingDeviation = 25;
	ppeTask.setLandingDeviation(landingDeviation);

	//flight speed
	double flightSpeed = 3;
	ppeTask.setFlightSpeed(flightSpeed);

	//one battery endurance time at normal spraying speed with full pesticides
	double timeOneBattery = 1000;
	ppeTask.setTimeOneBattery(timeOneBattery);

	//the needed time to change battery one time 
	double timeToChangeBattery = 120;
	ppeTask.setTimeToChangeBattery(timeToChangeBattery);

	//the setted reserve power
	int minPowerPercent = 25;
	ppeTask.setMinPowerPercent(minPowerPercent);

	//the emergency landing power
	int safeLandPercent = 10;
	ppeTask.setSafeLandPercent(safeLandPercent);

	double maxFlightDist = flightSpeed*timeOneBattery;


	//*****************************Run Algorithm************************//
	vector<vector<DjiPoint> > taskSet = ppeTask.getMultDroneMission();

	

	clearMapImg();
	Mat imgOrg = testImg.clone();
	SprayTask sprayTask;
	vector<DjiPoint> waypoints;
	for (int i = 0; i < taskSet.size(); i++)
	{
		testImg = imgOrg.clone();
		waypoints = taskSet[i];
		const vector<DjiEdge* > aroundEdgesRef = ppeTask.getAllAroundEdges();
		vector<DjiEdge> aroundEdges;
		for (int j = 0; j < aroundEdgesRef.size();j++)
		{
			aroundEdges.push_back(*aroundEdgesRef[j]);
		}

		/*SprayTask(vector<DjiPoint> waypoints, vector<DjiPoint> chargingLine, vector<DjiEdge * > allAroundObsEdges
		, DjiPoint homePoint, double minPowerPercent,
		double maxFlightDist, double  maxGroundWalk,
		double landingDeviation,double pieceLength) 
		, bool isUseCoordTransform=true*/



		sprayTask = SprayTask
			(waypoints, ppeTask.getDisperatedChargingLine(), aroundEdges,
			homePt, minPowerPercent,
			maxFlightDist, maxGroundWalk,
			landingDeviation, pieceLength,false);

		imgSave = testImg.clone();
		drawImg(&testImg, waypoints, COL_YELLOW_GREEN, CV_DRAW_LINE_SPRAY, 1);
		testImg = imgSave.clone();


		//**********************Estimate the  battery changing process************************//
		imgSave = testImg.clone();
		drawImg(&testImg, sprayTask.getBatterySupplyArea(), COL_ORANGE_RED, CV_DRAW_POLYLINE, 1);
		testImg = imgSave.clone();

		vector<DjiPoint> batteryPoint;

		double powerLeft = 100;

		while (waypoints.size())
		{
			powerLeft = 100;
			for (int j = 0; j < waypoints.size(); j++)
			{
				if (j>0)
				{
					if (waypoints[j - 1].s == 1)
					{
						drawLine(&testImg, waypoints[j - 1], waypoints[j], COL_GREEN, 0);
					}
					if (waypoints[j - 1].s == 0)
					{
						drawLine(&testImg, waypoints[j - 1], waypoints[j], COL_BLUE, 0);
					}
					if (waypoints[j - 1].s == 2)//raise to avoid obstacles
					{
						drawLine(&testImg, waypoints[j - 1], waypoints[j], COL_ORANGE, 0);
					}

					powerLeft -= simulatePowerCost(pointDistance(waypoints[j - 1], waypoints[j]),maxFlightDist, minPowerPercent);
				}
				printf("powerLeft%f\n", powerLeft);

				if (sprayTask.askCharging(j, powerLeft))//need change battery
				{

					drawImg(&testImg, sprayTask.getChangeBatteryPath(), COL_WHITE, CV_DRAW_LINE_SPRAY, 0);

					imgSave = testImg.clone();
					drawImg(&testImg, sprayTask.getBatterySupplyArea(), COL_ORANGE_RED, CV_DRAW_POLYLINE, 1);
					testImg = imgSave.clone();

					printf("i=%d\n", j);

					waypoints = sprayTask.getResumeMissionTask();
					break;
				}
			}

		}
	}
	printf("结束,任意键退出\n");
	getchar();

}

int _tmain(int argc, _TCHAR* argv[])
{
	int a = 1; float b = 2; double c= 3;
	mapImg = Mat::zeros(512, 512, CV_8UC3);

	testOnWindows();

	//testOnAnroid();

	//testTransform();

}
void onmouse(int event, int x, int y, int flags, void* param)
{
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		if (!opened_mark)
		{
			fopen_s(&fPts_mark, "pts.txt", "w");
			opened_mark = true;
		}
		if (!opened_ctrl)
		{
			fopen_s(&fPts_ctrl, "ctrl.txt", "w");
			opened_ctrl = true;
		}
		if (!opened_shift)
		{
			fopen_s(&fPts_shift, "shift.txt", "w");
			opened_shift = true;
		}
		if (!opened_alt)
		{
			fopen_s(&fPts_alt, "alt.txt", "w");
			opened_alt = true;
		}


		if (flags == 1)
		{
			mapImg.at<Vec3b>(y, x) = Vec3b(255, 255, 255);
			_farmEdge.push_back(DjiPoint(x, y));
			fprintf(fPts_mark, "%lf\t%lf\n", (double)x, (double)y);
			imshow("mapImg", mapImg);
		}
		if (flags == 9)//ctrl
		{
			mapImg.at<Vec3b>(y, x) = Vec3b(0, 0, 255);
			ptsCtrl.push_back(DjiPoint(x, y));
			fprintf(fPts_ctrl, "%lf\t%lf\n", (double)x, (double)y);
			imshow("mapImg", mapImg);
		}
		if (flags == 17)//shift
		{
			mapImg.at<Vec3b>(y, x) = Vec3b(192, 192, 192);
			ptsShift.push_back(DjiPoint(x, y));
			fprintf(fPts_shift, "%lf\t%lf\n", (double)x, (double)y);
			imshow("mapImg", mapImg);
		}
		if (flags == 33)//alt
		{
			mapImg.at<Vec3b>(y, x) = Vec3b(0, 255, 255);
			ptsAlt.push_back(DjiPoint(x, y));
			fprintf(fPts_alt, "%lf\t%lf\n", (double)x, (double)y);
			imshow("mapImg", mapImg);
		}
	}
}

void getTestData()
{
	cvNamedWindow("mapImg", 0);
	cvSetMouseCallback("mapImg", onmouse);
	imshow("mapImg", mapImg);

	int recvKeyVal = cvWaitKey(0);
	cvSetMouseCallback("mapImg", NULL);

	if (fPts_mark != NULL)
	{
		fclose(fPts_mark);
	}
	if (fPts_ctrl != NULL)
	{
		fclose(fPts_ctrl);
	}
	if (fPts_shift != NULL)
	{
		fclose(fPts_shift);
	}
	if (fPts_alt != NULL)
	{
		fclose(fPts_alt);
	}

	if (27 == recvKeyVal)//ctrl
	{
		_farmEdge.clear();
		double  x;
		double  y;

		FILE* file_points = NULL;
		fopen_s(&file_points, "pts.txt", "rt");
		vector<double> apoint;
		while (2 == fscanf_s(file_points, "%lf%lf", &x, &y))
		{
			_farmEdge.push_back(DjiPoint(x, y));
		};
		fclose(file_points);

		FILE* file_ctrl = NULL;
		fopen_s(&file_ctrl, "ctrl.txt", "rt");
		while (2 == fscanf_s(file_ctrl, "%lf%lf", &x, &y))
		{
			ptsCtrl.push_back(DjiPoint(x, y));
		};
		fclose(file_ctrl);

		FILE* file_shift = NULL;
		fopen_s(&file_shift, "shift.txt", "rt");
		while (2 == fscanf_s(file_shift, "%lf%lf", &x, &y))
		{
			ptsShift.push_back(DjiPoint(x, y));

		};
		fclose(file_shift);

		FILE* file_alt = NULL;
		fopen_s(&file_alt, "alt.txt", "rt");
		while (2 == fscanf_s(file_alt, "%lf%lf", &x, &y))
		{
			ptsAlt.push_back(DjiPoint(x, y));
		};
		fclose(file_alt);
	}
	if (ptsCtrl.size())
	{
		ptsCtrl[0].h = -1;
		_forbidSeqPts.push_back(ptsCtrl);
	}
	if (ptsShift.size())
	{
		ptsShift[0].h = 0.0;
		_forbidSeqPts.push_back(ptsShift);
	}
	if (ptsAlt.size())
	{
		ptsAlt[0].h = 5;
		_forbidSeqPts.push_back(ptsAlt);
	}
}


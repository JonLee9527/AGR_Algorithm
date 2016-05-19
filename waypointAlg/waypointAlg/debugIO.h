#ifndef DJI_WAYPOINT_Agriculture_Debug_IO_Helper
#define DJI_WAYPOINT_Agriculture_Debug_IO_Helper




#include "math.h"

#include "djiPoint.h"
#include "djiConst.h"

#include <string>

#include<stdio.h> 

#include <stdarg.h> 

// #ifdef DJI_RUN_ON_ANDROID_WAYPOINT_ALG
// #define PR(X,...)    /*do nothing */
// #else
// #define PR(X,...) printf(X,__VA_ARGS__)

//#endif


class djiRgb
{
public:

	int r;
	int g;
	int b;

	djiRgb(int _r, int _g, int _b)
	{
		r = _r;
		g = _g;
		b = _b;
	}

};

#define COL_RED       djiRgb(255,0, 0)   
#define COL_GREEN      djiRgb(0, 255, 0)
#define COL_BLUE       djiRgb(0, 0, 255)
#define COL_WHITE      djiRgb(255,255,255)
#define COL_YELLOW     djiRgb(255,255,0)
#define COL_ORANGE     djiRgb(255,165,0)
#define COL_ORANGE_RED djiRgb(255,69,0)
#define COL_PINK       djiRgb(255,192,203)
#define COL_OLIVE       djiRgb(128,128,0)
#define COL_PURPLE      djiRgb(128,0,128)
#define COL_SLIVER      djiRgb(192,192,192)
#define COL_YELLOW_GREEN   djiRgb(154,205,50)


#define  CV_DRAW_POINTS  0
#define  CV_DRAW_POLYLINE 1
#define  CV_DRAW_CLOSED_POLYLINE 2
#define  CV_DRAW_LINE_SPRAY 3

void drawImg(const vector<DjiPoint>& pts, djiRgb rgb = COL_WHITE, int drawType = CV_DRAW_POLYLINE, bool isShow = false);
void drawImg(void* img, const vector<DjiPoint>& pts, djiRgb rgb = COL_WHITE, int drawType = CV_DRAW_POLYLINE, bool isShow = false);

void drawPoint(DjiPoint pt, djiRgb rgb = COL_WHITE, bool isShow = false);
void drawPoint(void* img, DjiPoint pt, djiRgb rgb = COL_WHITE, bool isShow = false);

void drawLine(DjiPoint pt1, DjiPoint pt2, djiRgb rgb = COL_WHITE, bool isShow = false);
void drawLine(void* img,DjiPoint pt1, DjiPoint pt2, djiRgb rgb = COL_WHITE, bool isShow = false);

void clearMapImg();
void setMapImg(void* const img);

void printLog(const string s, bool isError = false);

void printPoint(string name, DjiPoint pt);
void printPoint(DjiPoint pt);
void printPoints(vector<DjiPoint> pts);



#endif
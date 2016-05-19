
#include "stdafx.h"


#include "debugIO.h"

#include "djiUtils.h"



#ifdef DJI_RUN_ON_ANDROID_WAYPOINT_ALG

#include "waypoint.h"

void printLog(const string s, bool isError )
{
	isError ? LOGE(s.c_str()) : LOGI(s.c_str());
}

void clearMapImg()
{
}
void setMapImg(void* const img)
{
}

void drawImg(const vector<DjiPoint>& pts, djiRgb rgb, int drawType, bool isShow)
{
}

void drawImg(void* img, const vector<DjiPoint>& pts, djiRgb rgb, int drawType, bool isShow)
{
}

void drawPoint(DjiPoint pt, djiRgb rgb, bool isShow)
{
}
void drawPoint(void* const img, DjiPoint pt, djiRgb rgb, bool isShow)
{
}

void drawLine(void* const img, DjiPoint pt1, DjiPoint pt2, djiRgb rgb, bool isShow)
{
}

void drawLine(DjiPoint pt1, DjiPoint pt2, djiRgb rgb, bool isShow)
{
}

#else
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#define dontShowImgInPPETask 0

static Mat mapImg = Mat::zeros(512, 512, CV_8UC3);

void clearMapImg()
{
	if (dontShowImgInPPETask)
	{
		return;
	}
	mapImg = Mat::zeros(512, 512, CV_8UC3);
}
void setMapImg(void* const img)
{
	if (dontShowImgInPPETask)
	{
		return;
	}
	// 	Mat const * p = (Mat *)img;
	// 	mapImg = (*p).clone();
}

void drawImg(const vector<DjiPoint>& pts, djiRgb rgb, int drawType, bool isShow)
{
	if (dontShowImgInPPETask)
	{
		return;
	}

	drawImg(&mapImg, pts, rgb, drawType, isShow);
}

void drawImg(void* img, const vector<DjiPoint>& pts, djiRgb rgb, int drawType, bool isShow)
{
	if (dontShowImgInPPETask)
	{
		return;
	}
	
	CvScalar scalar = CV_RGB(rgb.r, rgb.g, rgb.b);
	switch (drawType)
	{
	case CV_DRAW_POINTS:
		for (int i = 0; i < (int)pts.size(); i++)
		{
			circle(*(Mat*)img, Point2f((float)pts[i].x, (float)pts[i].y), 3, scalar, 1, 8, 0);
		}

	case CV_DRAW_POLYLINE:
		for (int i = 0; i < (int)pts.size() - 1; i++)
		{
			line(*(Mat*)img, Point2f((float)pts[i].x, (float)pts[i].y), Point2f((float)pts[i + 1].x, (float)pts[i + 1].y), scalar, 1, 8, 0);
		}
		break;
	case CV_DRAW_CLOSED_POLYLINE:
		if (pts.size() < 2)
		{
			break;
		}
		for (int i = 1; i < (int)pts.size(); i++)
		{
			line(*(Mat*)img, Point2f((float)pts[i - 1].x, (float)pts[i - 1].y), Point2f((float)pts[i].x, (float)pts[i].y), scalar, 1, 8, 0);
		}
		line(*(Mat*)img, Point2f((float)pts[0].x, (float)pts[0].y), Point2f((float)pts[pts.size() - 1].x, (float)pts[pts.size() - 1].y), 
			scalar, 1, 8, 0);
		break;
	case CV_DRAW_LINE_SPRAY:
// 		if (pts.size() >= 2)
// 		{
// 			drawPoint(img, pts[0], COL_GREEN, 0);
// 			drawPoint(img, pts[pts.size() - 1], COL_RED, 0);
// 		}
		for (int i = 1; i < (int)pts.size(); i++)
		{
			if (pts[i - 1].s == 1)
			{
				drawLine(img, pts[i - 1], pts[i], COL_GREEN, 0);
			}
			else if (pts[i - 1].s == 0)
			{
				drawLine(img, pts[i - 1], pts[i], COL_BLUE, 0);
			}
			else if (pts[i - 1].s == 2)//°Î¸ß
			{
				drawLine(img, pts[i - 1], pts[i], COL_ORANGE, 0);
			}
			else
			{
				drawLine(img, pts[i - 1], pts[i], COL_RED, 0);
			}
		}
		break;
	default:
		break;
	}
	imshow("mapImg", *(Mat*)img);
	if (isShow)
	{
		cvWaitKey(0);
	}
}


void drawPoint(DjiPoint pt, djiRgb rgb, bool isShow)
{
	if (dontShowImgInPPETask)
	{
		return;
	}

	drawPoint(&mapImg, pt, rgb, isShow);
}
void drawPoint(void* const img, DjiPoint pt, djiRgb rgb, bool isShow)
{
	if (dontShowImgInPPETask)
	{
		return;
	}
	CvScalar scalar = CV_RGB(rgb.r, rgb.g, rgb.b);
	circle(*(Mat*)img, Point2f((float)pt.x, (float)pt.y), 3, scalar, 1, 8, 0);
	imshow("mapImg", *(Mat*)img);
	if (isShow)
	{
		cvWaitKey(0);
	}
}


void drawLine(void* const img, DjiPoint pt1, DjiPoint pt2, djiRgb rgb, bool isShow)
{
	if (dontShowImgInPPETask)
	{
		return;
	}

	CvScalar scalar = CV_RGB(rgb.r, rgb.g, rgb.b);
	line(*(Mat*)img, Point2f((float)pt1.x, (float)pt1.y), Point2f((float)pt2.x, (float)pt2.y), scalar, 1, 8, 0);
	imshow("mapImg", *(Mat*)img);
	if (isShow)
	{
		cvWaitKey(0);
	}
}

void drawLine(DjiPoint pt1, DjiPoint pt2, djiRgb rgb, bool isShow)
{
	if (dontShowImgInPPETask)
	{
		return;
	}
	drawLine(&mapImg, pt1, pt2, rgb, isShow);
}


void printLog(const string s, bool isError)
{
	printf(s.c_str());
	printf("\n");
	if (isError)
	{
		throw new exception;
		exit(110);
	}
}

#endif // DJI_RUN_ON_ANDROID


void printPoint(string name, DjiPoint pt)
{
	printLog(name + " " + double2string(pt.x) + ", " + double2string(pt.y)+" s="+int2string(pt.s));
}
void printPoint(DjiPoint pt)
{
	printLog(double2string(pt.x) + ", " + double2string(pt.y)+" s=" + int2string(pt.s));
}

void printPoints(vector<DjiPoint> pts)
{
	printLog(" ");
	printLog("points size ="+int2string(pts.size()));
	for (int i = 0; i < pts.size();i++)
	{
		printPoint(pts[i]);
	}
	printLog(" ");
}

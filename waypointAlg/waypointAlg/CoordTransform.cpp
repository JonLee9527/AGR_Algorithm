
#include "stdafx.h"


#include "math.h"

#include "CoordTransform.h"

using namespace std;


#define USED_X_AXIS 0
#define USED_Y_AXIS 1
#define USED_Z_AXIS 2


CoordTransform::CoordTransform()
{
	isInit = false;

}
CoordTransform::CoordTransform(DjiPoint home)
{
	if (!isLegalLatLong(home))
	{
		printPoint(home);
		printLog("latLong is inlegal in CoordTransform()", true);
		
	}
		
	latitudeOrigin = home.x;
	longtitudeOrigin = home.y;

	//使用地球半径  这样改坐标系下单位为米
	earthRadius = EARTH_RADIUS_AVER;

	home_w = latlong2world(home);
	//home_w.print();

	getTransferMats();

	isInit = true;
}


void CoordTransform::getTransferMats()
{
	//rotate lontitude degree around Z axis
	MatD Rz = MatD(4, 4, 0.0);
	Rz(2, 2) = Rz(3, 3) = 1.f;
	Rz(0, 0) = Rz(1, 1) = cos(longtitudeOrigin*TO_RADIAN);
	Rz(0, 1) = sin(longtitudeOrigin*TO_RADIAN);
	Rz(1, 0) = -sin(longtitudeOrigin*TO_RADIAN);

	//printMatrix("Rz", Rz);

	//rotate (90-longtitude) degree around Y axis
	MatD Ry = MatD(4, 4, 0.0);
	Ry(1, 1) = Ry(3, 3)  = 1.f;
	Ry(0, 0) = Ry(2, 2) = cos((90 - latitudeOrigin)*TO_RADIAN);
	Ry(0, 2) = -sin((90 - latitudeOrigin)*TO_RADIAN);
	Ry(2, 0) = sin((90 - latitudeOrigin)*TO_RADIAN);

	//printMatrix("Ry", Ry);

	//be carefal with the rotation order
	MatD rotMat = Ry*Rz;
	
	//the transfer vector is calculated in the new coordinate after rotations !
	rotMat(2, 3) = -1;

	/*@brief actually is a south_east coordinate after rotations and transfer
	*/
	World2Plane = rotMat;
	//printMatrix("World2Plane", World2Plane);
	
	Plane2World = rotMat.inv();
	//printMatrix("Plane2World", Plane2World);
}
MatD CoordTransform::latlong2world(DjiPoint latLong)
{
	if (!isLegalLatLong(latLong))
	{
		printPoint(latLong);
		printLog("latLong is inlegal in latlong2world()", true);
		
	}

	MatD pt_w = MatD(4, 1, 1);
	double lat = latLong.x;
	double lon = latLong.y;
	pt_w(0, 0) = cos(lat*TO_RADIAN)*cos(lon*TO_RADIAN);
	pt_w(1, 0) = cos(lat*TO_RADIAN)*sin(lon*TO_RADIAN);
	pt_w(2, 0) = sin(lat*TO_RADIAN);
	return pt_w;
}
DjiPoint CoordTransform::latlong2plane(DjiPoint latLong)
{
// 	if (DJI_USE_COORDINATE_TRANSFORM == 0)
// 		return latLong;
	if (!isLegalLatLong(latLong))
	{
		printPoint(latLong);
		printLog("latLong is inlegal in latlong2plane()", true);
		
	}
	
	if (!isInit)
	{
		printLog("doesnt init CoordTrasfer", true);
		return DjiPoint();
	}
	MatD pt_w = latlong2world(latLong);
	//printMatrix("pt_w", pt_w);
	MatD pt_p = World2Plane*pt_w;
	//printMatrix("pt_p", pt_p);

	//turn the plane coordinate unit into metre 
	pt_p = pt_p*earthRadius;  //将平面坐标系内单位 转化为米

	//project on plane, the third value forced to zero
	return DjiPoint(pt_p(0,0), pt_p(1,0));
}
vector<DjiPoint> CoordTransform::latlong2plane(vector<DjiPoint> latLongs)
{
	vector<DjiPoint> pts_p = latLongs;
	for (int i = 0; i < (int)pts_p.size(); i++)
	{
		pts_p[i] = latlong2plane(pts_p[i]);
	}
	return pts_p;
}

DjiPoint CoordTransform::plane2latlong(DjiPoint pt)
{
// 	if (DJI_USE_COORDINATE_TRANSFORM == 0)
// 		return pt;
	if (!isInit)
	{
		printLog("doesnt init CoordTrasfer", true);
		return DjiPoint();
	}

	pt = pt / earthRadius;
		
	MatD pt_p = MatD(4, 1, 0.0);
	//the plane coordinate decribed in 3_d, but the third value can be ignored
	pt_p(0, 0) = pt.x;
	pt_p(1, 0) = pt.y;
	pt_p(3, 0) = 1.0;

	//printMatrix("pt_p", pt_p);
	//Plane2World
	MatD pt_w = Plane2World*pt_p;
	//printMatrix("pt_w before intersect with sphere", pt_w);

	//calculate intersection between earth x2+y2+z2=1 and line constructed with point pt_w and vec home_vec
	double x0 = pt_w(0, 0);
	double y0 = pt_w(1, 0);
	double z0 = pt_w(2, 0);

	double nx = home_w(0, 0);
	double ny = home_w(1, 0);
	double nz = home_w(2, 0);

	//if nx==0 use a simple transform
	int used = simpleTransform(x0,y0,z0,nx,ny,nz);

	if (nx == 0)
	{
		printLog("nx==0 in plane2latlong", true);
		return DjiPoint();
	}
		
	//mutiply nx both the side to avoide zero divisor
	double a = pow(nx,2) + pow(ny, 2) + pow(nz,2);
	double b = 2 * ny * (nx*y0 - ny*x0) + 2 * nz * (nx*z0 - nz*x0);
	double c = pow(nx*y0 - ny*x0, 2) + pow(nx*z0 - nz*x0, 2) - pow(nx,2);
	
	double s = b*b - 4 * a*c;
	if (s < 0)
	{
		printLog("no intersection b*b - 4 * a*c < 0  in plane2latlong", false);
		return DjiPoint();
	}
		
	s = sqrt(s);


	//quadratic formula
	double x1 = (-b - s) / (2 * a);
	double y1 = ny / nx * (x1 - x0) + y0;
	double z1 = nz / nx*(x1 - x0) + z0;
	double dist1 = pow(x1 - x0, 2) + pow(y1 - y0, 2) + pow(z1 - z0, 2);

	double x2 = (-b + s) / (2 * a);
	double y2 = ny / nx * (x2 - x0) + y0;
	double z2 = nz / nx*(x2 - x0) + z0;
	double dist2 = pow(x2 - x0, 2) + pow(y2 - y0, 2) + pow(z2 - z0, 2);
   
	if (dist1<dist2)
	{
		pt_w(0, 0)=x1;
		pt_w(1, 0)=y1;
		pt_w(2, 0)=z1;
	}
	else
	{
		pt_w(0, 0) = x2;
		pt_w(1, 0) = y2;
		pt_w(2, 0) = z2;
	}

	//if used simple transform,  need inverse transform
	simpleTransformInv(pt_w(0, 0), pt_w(1, 0), pt_w(2, 0),used);
	
	//printMatrix("pt_w", pt_w);
	if (abs(pow(pt_w(0, 0), 2) + pow(pt_w(1, 0), 2) + pow(pt_w(2, 0), 2)-1)>ZERO_ABS)
	{
		printLog("x2+y2+z2!=1", true);
	}
	
	//world to latlong
	//latitde scope(-90 90)   longitude scope(-180 180)
	double lat = asin(pt_w(2, 0));
	double lon=0;
	if (abs(cos(lat)) > ZERO_ABS)
	{
		lon = acos(pt_w(0, 0) / cos(lat));
		if (pt_w(1, 0) / cos(lat) < 0)
			lon = -lon;

	}
	//computer radian to angle degree
	DjiPoint result = DjiPoint(lat / TO_RADIAN, lon / TO_RADIAN,pt.s);
	if (!isLegalLatLong(result))
	{
		printPoint(result);
		printLog("latLong is inlegal in plane2latlong()", true);
		
	}
	return result;
}

vector<DjiPoint> CoordTransform::plane2latlong(vector<DjiPoint> pts_p)
{
	vector<DjiPoint> latLongs = pts_p;
	for (int i = 0; i < (int)latLongs.size(); i++)
	{
		latLongs[i] = plane2latlong(latLongs[i]);
	}
	return latLongs;
}


int CoordTransform::simpleTransform(double& x0, double& y0, double& z0, double& nx, double& ny, double& nz)
{
	int used = USED_X_AXIS;
	if (abs(nx) < ZERO_ABS)
	{
		used = USED_Y_AXIS;
		if (abs(ny) < ZERO_ABS)
		{
			used = USED_Z_AXIS;
			if (abs(nz) < ZERO_ABS)
			{
				printLog("home_vec == 0 !", true);
			}
		}
	}
	if (used == USED_Y_AXIS)
	{
		double temp = x0;
		x0 = y0;
		y0 = temp;

		temp = nx;
		nx = ny;
		ny = temp;
	}
	if (used == USED_Z_AXIS)
	{
		double temp = x0;
		x0 = z0;
		z0 = temp;

		temp = nx;
		nx = nz;
		nz = temp;
	}
	return used;
}
void CoordTransform::simpleTransformInv(double& x, double& y, double& z, int used)
{
	if (used == USED_Y_AXIS)
	{
		double temp = x;
		x = y;
		y = temp;
	}
	if (used == USED_Z_AXIS)
	{
		double temp = x;
		x = z;
		z = temp;
	}
}


DjiVec CoordTransform::dirSouthEastFromNorthAngle(double angle)
{
	//the North describe in North_East coordinate is (1,0)  
	//but (-1，0）when described in South_East coordinate !
	MatD north = MatD(2, 1, 0.0);
	north(0, 0) = -1.0;

	// positive rotation: from positive direction of X axis to positive direction of Y axis
	double sita=-angle;
	sita = sita*TO_RADIAN;
	MatD rot = MatD(2, 2, 0.0);
	rot(0, 0) = rot(1, 1) = cos(sita);
	rot(0, 1) = -sin(sita);
	rot(1, 0) = sin(sita);
	MatD dir = rot*north;
	return DjiVec(dir(0,0),dir(1,0));
}
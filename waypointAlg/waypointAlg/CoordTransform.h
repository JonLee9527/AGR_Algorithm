#ifndef dji_waypointAlg_coordinate_transfer
#define dji_waypointAlg_coordinate_transfer

#include "debugIO.h"
#include "djiPoint.h"
#include "djiConst.h"

#include "DjiMatD.h"


class CoordTransform
{
private:

	bool isInit;// if the class was constructed with a point , isInit is true

	double latitudeOrigin;// lantitude of the tangent point 

	double longtitudeOrigin;// longtitude of the tangent point 

	//Unit conversion to metre
	double earthRadius;

	MatD  home_w;//3_dimension vector descirbe in earth coordinate

	MatD Plane2World;// transform plane coordinate to earth coordinate

	MatD World2Plane;// transform earth coordinate to  plane coordinate

public:
	CoordTransform();

	/**
	 @param home The point to genarate a Tangent plane of earth
	*/
	CoordTransform(DjiPoint home);

	/*@brief Transform latitude_longtitude on earth sphere to a tangent plane coordinates
	 @brief the unit of plane coordinate is metre 
	 @param latLong Consist of latitude and longtitude
	*/
	DjiPoint latlong2plane(DjiPoint latLong);
	vector<DjiPoint> latlong2plane(vector<DjiPoint> latLongs);

	/*@brief Transform tangent plane coordinates to latitude_longtitude on earth sphere
	  actually the destination coordinate is a 2_dimension south_east coordinate
	@param Plane A point on tangent plane of home point
	*/
	DjiPoint plane2latlong(DjiPoint plane);
	vector<DjiPoint> plane2latlong(vector<DjiPoint> pts_p);

public:
	/*@brief get the spray direction in south_east coordinates after transform
	@param northAngle Clockwise angle with north direction in the real world
	*/
	static DjiVec dirSouthEastFromNorthAngle(double northAngle);

private:
	/*@brief get two transform matrix----Plane2World and World2Plane
	*/
	void getTransferMats();

	/*@brief transform latitude_longtitude into world(3_d earth) coordinate
	@param latLong  Latitude and longtitude
	*/
	MatD latlong2world(DjiPoint latLong);

	
	/*@brief avoiding zero divisor with a simple transform
	*/
	int simpleTransform(double& x0, double& y0, double& z0, double& nx, double& ny, double& nz);
	void simpleTransformInv(double& x, double& y, double& z, int used);
};
#endif // !dji_waypointAlg_coordinate_transfer

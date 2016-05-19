#ifndef  WAPOINT_ALGORI_DJICONST
#define WAPOINT_ALGORI_DJICONST



//**********************if run on android, comment the next line********************************///

//#define DJI_RUN_ON_ANDROID_WAYPOINT_ALG

//********************************************************************************************//




//earth maxmum radius is 6378.38km, average radius is 6371km, 1m = 1/6371000.0*radius
#define EARTH_RADIUS_AVER  6371000.0

#define PI  3.141592653589793238462643383
#define  TO_RADIAN (PI/180.0)
#define  ZERO_ABS 0.00000000001

#define DIVIDE_MAX_DIST_NO_SPRAY  10.0

#define  MAX_SPRAY_WIDTH_METER 10.0
#define  MAX_CHARGING_DIVID_METER 10.0
#define  MIN_CHARGING_DIVID_METER 1.0

#define LANDING_DEVI_FACTOR (1.0/4.0)

#define MIN_DIST_TO_DELETE 3.0


#define INTERSEC_MARKED_WITH_POINT_S 0
#define INTERSEC_MARKED_WITH_POINT_INDEX 1

//#define  DIVIDE_PARA 5
//#define  TURN_PARA 3

//#define DJI_USE_COORDINATE_TRANSFORM 1
#endif

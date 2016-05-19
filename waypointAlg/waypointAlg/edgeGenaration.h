#ifndef Dji_gen_edge_method
#define Dji_gen_edge_method

#include "djiPoint.h"
#include "CoordTransform.h"
#include "djiConst.h"

#include <vector>
using namespace std;


/*@brief Interface
@param p Center point
@param radius Obstacle radius
*/
vector<DjiPoint> getEdge(DjiPoint p, double radius);
vector<DjiPoint> getEdge(vector<DjiPoint> polylinePts, double radius);

//static void calOneDirection(const vector<DjiPoint>& _polylinePts, double offset, vector<DjiPoint>& resPts);
//static vector<DjiPoint> initWithSiglePoint(DjiPoint pts, double radius);

#endif // !Dji_gen_edge_method

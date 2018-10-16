#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <math.h>
#include "spline.h"

using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  // calculation of degree to radians
  double deg2rad(double x);

  // Calculate the distance between 2 points
  double distance(double x1, double y1, double x2, double y2);
  
  // find the closest Waypoint
  int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
  
  // find the next Waypoint
  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  // Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

  // Plan the path
  vector<pair <double, double> > planPath(vector<double> car, auto previous_path_x, auto previous_path_y, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, int lane);

};

#endif /* TOOLS_H_ */

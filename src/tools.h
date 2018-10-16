#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <math.h>
#include "spline.h"

using namespace std;

class Tools {
public:

  int lane; // initially car is on lane the middle lane 1 - lane 0 is the left lane - lane 2 is the right lane
  double ref_vel;  // reference velocitiy in mph

  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  ~Tools();

  // init
  void init();

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

  // Sensor Fusion
  bool sensorFusion(vector< vector<double> > sensor_fusion, vector<double> car, int prev_size);
  
  // Plan the path
  vector<pair <double, double> > planPath(vector<double> car, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s);

};

#endif /* TOOLS_H_ */

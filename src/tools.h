#ifndef TOOLS_H_
#define TOOLS_H_

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

  // Calculate the distance between 2 points
  double distance(double x1, double y1, double x2, double y2);
  
  // find the closest Waypoint
  int Tools::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
  
  // find the next Waypoint
  int Tools::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double> Tools::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  // Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> Tools::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


};

#endif /* TOOLS_H_ */

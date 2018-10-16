// Helper functions for Path Planning Project

#include "tools.h"

using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// -------------------------------------------------------------------------------------------
// Calculate the distance between 2 points
double Tools::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}



// -------------------------------------------------------------------------------------------
// find the closest Waypoint
int Tools::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}



// -------------------------------------------------------------------------------------------
// find the next Waypoint
int Tools::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*M_PI - angle, angle);

  if(angle > M_PI/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}



// -------------------------------------------------------------------------------------------
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Tools::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}



// -------------------------------------------------------------------------------------------
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Tools::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


// -------------------------------------------------------------------------------------------
// Plan the path
vector<pair <double, double> > Tools::planPath(vector<double> car, auto previous_path_x, auto previous_path_y, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s)
{
	int prev_size = previous_path_x.size();
	
	// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
	// Later we will interpolate these waypoints with a spline and fill it in with more points
	vector<double> ptsx;
	vector<double> ptsy;
		
	// reference x,y, yaw state
	double ref_x = car[0];
	double ref_y = car[1];
	double ref_yaw = deg2rad(car[4]);

	// if previous size is almost empty, use the car as starting reference
	if(prev_size < 2) {
	  //use 2 points, that make the path tangent to the car
	  double prev_car_x = car[0] - cos(car[4]);
	  double prev_car_y = car[1] - sin(car[4]);

	  ptsx.push_back(prev_car_x);
	  ptsx.push_back(car[0]);

	  ptsy.push_back(prev_car_y);
	  ptsy.push_back(car[1]);

	}
	// use the previous path's end point as starting reference
	else {
	  // redefine reference state as previous path end point
	  ref_x = previous_path_x[prev_size-1];
	  ref_y = previous_path_y[prev_size-1];

	  double ref_x_prev = previous_path_x[prev_size-2];
	  double ref_y_prev = previous_path_y[prev_size-2];
	  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

	  // use two points that make the path tangent to the previous path's end point
	  ptsx.push_back(ref_x_prev);
	  ptsx.push_back(ref_x);

	  ptsy.push_back(ref_y_prev);
	  ptsy.push_back(ref_y);
	}

	// in frenet add evenly 30m spaced points ahead of the starting reference
	vector<double> next_wp0 = tools.getXY(car[2]+30,(2+4*lane),map_waypoints_s,
							map_waypoints_x,map_waypoints_y);
	vector<double> next_wp1 = tools.getXY(car[2]+60,(2+4*lane),map_waypoints_s,
							map_waypoints_x,map_waypoints_y);
	vector<double> next_wp2 = tools.getXY(car[2]+90,(2+4*lane),map_waypoints_s,
							map_waypoints_x,map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	// ----------------------------
	// Convert ptsx and ptsy to local coordinate system
	for(int i=0; i<ptsx.size(); i++) {
	  // shift car reference angle to 0 degrees
	  double shift_x = ptsx[i]-ref_x;
	  double shift_y = ptsy[i]-ref_y;

	  ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
	  ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
	}		

	// create a spline
	tk::spline s;

	/*
	// sort ptsx and ptsy for set_points
	// source: www.quora.com/How-do-I-sort-array-of-pair-int-int-in-C++-according-to-the-first-and-the-second-elemen
	vector<pair <double, double> > pts;
	for(int i=0; i<ptsx.size(); i++) {
	  cout << "ptsx before: " << ptsx[i] << endl;
	  pts.push_back(make_pair(ptsx[i], ptsy[i]));
	}
	sort(pts.begin(),pts.end());
	for(int i=0; i<pts.size(); i++) {
	  ptsx[i] = pts[i].first;
	  cout << "ptsx after: " << ptsx[i] << endl;
	  ptsy[i] = pts[i].second;
	}*/

	// set (x,y) points to the spline
	s.set_points(ptsx, ptsy);

	// Define the actual (x,y) points we will use for the planner
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	// Start with all of the previous path points from last time
	for(int i=0; i<previous_path_x.size(); i++) {
	  next_x_vals.push_back(previous_path_x[i]);
	  next_y_vals.push_back(previous_path_y[i]);
	}

	// Calculate how to break up sline points so that we travel at our desired
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));

	double x_add_on = 0;

	// Fill up the rest of our path planner after filling it with previous points
	for(int i = 1; i <= 50-previous_path_x.size(); i++) {
	  double N = (target_dist/(0.02*ref_vel/2.24));
	  double x_point = x_add_on + (target_x)/N;
	  double y_point = s(x_point);

	  x_add_on = x_point;

	  double x_ref = x_point;
	  double y_ref = y_point;

	  // rotate back to normal after rotating it earlier
	  x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
	  y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

	  x_point += ref_x;
	  y_point += ref_y;

	  next_x_vals.push_back(x_point);
	  next_y_vals.push_back(y_point);
	}

	vector<pair <double, double> > next_vals;
	for(int i=0; i<next_x_vals.size(); i++) {
	  next_vals.push_back(make_pair(next_x_vals[i],next_y_vals[i]));
	}
	
	return next_vals;
}

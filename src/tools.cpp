// Helper functions for Path Planning Project

#include "tools.h"
#include <iostream>

using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// -------------------------------------------------------------------------------------------
void Tools::init() {
  this->lane = 1;
  this->ref_vel = 0.0;
}


// -------------------------------------------------------------------------------------------
// TODO
double Tools::deg2rad(double x)
{
	return x * M_PI / 180;
}

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
// Sensor Fusion
void Tools::sensorFusion(vector< vector<double> > sensor_fusion, vector<double> car, int prev_size)
{
	double check_speed = 49.8;

	// Step 1: Calculate all possible lanes ------------------------------------------
	// These are the current, and maybe the left and/or right lane
	vector<int> check_lanes; // all possible lanes will be written into the vector
	int number_of_lanes = 3; // total number of lanes in one direction
	for (int i=0; i<number_of_lanes;i++) {
	  // check if the lane is the current lane, or left, or right of the current lane
	  if(i == (lane-1) || i == lane || i == (lane+1)) {
	    check_lanes.push_back(i); // write lane into check_lanes
	  }
	}

	// Step 2: Get the distances to other cars in the possible lanes -----------------
	vector<double> lane_distances; // for the shortest distances to other cars of each lane
	vector<double> lane_vel; // for the velocity of the nearest car
	// initally set the shortest distance to 500.0 and the velocity to 0.0 for each lane
	for (int i=0; i<check_lanes.size();i++) {
	  lane_distances.push_back(500.0);
	  lane_vel.push_back(0.0);
	}
	
	// Go through all detected cars in sensor fusion
	for (int i=0; i< sensor_fusion.size(); i++) {
	  // get the distance from the middle of the street to see in which lane the car is
	  float d = sensor_fusion[i][6];
	  // Go through all possible lanes
	  for (int l=0; l<check_lanes.size(); l++) {
	    // Check if the checked car is in the lane
	    if(d < (2+4*check_lanes[l]+2) && d > (2+4*check_lanes[l]-2) ) {
	      // calculate the velocity of the car
	      double vx = sensor_fusion[i][3];
	      double vy = sensor_fusion[i][4];
	      check_speed = sqrt(pow(vx,2)+pow(vy,2));
	      // get the s value of the car
	      double check_car_s = sensor_fusion[i][5];
	      // add the driven s value in 0.02 seconds to get the estimated current position
	      check_car_s += ((double)prev_size*0.02*check_speed);
	      // check if car is in front of my car
	      if(check_car_s > car[2]) {
	        // calculate the distance to my car in s direction
		double dist = check_car_s-car[2];
		// if the distance is shorter, than to another detected car in this lane
		// set this distance as the shortest
		if (dist<lane_distances[l]) {
		  lane_distances.begin()[l] = dist;
		  lane_vel.begin()[l] = check_speed;
		}
	      }
	      // check if a lane change will be save for changing
	      if (check_car_s > (car[2]-8) && check_car_s < (car[2]+5) && l != lane) {
	        // there is a car next to my car in the lane
	        // so this lane is no candidate for lane change
	        lane_distances.begin()[l] = 0.0;
	        cout << "THERE IS A CAR NEXT TO MINE IN LANE " << l << endl; 
	      }
	    }
	  }
	}
	
	// Step 3: Choose the best lane ---------------------------------------------------
	// find dinstance and velocity in current lane
	double dist_current_lane;
	double vel_current_lane;
	for (int i=0; i<check_lanes.size();i++) {
	  if (check_lanes[i] == lane) {
	    dist_current_lane = lane_distances[i];
	    vel_current_lane = lane_vel[i];
	  }
	}
	
	double best_distance = 30; // initally set the best distance to 30
	int best_lane = lane;
	// check if there is a other car in the distance of 30 in the current lane
	// otherwise the current lane will be the best lane
	if (dist_current_lane < best_distance) {
	  best_distance = dist_current_lane; // set the best distance to the distance in the current lane
	  // check the other lanes
	  for (int i=0; i<lane_distances.size(); i++) {
	    if (check_lanes[i] != lane) {
	      double dist = lane_distances[i];
	      // check if the distance in the other lane is better than the distance in the best lane
	      if (dist > best_distance) {
	        best_distance = dist; // set the best distance to the distance in this lane
		best_lane = check_lanes[i]; // set this lane as the best lane
	      }
	    }
	  }
	}

	// Step 4: Change velocity if there is a car in front and no lane change is possible
	// Only when ther is no lane change
	if (best_lane == lane) {
	  double max_delta = 0.224; // calculated from the max acceleration
	  double new_vel;
	  double max_vel = 49.5; // max allowed velocity
	  // If the distance is shorter than 5 reduce velocity
	  if (dist_current_lane<5 ) {
	    ref_vel -= max_delta;
	  }
	  // if the distance is shorter than 25
	  else if (dist_current_lane < 25) {
	    // if the velocity is lower than mine
	    if(vel_current_lane < ref_vel) {
	      ref_vel -=max_delta; // reduce my velocity
	    }
	    // if the velocity is higher than mine and my velocity is lower than the max velocity
	    else if (ref_vel < max_vel) {
	      ref_vel += max_delta; // increase my velocity
	    }
	  }
	  // if the distance is 25 or higher and my velocity is lower than the max velocity
	  else if (ref_vel < max_vel) {
	    ref_vel += max_delta; // increase my velocity
	  }  
	}
	// otherwise set the best lane as current lane
	else {
	  lane = best_lane;
	}
}


// -------------------------------------------------------------------------------------------
// Plan the path
vector<pair <double, double> > Tools::planPath(vector<double> car, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s)
{
	//Tools tools;
	//ref_vel = 49.5;
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
	vector<double> next_wp0 = getXY(car[2]+30,(2+4*lane),map_waypoints_s,
							map_waypoints_x,map_waypoints_y);
	vector<double> next_wp1 = getXY(car[2]+60,(2+4*lane),map_waypoints_s,
							map_waypoints_x,map_waypoints_y);
	vector<double> next_wp2 = getXY(car[2]+90,(2+4*lane),map_waypoints_s,
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

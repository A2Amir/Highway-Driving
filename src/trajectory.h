#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include "helpers.h"


// for convenience
using std::string;
using std::vector;

// for convenience
using std::string;
using std::vector;


vector<vector<double>> trajectoryPlanner(int lane, double ref_vel, double car_x, double car_y, double car_yaw, double car_s, double car_d,
                                              const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                              vector<double> previous_path_x, vector<double> previous_path_y )
{

  int prev_size = previous_path_x.size();
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  vector<double> ptsx;
  vector<double> ptsy;
  // find two points in the past
  if ( prev_size < 2 ) {    //Use two points thats makes path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }else{
    //Redefine the reference point to previous point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  // find another 3 points in the future
  for(int i = 1; i <4; i++){
    vector<double> next_wp = getXY(car_s + 30*i, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
  }
  // shift to car coordinates
  for(int i=0; i <ptsx.size(); i++){
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw));
    ptsy[i] = (shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw));
  }
  // build the spline based on the 5 points
  tk::spline s;
  s.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  //For the smooth transition, we are adding previous path points
  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for( int i = 1; i < 50 - prev_size; i++ ) {

    double N = target_dist/(0.02*ref_vel);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    //Rotate back to normal after rotating it earlier
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  vector<vector<double>> next_vals;
  next_vals.push_back(next_x_vals);
  next_vals.push_back(next_y_vals);

  return next_vals;
}


#endif  // TRAJECTORY_H

#ifndef BEHAVOIR_H
#define BEHAVOIR_H

#include <math.h>
#include <string>
#include <vector>
#include "helpers.h"

#include "spline.h"

// for convenience
using std::string;
using std::vector;



vector<std::pair<int, double>> behaviour_planning(bool car_left, bool car_right, bool car_front, int lane, double ref_vel, double front_speed){
  vector<std::pair<int, double>> successor_lanes_ref_vels;
  if(car_front){
    successor_lanes_ref_vels.push_back(std::make_pair(lane, ref_vel - 5 * 0.02));
    if(!car_left)
      successor_lanes_ref_vels.push_back(std::make_pair(std::max(lane - 1, 0), std::min(49.5/2.24, ref_vel + 5 * 0.02)));
    if(!car_right)
      successor_lanes_ref_vels.push_back(std::make_pair(std::min(lane + 1, 2), std::min(49.5/2.24, ref_vel + 5 * 0.02)));
  }

  if(!car_front){
    successor_lanes_ref_vels.push_back(std::make_pair(lane, std::min(49.5/2.24, ref_vel + 8 * 0.02)));
  }
  return successor_lanes_ref_vels;
}


double calculate_cost(int lane, double car_speed, int successor_lane, double successor_ref_vel){

  double cost_lane_change = std::abs(lane - successor_lane)/3;
  // cost of acceleration
  double acc = (successor_ref_vel - car_speed)/0.02;
  double cost_acc = 0;
  if(acc<-9.8 || acc >9.8)
    cost_acc = 1;
  else
    cost_acc = std::abs(acc)/9.8;

  //   cost of speed
  double target_speed = 49.5/2.24;
  double cost_speed = 0;
  if(successor_ref_vel < target_speed)
    cost_speed = 0.8 * (target_speed - successor_ref_vel) / target_speed;
  else{
    if(car_speed > 50)
      cost_speed = 1;
    else
      cost_speed = (successor_ref_vel - target_speed) / 0.5;
  }


  return 10*cost_lane_change + 10*cost_acc + 5*cost_speed;
}

#endif  // BEHAVOIR_H

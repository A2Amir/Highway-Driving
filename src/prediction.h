#ifndef PREDICTION_H
#define PREDICTION_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include "helpers.h"


// for convenience
using std::string;
using std::vector;



vector<double> get_predictions(vector<vector<double>> sensor_fusion, int prev_size, double car_s, int lane)
{
  double car_left = 0;
  double car_front = 0;
  double car_right = 0;
  double front_speed;
  for(int i = 0; i < sensor_fusion.size(); i++){
    double check_car_s = sensor_fusion[i][5];
    // filter the cars that are too far from the ego car #The data format for each car is: [ id, x, y, vx, vy, s, d]
    if(abs(check_car_s - car_s) < 100){
      double check_car_d = sensor_fusion[i][6];
      int check_car_lane = get_lane(check_car_d);
      // check if the car in front is too close
      double check_car_vx = sensor_fusion[i][3];
      double check_car_vy = sensor_fusion[i][4];
      double check_car_speed = sqrt(pow(check_car_vx, 2) + pow(check_car_vy, 2));
      check_car_s = check_car_s + prev_size * 0.02 * check_car_speed;
      if(check_car_lane == lane){
        if((check_car_s > car_s) && (check_car_s < car_s + 30)){
          car_front = 1;
          front_speed = check_car_speed; // m/s
        }
        
      } else {
        if((check_car_lane == lane - 1) && (((check_car_s > car_s) && (check_car_s - car_s)) <(40 ||  (check_car_s < car_s) && (car_s - check_car_s ) < 30))|| lane == 0)
          car_left = 1;
        if((check_car_lane == lane + 1) && (((check_car_s > car_s) && (check_car_s - car_s)) < (40 ||  (check_car_s < car_s) && (car_s - check_car_s ) < 30))|| lane == 2)
          car_right = 1;
      }
    }
  }
  vector<double> pred {car_left, car_right, car_front, front_speed};
  
  return pred;
}

#endif  // HELPERS_Hcd

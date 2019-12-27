#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <algorithm>    // std::max
#include <string>
#include <vector>
#include <utility>      // std::pair, std::make_pair
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "prediction.h"
#include "behavoir.h"
#include "trajectory.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  int lane = 1;
  double ref_vel = 0;
  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2') {

                  auto s = hasData(data);

                  if (s != "") {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry") {
                      // j[1] is the data JSON object

                      // Main car's localization Data
                      double car_x = j[1]["x"];
                      double car_y = j[1]["y"];
                      double car_s = j[1]["s"];
                      double car_d = j[1]["d"];
                      double car_yaw = j[1]["yaw"];
                      double car_speed = j[1]["speed"];

                      // Previous path data given to the Planner
                      auto previous_path_x = j[1]["previous_path_x"];
                      auto previous_path_y = j[1]["previous_path_y"];
                      // Previous path's end s and d values
                      double end_path_s = j[1]["end_path_s"];
                      double end_path_d = j[1]["end_path_d"];

                      // Sensor Fusion Data, a list of all other cars on the same side
                      //   of the road.The data format for each car is: [ id, x, y, vx, vy, s, d]
                      vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

                      int prev_size = previous_path_x.size();
                      if(prev_size >0)
                        car_s = end_path_s;
                      vector<double> predictions= get_predictions(sensor_fusion, prev_size, car_s, lane);
                      // Behaviour planning
                      bool car_left = predictions[0];
                      bool car_right = predictions[1];
                      bool car_front = predictions[2];
                      double front_speed = predictions[3];
                      std::cout<< "current state: " <<lane << "   " << ref_vel << std::endl;
                      std::cout<< "predictions: " << "car_left:" << car_left<< ", car_right: "<< car_right<<  ", car_front:" <<car_front <<std::endl;
                      std::vector<std::pair<int, double>> successors = behaviour_planning(car_left, car_right, car_front, lane, ref_vel, front_speed);
                      // choose the least cost trajectory
                      double lowest_cost = 999999;
                      for(int i = 0; i < successors.size(); i++){

                        int successor_lane = successors[i].first;
                        double successor_ref_vel = successors[i].second;
                        // calculate cost
                        double cost = calculate_cost(lane, car_speed, successor_lane, successor_ref_vel);
                        // get the lowest cost and corresponding trajectory
                        if(cost < lowest_cost){
                          lowest_cost = cost;
                          lane = successor_lane;
                          ref_vel = successor_ref_vel;
                        }
                      }

                      std::cout<< "successors size:<<<<<<<<<< " << successors.size() << std::endl;
                      std::cout<< "successor 0:<<<<<<<<<< " << successors[0].first<<"   " << successors[0].second << std::endl;
                      std::cout<< "best successor choice:  " << lane << "     " << ref_vel << std::endl;
                      if(successors.size()>1){
                        std::cout<< "successor 1:<<<<<<<<<< " << successors[1].first << successors[1].second << std::endl;

                      }
                      // Trajectory generation
                      vector<vector<double>> best_trajectory = trajectoryPlanner(lane, ref_vel, car_x, car_y, car_yaw, car_s, car_d,
                                                                                      map_waypoints_s, map_waypoints_x, map_waypoints_y,
                                                                                      previous_path_x, previous_path_y);
                      /**
           * generate the best path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
                      json msgJson;
                      msgJson["next_x"] = best_trajectory[0];
                      msgJson["next_y"] = best_trajectory[1];

                      auto msg = "42[\"control\","+ msgJson.dump()+"]";

                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }  // end "telemetry" if
                  } else {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                }  // end websocket if
              }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}

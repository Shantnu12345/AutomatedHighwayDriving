#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "LaneChangerFSM.hpp"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  MapData mapData;
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  LaneChangerFSM laneChangerFSM;
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
    mapData.waypoints_x.push_back(x);
    mapData.waypoints_y.push_back(y);
    mapData.waypoints_s.push_back(s);
    mapData.waypoints_dx.push_back(d_x);
    mapData.waypoints_dy.push_back(d_y);
  }

  h.onMessage([&mapData, &laneChangerFSM]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //std::cout << "Message!!!" << std::endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          //cout<<"Message"<<endl;
          // j[1] is the data JSON object
          Pose pose;
          // Main car's localization Data
          pose.x = j[1]["x"];
          pose.y = j[1]["y"];
          pose.s = j[1]["s"];
          pose.d = j[1]["d"];
          pose.yaw = deg2rad(j[1]["yaw"]);
          pose.spd = mph_to_mps(j[1]["speed"]);

          // Previous path data given to the Planner
          auto previous_path_x  = j[1]["previous_path_x"];
          auto previous_path_y  = j[1]["previous_path_y"];

          Path prev;
          prev.xpts.reserve(previous_path_x.size());
          prev.ypts.reserve(previous_path_x.size());
          for(int i=0; i<previous_path_x.size(); i++)
          {
            prev.xpts.emplace_back(previous_path_x[i]);
            prev.ypts.emplace_back(previous_path_y[i]);
          }        
          
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          vector<Pose> cars;
          cars.reserve(sensor_fusion.size());
          for(auto const& carData:sensor_fusion)
          {
            Pose p;
            p.x = carData[0];
            p.y = carData[1];
            p.vx = carData[2];
            p.vy = carData[3];
            p.s = carData[4];
            p.d = carData[5];
            cars.emplace_back(p);
          }

          json msgJson;
          cout<<endl<<"H1"<<endl;
          Path next = laneChangerFSM.findNextPath(pose, mapData, prev, cars); //Index0: x values, Index1: y values 
          msgJson["next_x"] = next.xpts;
          msgJson["next_y"] = next.ypts;

          // vector<double> next_xpts;
          // vector<double> next_ypts;
          // msgJson["next_x"] = next_xpts;
          // msgJson["next_y"] = next_ypts;

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
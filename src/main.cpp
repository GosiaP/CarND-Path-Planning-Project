#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "map.h"
#include "traffic_planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

#define MY_WINDOWS (1)

int main() {
  uWS::Hub h;


  HighwayMap higwayMap("../data/highway_map.csv");
  TrafficPlanner trafficPlanner(higwayMap);

#if MY_WINDOWS
  h.onMessage([&higwayMap, &trafficPlanner](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode)
#else
  h.onMessage([&higwayMap, &trafficPlanner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
#endif
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // traffic snapshot
          Traffic traffic(j[1]);
          // provide ego car path (trajectory)
          Path path = trafficPlanner.getEgoCarPath(traffic);
        
          //  std::cout << "test..." << std::endl;

          json msgJson;
      
          msgJson["next_x"] = path.x;
          msgJson["next_y"] = path.y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
        #if MY_WINDOWS
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        #else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        #endif
          
        }
      } 
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        #if MY_WINDOWS
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        #else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        #endif
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

#if MY_WINDOWS
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req)
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
#endif
  {
    std::cout << "Connected!!!" << std::endl;
  });

#if MY_WINDOWS
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length)
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
#endif
  {
  #if MY_WINDOWS
      ws->close();
  #else
      ws.close();
  #endif
    std::cout << "Disconnected" << std::endl;
  });

#if MY_WINDOWS
  int port = 4567;
  auto host = "127.0.0.1";
  if (h.listen(host, port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
#else
  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
#endif

  h.run();
}

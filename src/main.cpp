#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"]; //The global x positions of the waypoints
          vector<double> ptsy = j[1]["ptsy"]; // The global y positions of the waypoints
          double px = j[1]["x"]; // Cars X position in the global mapping reference
          double py = j[1]["y"]; // Cars Y position in the global mapping reference
          double psi = j[1]["psi"]; //he orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions
          //       90
          //  180     0/360
          //      270
          
          double v = j[1]["speed"]; //The current velocity in mph.
          double delta= j[1]["steering_angle"];
          double a = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          const int actuator_delay = 100; //ms
          const double delay = 0.1; //s

          // transform waypoints to be from car's perspective
          // this means we can consider px = 0, py = 0, and psi = 0
          // greatly simplifying future calculations
          std::vector<float> waypoints_x;
          std::vector<float> waypoints_y;
          for (size_t i = 0; i < ptsx.size(); i++) 
          {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            waypoints_x.push_back(dx * cos(-psi) - dy * sin(-psi));
            waypoints_y.push_back(dx * sin(-psi) + dy * cos(-psi));
          }
          
          /*
          static vector<float> ptsx_avg;
          static vector<float> ptsy_avg;
          static int count;
          if (count <2)
          {
              count++;
              ptsx_avg.push_back(waypoints_x);
              ptsy_avg.push_back(waypoints_y);
          }
          else
          {
              ptsx_avg.push_back(waypoints_x);
              ptsy_avg.push_back(waypoints_y);
          }
          */
          

          

          // convert from vector<double> to Eigen::VectorXd to match input to polyfit
          Eigen::VectorXd ptsx_v (waypoints_x.size());
          Eigen::VectorXd ptsy_v (waypoints_y.size());

          size_t shift_count = 0;
          for (size_t i = 0; i < ptsx.size()-shift_count; i++)
          {
            ptsx_v(i) = waypoints_x[i+shift_count];
            ptsy_v(i) = waypoints_y[i+shift_count];
          }
          
          
          #ifdef INTERP_SECOND
          auto coeffs = polyfit(ptsx_v, ptsy_v, 2);
          #else
          auto coeffs = polyfit(ptsx_v, ptsy_v, 3);
          #endif
          // Calculate cross track error
          //double cte;
          //double epsi;
         
          double cte0 = coeffs[0];

          //Calculate error in angle. Since we changed the coordinates, px=0, and then f'(x0) = coeffs[1]
          double epsi0 = - atan(coeffs[1]);
          double x0 = 0;
          double y0 = 0;
          double psi0 = 0;
   
          // Update states
          double x_state = x0 + ( v * cos(psi0) * delay );
          double y_state = y0 + ( v * sin(psi0) * delay );
          double psi_state = psi0 - ( v * delta * delay / 2.67 );
          double v_state = v + a * delay;
          double cte_state = cte0 + ( v * sin(epsi0) * delay );
          double epsi_state = epsi0 - ( v * atan(coeffs[1]) * delay /2.67 );
          Eigen::VectorXd state(6);
          state << x_state, y_state, psi_state, v_state, cte_state, epsi_state; //px = 0; py = 0 and psi = 0, because the axis were changed to the car's coordinate
          //state << x0,y0,psi0,v,cte0,epsi0;
          #ifdef PRINT_STATE
          cout << "STATE:"<< state<< endl;
          #endif
          auto varsMPCsolve = mpc.Solve(state, coeffs);
          

          //State vector
          
          
          #ifdef PRINT_VARS
          std::cout <<"----------PRINT VARS----------"<< std::endl;
          std::cout <<"coeffs: "<< coeffs << std::endl;
          std::cout <<"cte: "<< cte << std::endl;
          std::cout <<"state: "<< state << std::endl;
          std::cout <<"--------------------"<< std::endl;
          std::cout << "x = " << varsMPCsolve[0] << std::endl;
          std::cout << "y = " << varsMPCsolve[1] << std::endl;
          std::cout << "psi = " << varsMPCsolve[2] << std::endl;
          std::cout << "v = " << varsMPCsolve[3] << std::endl;
          std::cout << "cte = " << varsMPCsolve[4] << std::endl;
          std::cout << "epsi = " << varsMPCsolve[5] << std::endl;
          std::cout << "delta = " << varsMPCsolve[6] << std::endl;
          std::cout << "a = " << varsMPCsolve[7] << std::endl;
          /*
          std::cout <<"py"<< py << std::endl;
          std::cout <<"psi"<< psi << std::endl;
          std::cout <<"v"<< v << std::endl;
          std::cout <<"ptsx"<< ptsx << std::endl;
          std::cout <<"py"<< ptsy << std::endl;
          */
          #endif

          double steer_value;
          double throttle_value;

          steer_value = varsMPCsolve[0]/deg2rad(25);
          throttle_value = varsMPCsolve[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for ( size_t i = 2; i < varsMPCsolve.size(); i++ )
          {
            if ( i % 2 == 0 ) 
            {
              mpc_x_vals.push_back( varsMPCsolve[i] );
            } else
            {
              mpc_y_vals.push_back( varsMPCsolve[i] );
            }

          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          double poly_inc = 2.5;
          int num_points = 25;

          for ( int i = 0; i < num_points; i++ )
          {
              double x = poly_inc * i;
              next_x_vals.push_back( x );
              next_y_vals.push_back( polyeval(coeffs, x) );
            }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(actuator_delay));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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

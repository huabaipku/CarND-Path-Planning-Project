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
#include "spline.h"
#include "util.h"
#include "vehicle.h"

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  Vehicle ego_car = Vehicle(-1); //Initialize ego car

  h.onMessage([&ego_car, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
        if (event == "telemetry") {

            ///============== PART I: Get and update the data for Ego Vehicle ===================///
            std::cout << "*********==== Ego Car Data ====*********" << std::endl;
        
            // j[1] is the data JSON object
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
            double car_yaw_rad = deg2rad(car_yaw);
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
            ego_car.update_vehicle(car_x, car_y, car_yaw_rad, car_speed, car_s, car_d, 0.0);
            ego_car.update_waypoints_cache(previous_path_x, previous_path_y);
            std::cout << "==ego_car updated" << std::endl;

            std::cout << "==ego_car waypoints cache size: " << ego_car._cache_size << std::endl;
            int cache_size = ego_car._cache_size;

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
            std::cout << " end_path_s:" << end_path_s << std::endl;

            ///========= PART II: Get and update the data for other background Vehicle ==========///
            std::cout << "========== Background Car Data ===========" << std::endl;
            
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            map< int, vector< Vehicle > > bg_car_map; // map of (index and trajs of the car)
            for(int i = 0; i < sensor_fusion.size(); i++){
                int bg_car_id = sensor_fusion[i][0]; // for backgroud (bg) car
                double bg_car_d = sensor_fusion[i][6];
                if ( bg_car_d / 4 < 0 ) { continue;} // some vehicles hasn't been put on track yet.

                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double bg_car_speed = sqrt(vx*vx + vy*vy);
                double bg_car_s = sensor_fusion[i][5];
                
                vector<double> bg_car_xy = getXY(bg_car_s, bg_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                Vehicle bg_vehicle = Vehicle(bg_car_id, bg_car_xy[0], bg_car_xy[1], 0.0, bg_car_speed, bg_car_s, bg_car_d, 0.0, "CS");
                vector < Vehicle > bg_car_traj = bg_vehicle.generate_predictions(map_waypoints_x, map_waypoints_y, map_waypoints_s, 2, 2); // 50 waypoint away, and 100 waypoints away.
                bg_car_map.insert( pair<int, vector< Vehicle >>(bg_car_id, bg_car_traj) );
            }

            ///========= PART III: Behavior planning for the Ego Vehicle ==========///
            double future_s = ego_car._s;
            if(cache_size > 0){
                future_s = end_path_s;
            }

            bool too_close = false;
            bool change_lane = false;
            int target_lane = ego_car._lane;

            vector<string> lane_status = {"FREE", "FREE", "FREE"};
            lane_status[ego_car._lane] = "SELF";

            std::cout << "======== Loop through backgroud vehicles ========" << std::endl;

	        for ( auto it = bg_car_map.begin(); it != bg_car_map.end(); it++) {
				int each_id = it->first;
				vector< Vehicle > each_car_traj = it->second;
			//	std::cout<< "== Car id: " << each_car_traj[0]._id << " current s: " << each_car_traj[0]._s << " which lane: " << each_car_traj[0]._lane << std::endl;
			//	std::cout<< "how many predicted points: " << each_car_traj.size() << std::endl;
                int which_lane = each_car_traj[1]._lane;

                if ((each_car_traj[1]._s - future_s < 50 && each_car_traj[1]._s > future_s) || (each_car_traj[0]._s - ego_car._s < 50 && each_car_traj[0]._s > ego_car._s)) {
                    if (ego_car._lane == which_lane) {
                        if (each_car_traj[1]._s - future_s < 30 || each_car_traj[0]._s - ego_car._s < 30 ) {
                            std::cout << " SLOW DOWN! follow the slow car" << std::endl;
                            too_close = true;
                            change_lane = true;
                        }
                    }
                    else {
                        lane_status[which_lane] = "BUSY";
                    }
                }
                else if ((future_s > each_car_traj[1]._s && future_s - each_car_traj[1]._s < 10) || (ego_car._s > each_car_traj[0]._s && ego_car._s - each_car_traj[0]._s < 10)) {
                    lane_status[which_lane] = "BUSY";
                }
			}

            /*
            std::cout << " lane statue: " << lane_status[0] << std::endl;
            std::cout << " lane statue: " << lane_status[1] << std::endl;
            std::cout << " lane statue: " << lane_status[2] << std::endl;
            */

            int current_lane_record = ego_car._lane;
            if(change_lane){
                if(ego_car._lane == 0){ 
                    if (lane_status[1] == "FREE") { ego_car._lane = 1;}
                    else {ego_car._lane =0; }
                }
                else if(ego_car._lane == 2){ 
                    if (lane_status[1] == "FREE") { ego_car._lane = 1;}
                    else {ego_car._lane =2; }
                }
                else if(ego_car._lane == 1){ 
                    if (lane_status[0] == "FREE") { ego_car._lane = 0;}
                    else if (lane_status[2] == "FREE") { ego_car._lane = 2;}
                    else {ego_car._lane = 1; }
                }
            }

            double target_speed_mph = ego_car._target_speed * 3600 / 1609.34;
            double increment = 0.5; // m/s 

            if(too_close){
                ego_car._target_speed -= 0.1;

            }
            else if(target_speed_mph < 45){
                ego_car._target_speed += 0.1;
            }


            ///========= PART IV: Spline path planning for the Ego Vehicle ==========///
            double x1_for_spline;
            double x2_for_spline;
            double y1_for_spline;
            double y2_for_spline;

            if(cache_size < 2){
                x1_for_spline = car_x - 30*cos(car_yaw_rad);
                y1_for_spline = car_y - 30*sin(car_yaw_rad);

                x2_for_spline = car_x;
                y2_for_spline = car_y;
            } else {

                x1_for_spline = previous_path_x[cache_size - 2];
                y1_for_spline = previous_path_y[cache_size - 2];

                x2_for_spline = previous_path_x[cache_size - 1];
                y2_for_spline = previous_path_y[cache_size - 1];
            }

            // check three other points for fiting spline
            // use s d coordinates

            double s3_for_spline = future_s + 30;
            double d3_for_spline = 2+4*ego_car._lane;
            double s4_for_spline = future_s + 60;
            double d4_for_spline = 2+4*ego_car._lane;
            double s5_for_spline = future_s + 90;
            double d5_for_spline = 2+4*ego_car._lane;

            vector<double> xy3_for_spline = getXY(s3_for_spline, d3_for_spline, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> xy4_for_spline = getXY(s4_for_spline, d4_for_spline, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> xy5_for_spline = getXY(s5_for_spline, d5_for_spline, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            vector<double> x_pts_for_spline;
            vector<double> y_pts_for_spline;

            x_pts_for_spline.push_back(x1_for_spline);
            x_pts_for_spline.push_back(x2_for_spline);
            x_pts_for_spline.push_back(xy3_for_spline[0]);
            x_pts_for_spline.push_back(xy4_for_spline[0]);
            x_pts_for_spline.push_back(xy5_for_spline[0]);

            y_pts_for_spline.push_back(y1_for_spline);
            y_pts_for_spline.push_back(y2_for_spline);
            y_pts_for_spline.push_back(xy3_for_spline[1]);
            y_pts_for_spline.push_back(xy4_for_spline[1]);
            y_pts_for_spline.push_back(xy5_for_spline[1]);

            double ref_x = x_pts_for_spline[1];
            double ref_y = y_pts_for_spline[1];
            double ref_yaw = car_yaw_rad;

            for (int i = 0; i < x_pts_for_spline.size(); i++){
                double shift_x = x_pts_for_spline[i]-ref_x;
                double shift_y = y_pts_for_spline[i]-ref_y;
                x_pts_for_spline[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                y_pts_for_spline[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

            tk::spline sp;
            sp.set_points(x_pts_for_spline, y_pts_for_spline);

            /// set next road points
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            vector<double> next_s_vec;
            vector<double> next_d_vec;

            double interval = ego_car._target_speed * 0.02;

            /// spline method
            for(int i=0; i < previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = 60.0;
            double target_y = sp(target_x);
            double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
            double x_add_on = 0;
            double N = target_dist/interval;

            for (int i = 1; i <= 50 - previous_path_x.size(); i++){
                double x_point = x_add_on + (target_x)/N;
                double y_point = sp(x_point);
                x_add_on = x_point;

                double global_x_point = (x_point * cos(ref_yaw) - y_point * sin(ref_yaw));
                double global_y_point = (x_point * sin(ref_yaw) + y_point * cos(ref_yaw));
                global_x_point += ref_x;
                global_y_point += ref_y;
                next_x_vals.push_back(global_x_point);
                next_y_vals.push_back(global_y_point);
            }


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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

#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <cmath>
#include <iterator>

#include "util.h"

class Vehicle {
public:
  int _id; // self: -1
  double _x; // global x
  double _y; // global x
  double _yaw;
  double _speed; // speed or sqrt(vx*vx + vy*vy)
  double _target_speed; 
  double _s; // Frenet s
  double _d; // Frenet d
  int _lane; // left: 0; middle 1; right 2;
  int _target_lane; //default current lane

  double _acceleration; // placeholder for other vehicles
  string _current_state; // for non_ego_car, will be "CS", (current_CS)
  vector<string> _available_states;
  vector<double> _waypoints_cache_x; // previous waypoints cache, just for ego car
  vector<double> _waypoints_cache_y; // previous waypoints cache, just for ego car
  int _cache_size;

  /**
  * Constructors
  */
  Vehicle();
  Vehicle(int id);
  Vehicle(int id, double x, double y, double yaw, double speed, double s, double d, double acceleration = 0.0, string state="CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_vehicle(double x, double y, double yaw, double speed, double s, double d, double acceleration);
  void update_waypoints_cache(vector<double> cache_x, vector<double> cache_y);

  double position_at(double t);
  vector<Vehicle> generate_predictions(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, double horizon=2, int size=10);

};



/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}
Vehicle::Vehicle(int id){ _id = id;}
Vehicle::Vehicle(int id, double x, double y, double yaw, double speed, double s, double d, double acceleration, string state){
    _id = id;
    _x = x;
    _y = y;
    _yaw = yaw; 
    _speed = speed;
    _s = s;
    _d = d;
    _lane = d / 4;
    _current_state = state;
    _acceleration = acceleration;
}
Vehicle::~Vehicle() {}


void Vehicle::update_vehicle(double x, double y, double yaw, double speed, double s, double d, double acceleration){
    _x = x;
    _y = y;
    _yaw = yaw; 
    _speed = speed;
    _s = s;
    _d = d;
    _lane = d / 4;
    _acceleration = acceleration;
}


void Vehicle::update_waypoints_cache(vector<double> cache_x, vector<double> cache_y){
    _cache_size = cache_x.size();
    _waypoints_cache_x = cache_x;
    _waypoints_cache_y = cache_y;
}

double Vehicle::position_at(double t) {
    return _s + _speed*t + _acceleration*t*t/2.0;
}

vector<Vehicle> Vehicle::generate_predictions(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, double horizon, int size) {
	vector<Vehicle> trajectory;
    Vehicle current_vehicle = Vehicle(_id, _x, _y, _yaw, _speed, _s, _d, _acceleration, _current_state);
    trajectory.push_back(current_vehicle);
    for(int i = 0; i < size; i++) {
        double future_time = (horizon / size ) * (i+1);
        double next_s = _s + _speed*future_time + _acceleration*future_time*future_time/2.0;
        vector<double> future_xy = getXY(next_s, _d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        Vehicle future_vehicle = Vehicle(_id, future_xy[0], future_xy[1], _yaw, _speed, next_s, _d, 0.0, "CS");
        trajectory.push_back(future_vehicle);
  	}
    return trajectory;
}


#endif

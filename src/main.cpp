/*
*Most of this program was supplied or given by the walkthrough
*/

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono> //clock stuff
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"//for the smoothing of the waypoints
#include "cost.h" //i should rename this to helper or something

using namespace std;

const double TARGET_SPEED = 49.5; //49.5 so we don't speed
const double VELOCITY_CHANGE_DECEL = 0.3; //velocity deceleration increment
const double VELOCITY_CHANGE_ACCEL = 0.3; //velocity acceleration increment

int lane = 1; //current lane target
int oldLane = 1; //lane previous to current lane target
double ref_vel = 0; //mph we wish to travel at
bool laneChanging = false; //is the car in the process of changing lanes
double lookingForwardMeters = 75.0; //how far my car's of interest look ahead
double lookingBackwardsMeters = 40.0; //how far my car's of interest look behind
double accel_sTarget = 0.0; //for easier acceleration if approching a car


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; } //very nice helpers
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}
//closest waypoint could be behind you, watch out
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = min(2 * pi() - angle, angle);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//only need to worry about x,y,theta
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return { frenet_s,frenet_d };

}

// Transform from Frenet s,d coordinates to Cartesian x,y
//only need to worry about s and d
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return { x,y };

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
		map_waypoints_x.push_back(x); //waypoint's x coordinate
		map_waypoints_y.push_back(y); //waypoint's y coordinate
		map_waypoints_s.push_back(s); //distance in meters of waypoint
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
		uWS::OpCode opCode) {
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
					// j[1] is the data JSON object

					// Main car's localization Data //Start to pay attention here
					double car_x = j[1]["x"]; //car's x position in map coordinates
					double car_y = j[1]["y"]; //car's y position in map coordinates
					double car_s = j[1]["s"]; //car's s position in frenet
					double car_d = j[1]["d"]; //car's d position in frenet
					double car_yaw = j[1]["yaw"]; //car's yaw angle, front end facing in the map
					double car_speed = j[1]["speed"]; //car's speed in mph

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"]; //previous list of x, minus the processed values
					auto previous_path_y = j[1]["previous_path_y"]; //previous list of y, minus the processed values
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"]; //previous list last s value, where car currently is
					double end_path_d = j[1]["end_path_d"]; //previous list last d value, where car currenlty is

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					//2d vector with, [ID, x in map, y in map, x velocity, y velocity, s in frenet, d in frenet]
					//probably use for a bunch of stuff
					auto sensor_fusion = j[1]["sensor_fusion"];

					int prev_size = previous_path_x.size(); //# of points we have left over from the previous list, transition helper

					if (prev_size > 0)
					{
						car_s = end_path_s; //where car will be
					}


					bool too_close = false; //if we are too close to a car ahead, we slow down, if not speed up
					accel_sTarget = 100.0; //
					
					vector<vector<double> > lane0CarsOfInterestAhead; //will hold all of the cars that i care about 75m ahead
					vector<vector<double> > lane1CarsOfInterestAhead;
					vector<vector<double> > lane2CarsOfInterestAhead;

					vector<vector<double> > lane0CarsOfInterestBehind; //will hold all of the cars that i care about 40m behind
					vector<vector<double> > lane1CarsOfInterestBehind;
					vector<vector<double> > lane2CarsOfInterestBehind;

					//find ref_v to use
					for (int i = 0; i < sensor_fusion.size(); i++)
					{
						//list of what is on the road and their velocities
						vector<double> tempCar; //hold the speed, s, and d values for each car that I care about
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_speed = sqrt(vx * vx + vy * vy); //how fast is the car going
						double check_car_s = sensor_fusion[i][5]; //where is the car on longitude
						//car is in my lane
						float d = sensor_fusion[i][6];
						//cout << "CAR ID: " << sensor_fusion[i][0] << "	s: " << check_car_s << "	d: " << d << "	vel: " << check_speed << endl;

						//Push the values that I car about into tempCar
						tempCar.push_back(check_speed); 
						tempCar.push_back(check_car_s);
						tempCar.push_back(d);

						//Checks if the tempCar is a car that I care about ahead or behind
						if (check_car_s >= car_s && check_car_s < (car_s + lookingForwardMeters))
						{
							//push important info 
							if (d >= 0 && d < 4.0) //car is in left lane
							{
								lane0CarsOfInterestAhead.push_back(tempCar);
							}
							else if (d >= 4.0 && d < 8.0) //car is in the middle lane
							{
								lane1CarsOfInterestAhead.push_back(tempCar);
							}
							else if (d >= 8.0) //car is in the right lane
							{
								lane2CarsOfInterestAhead.push_back(tempCar);
							}
						}
						else if (check_car_s < car_s && check_car_s > (car_s - lookingBackwardsMeters))
						{
							//push important info down
							if (d >= 0 && d < 4.0) //car is in left lane
							{
								lane0CarsOfInterestBehind.push_back(tempCar);
							}
							else if (d >= 4.0 && d < 8.0) //car is in the middle lane
							{
								lane1CarsOfInterestBehind.push_back(tempCar);
							}
							else if (d >= 8.0) //car is in the right lane
							{
								lane2CarsOfInterestBehind.push_back(tempCar);
							}
						}

						//car is in my lane and we need to slow down
						if (d < (car_d + 2) && d >(car_d - 2) && !laneChanging) //we need to slow down now
						{
							check_car_s += ((double)prev_size * 0.02 * check_speed);					
							//check s values greater than mine and s gap
							if ((check_car_s > car_s) && ((check_car_s - car_s) < 20)) //crude came up on slow car //20 worked and 25
							{
								//Do some logic here, change lanes or slow down
								too_close = true;
							}
							else if ((check_car_s > car_s) && ((check_car_s - car_s) < 50)) //ease up on the accelerator
							{
								accel_sTarget = check_car_s; 
							}
						}
					}
					//MPC would work great here
					if (too_close && ref_vel > VELOCITY_CHANGE_DECEL) //makes sure that we don't go negative
					{
						ref_vel -= VELOCITY_CHANGE_DECEL;
					}
					else if (!too_close && ref_vel < TARGET_SPEED) //49.5
					{
						if (accel_sTarget < 50) //ease the accelerator
						{
							ref_vel += (accel_sTarget - car_s) * 0.006; //for traffic, don't rush upto car infront
						}
						else
							ref_vel += VELOCITY_CHANGE_ACCEL;
					}

					oldLane = lane;

					if (!laneChanging && car_speed > 15) //If we are not in the process of changing lanes, see if we sould be
					{

						//find the best lane to be in
						int bestLane = BestLane(lane0CarsOfInterestAhead, lane1CarsOfInterestAhead, lane2CarsOfInterestAhead, TARGET_SPEED, car_speed, lane, car_s);

						//see if the next over lane to merge into is open or will be in the future
						if (lane != bestLane && car_speed > 15)
						{
							oldLane = lane;
							int testLane = 99;
							bool weCanMove = false;
							//We should request a move to the next lane over
							if ((bestLane == 0 && lane == 2) || (bestLane == 2 && lane == 0) || bestLane == 1) //best lane is two over, move to middle first
							{
								testLane = 1;
								weCanMove = CanIMerge(lane1CarsOfInterestAhead, lane1CarsOfInterestBehind, car_s); //might only need testLane here
							}
							else if (bestLane == 0 && lane == 1)
							{
								testLane = 0;
								weCanMove = CanIMerge(lane0CarsOfInterestAhead, lane0CarsOfInterestBehind, car_s); //might only need testLane here
							}
							else if (bestLane == 2 && lane == 1)
							{
								testLane = 2;
								weCanMove = CanIMerge(lane2CarsOfInterestAhead, lane2CarsOfInterestBehind, car_s); //might only need testLane here
							}

							if (weCanMove)
							{
								lane = testLane;
								laneChanging = true;
							}
							//cout << "Best Lane: " << bestLane << "	Can we pass: " << weCanMove << endl;
						}
					}

					

					//check if we are in the middle of a lane change
					if (laneChanging)
					{
						bool weCanMoveStill = true;

						if (lane == 1) //make sure car didn't merge infront of us
						{
							weCanMoveStill = CanIMerge(lane1CarsOfInterestAhead, lane1CarsOfInterestBehind, car_s); //might only need testLane here
						}
						else if (lane == 0)
						{
							weCanMoveStill = CanIMerge(lane0CarsOfInterestAhead, lane0CarsOfInterestBehind, car_s); //might only need testLane here
						}
						else if (lane == 2)
						{
							weCanMoveStill = CanIMerge(lane2CarsOfInterestAhead, lane2CarsOfInterestBehind, car_s); //might only need testLane here
						}

						if (!weCanMoveStill) //if a car moved infront of us, we return to the lane that we started in
						{
							lane = oldLane;
						}

						if (lane == 0 && car_d > 1.5 && car_d < 2.5) //my lane change is complete, or almost complete
						{
							laneChanging = false;
						}
						else if (lane == 1 && car_d > 5.5 && car_d < 6.5)
						{
							laneChanging = false;
						}
						else if (lane == 2 && car_d > 9.5 && car_d < 10.5)
						{
							laneChanging = false;
						}
					}

					/*
					*All of the following spline stuff was given in walk-through
					*/
					//Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
					//Later interpolate these with a spline and fill it in wth more points that control

					vector<double> ptsx;
					vector<double> ptsy;

					//reference x,y, and yaw states
					//either reference the starting point or the previous path end points
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);


					//if previous size is almost empty, use the car as starting ref
					if (prev_size < 2)
					{
						//use two points that make the path tangent to the car
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);

						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);

						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					}
					//use the previous path's endpoint as starting reference
					else
					{
						//redefine reference state as prevous path's end point
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];

						double ref_x_prev = previous_path_x[prev_size - 2];
						double ref_y_prev = previous_path_y[prev_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

						//use two points that make the path tangent to the previous path's endpoint
						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);

						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);
					}



					//In Frenet add evenly 30m spaced points head of the starting reference
					vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s + 50, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s + 75, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp3 = getXY(car_s + 100, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp4 = getXY(car_s + 125, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

					ptsx.push_back(next_wp0[0]); //just added a few way points
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);
					ptsx.push_back(next_wp3[0]);
					ptsx.push_back(next_wp4[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);
					ptsy.push_back(next_wp3[1]);
					ptsy.push_back(next_wp4[1]);

					//shift car to 0 axis
					for (int i = 0; i < ptsx.size(); i++)
					{
						//shift car reference angle to zero
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;

						ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
						ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
					}

					//create a spline along these new points
					tk::spline s;

					//set (x,y) points to the spline
					s.set_points(ptsx, ptsy);

					//define the actual (x,y) points we will use for the planner
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					//start with all of the previous path points from last time
					for (int i = 0; i < previous_path_x.size(); i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
						//cout << "prev x: " << previous_path_x[i] << endl;
					}

					//calculate how to break up spline points so that we travel at our desired reference velocity
					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt((target_x) * (target_x)+(target_y) * (target_y));

					double x_add_on = 0;

					//fill up the rest of our path planner
					for (int i = 0; i <= 50 - previous_path_x.size(); i++) //50 points
					{
						double N = (target_dist / (0.02 * ref_vel / 2.24)); //mph needs to be in m/s
						double x_point = x_add_on + (target_x) / N;
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						//rotate back to normal after rotating earlier
						x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
						y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);

						//cout << "next x: " << next_x_vals[i] << endl;
					}


					//SUPPLIED
					json msgJson;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}


			}
			else {
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
		}
		else {
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
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}

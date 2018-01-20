#ifndef COST_H
#define COST_H

#include <vector>

using namespace std;

//This function takes a lane and returns the speed of the slowest car that it finds
double LaneSpeed(const vector<vector<double> > lane, double desiredSpeed); 

//This function takes the three lane's cars of interest and figures out which lane is the best to be in going forward																		   
int BestLane(const vector<vector<double> > leftLane, const vector<vector<double> > middleLane, const vector<vector<double> > rightLane, double desiredSpeed, double currentSpeed, int currentLane, double myCars);

//This function finds the car in each lane that is closest to our car's 's' value
double CongestionCost(const vector<vector<double> > lane, double myCars);

//This function takes the three costs and decides the land that has the lowest cost
int BestLaneDecider(double leftLaneCost, double middleLaneCost, double rightLaneCost, int currentLane);

//This function determines if the best lane choice is okay to merge into
bool CanIMerge(const vector<vector<double> > aheadCars, const vector<vector<double> > behindCars, double myCars);

//double PredictCars()

#endif

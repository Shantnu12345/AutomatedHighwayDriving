#ifndef LANECHANGERFSM_H
#define LANECHANGERFSM_H

#include <iostream>
#include "helpers.h"
#include "spline.h" 

int logs=0, maxlogs=70;


static const double HIGHWAY_SPEED_MPS = 22;
static const int    TRAJ_SIZE         = 50;

enum State {
  IDLE, //vehicle is just standing and not moving
  KEEP_LANE, 
  PREP_LEFT_LANE_CHANGE, 
  LEFT_LANE_CHANGE, 
  PREP_RIGHT_LANE_CHANGE, 
  RIGHT_LANE_CHANGE
};

enum Lane{ LEFT, RIGHT};

class LaneChangerFSM
{
public:
  LaneChangerFSM():_state(IDLE), _curLane(1), _refvel(HIGHWAY_SPEED_MPS) {}
  Path findNextPath(Pose const& pose, MapData const& mapData, Path const& prev, vector<Pose> const& cars);

private:
  void setState(State s) {_state=s;}
  Lane findBestLane() { return LEFT; };
  Path findKeepLanePath(Pose const& pose, MapData const& mapData, Path const& prev);
  bool closeToVehicleInFront(Pose const& pose, vector<Pose> const& cars);
  
  State _state;
  int _curLane;
  double _refvel;//in mps
};

bool LaneChangerFSM::closeToVehicleInFront(Pose const& pose, vector<Pose> const& cars)
{ 
  // for(auto const& car:cars)
  // {
  //   double spd=sqrt(car.vx*car.vx + car.vy*car.vy);
  //   double stotal = car.s + 
  // }
  
  return false; 
    
}

Path LaneChangerFSM::findNextPath(Pose const& pose, MapData const& mapData, Path const& prev, vector<Pose> const& cars)
{
  //cout<<endl<<"FindNextPath";
  bool pathGenerated;
  while(!pathGenerated)
  {
    switch(_state)
    {
      case IDLE:
        setState(KEEP_LANE);
        break;
      case KEEP_LANE:
        if(closeToVehicleInFront(pose, cars))
        {
          Lane bestLane = findBestLane();
          if(bestLane==LEFT) 
          {
            setState(PREP_LEFT_LANE_CHANGE);
            continue;
          }
          else
          {
            setState(PREP_RIGHT_LANE_CHANGE);
            continue;
          }
        } 
        Path path = findKeepLanePath(pose, mapData, prev);
        pathGenerated=true;
        return path;

        break;
    }
  }
  return Path();
}

Path LaneChangerFSM::findKeepLanePath(Pose const& pose, MapData const& mapData, Path const& prev)
{  
  if(logs<maxlogs) cout<<"x:"<<pose.x<<" y:"<<pose.y<<" yaw:"<<pose.yaw<<" spd:"<<pose.spd <<" s:"<<pose.s <<" d:"<<pose.d<<endl;
  Path temp, path;
  temp.xpts.reserve(TRAJ_SIZE);
  temp.ypts.reserve(TRAJ_SIZE);

  path.xpts.reserve(TRAJ_SIZE);
  path.ypts.reserve(TRAJ_SIZE);

  /*
  //Basic way
  double disBetweenWayPts = 0.447;
  for(int i=1; i<= 50; i++)
  {
    vector<double> xy = getXY(pose.s+i*disBetweenWayPts, 6, mapData.waypoints_s, mapData.waypoints_x, mapData.waypoints_y);
    path.xpts.emplace_back(xy[0]);
    path.ypts.emplace_back(xy[1]);
  }
  */
 
  double refx, refy, refyaw;
  int prevSize = prev.xpts.size();
  
  if(prevSize < 2)
  {
    refx=pose.x; 
    refy=pose.y; 
    refyaw=pose.yaw;
    temp.xpts.emplace_back(pose.x - cos(pose.yaw));
    temp.ypts.emplace_back(pose.y - sin(pose.yaw));

    temp.xpts.emplace_back(pose.x);
    temp.ypts.emplace_back(pose.y);
  }
  else
  {
    refx=prev.xpts[prevSize-1]; 
    refy=prev.ypts[prevSize-1]; 
    double secondLastx=prev.xpts[prevSize-2], secondLasty=prev.ypts[prevSize-2];
    refyaw=atan2(refy-secondLasty, refx-secondLastx);
    
    temp.xpts.emplace_back(secondLastx);
    temp.ypts.emplace_back(secondLasty);

    temp.xpts.emplace_back(refx);
    temp.ypts.emplace_back(refy);
  }

  vector<double> xy1 = getXY(pose.s+30, 2+4*_curLane, mapData.waypoints_s, mapData.waypoints_x, mapData.waypoints_y);
  temp.xpts.emplace_back(xy1[0]); temp.ypts.emplace_back(xy1[1]);
  vector<double> xy2 = getXY(pose.s+60, 2+4*_curLane, mapData.waypoints_s, mapData.waypoints_x, mapData.waypoints_y);
  temp.xpts.emplace_back(xy2[0]); temp.ypts.emplace_back(xy2[1]);
  vector<double> xy3 = getXY(pose.s+90, 2+4*_curLane, mapData.waypoints_s, mapData.waypoints_x, mapData.waypoints_y);
  temp.xpts.emplace_back(xy3[0]); temp.ypts.emplace_back(xy3[1]);

  if(logs<maxlogs) cout<<"Refx:"<<refx<<" Refy:"<<refy<<" Refyaw:"<<refyaw<<" prev:"<<prevSize<<endl;
  for(int i=0; i<temp.xpts.size(); i++)
  {
    double shiftx = temp.xpts[i]-refx;//refx, refy, refyaw is car's location
    double shifty = temp.ypts[i]-refy;
    temp.xpts[i] =  shiftx*cos(refyaw) + shifty*sin(refyaw);
    temp.ypts[i] = -shiftx*sin(refyaw) + shifty*cos(refyaw);
    if(logs<maxlogs) cout<<temp.xpts[i]<<" "<< temp.ypts[i]<< endl;
  }

  //Put all the remianing points from the previous trajectory sent to the simulator
  path.xpts.insert(path.xpts.end(), prev.xpts.begin(), prev.xpts.end());
  path.ypts.insert(path.ypts.end(), prev.ypts.begin(), prev.ypts.end());
  
  tk::spline s;
  s.set_points(temp.xpts, temp.ypts);
  double targetx=30, targety=s(targetx), targetDis=sqrt(targetx*targetx + targety*targety), xAddOn=0;
  for(int i=1; i<=TRAJ_SIZE-prevSize; i++)
  {
    int N = targetDis/(0.02*_refvel);
    double nextx = xAddOn+targetx/N; 
    double nexty = s(nextx); //This is in veh frame

    xAddOn = nextx;

    //Convert back to global
    double nextGlobalx = refx + nextx*cos(refyaw) - nexty*sin(refyaw);
    double nextGlobaly = refy + nextx*sin(refyaw) + nexty*cos(refyaw);
    
    path.xpts.emplace_back(nextGlobalx);
    path.ypts.emplace_back(nextGlobaly);
  }

  logs++;
  return path;
}

#endif  // HELPERS_H
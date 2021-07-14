#ifndef LANECHANGERFSM_H
#define LANECHANGERFSM_H

#include <iostream>
#include "helpers.h"
#include "spline.h" 

int logs=0, maxlogs=70;


//Todo:
// - Lane shift is still aggressive
// - Lane change logic:
//      - Seeing cars in the other lane, using similar logic to what we did for current lane, 
//        and then seeing it the car is within certain gap (50m behind you, 30m infront of you)
//      - Cost function for lane change. Try looking at the lanes for next 5 secs and see which one is the best 
//        to be in. To look at future the prediction lesson would be helpful. There maive bayes is used to predict 
//        future trajectories 

static const double HIGHWAY_SPEED_MPS = 22;
static const int    TRAJ_SIZE         = 50;
static const double acc               = mph_to_mps(0.224);

enum State {
  IDLE, //vehicle is just standing and not moving
  KEEP_LANE, 
  PREP_LEFT_LANE_CHANGE, 
  LEFT_LANE_CHANGE, 
  PREP_RIGHT_LANE_CHANGE, 
  RIGHT_LANE_CHANGE
};

enum Lane{ LEFT, RIGHT, CURRENT};

class LaneChangerFSM
{
public:
  LaneChangerFSM():_state(IDLE), _curLane(1), _refvel(0) {}
  Path findNextPath(Pose const& pose, MapData const& mapData, Path const& prev, vector<Pose> const& cars);

private:
  void setState(State s) {_state=s;}
  LaneChangeResults isLaneChangePossible(int lane, Pose const& pose, MapData const& mapData, Path const& prev, vector<Pose> const& cars);  
  Lane findBestLane(Pose const& pose, MapData const& mapData, Path const& prev, vector<Pose> const& cars);
  Path findKeepLanePath(Pose const& pose, MapData const& mapData, Path const& prev);
  int closeToVehicleInFront(Pose const& pose, MapData const& mapData, Path const& prev, vector<Pose> const& cars);
  inline double predictEgoS(Pose const& pose, Path const& prev);
  State _state;
  int _curLane;
  double _refvel;//in mps
};

double LaneChangerFSM::predictEgoS(Pose const& pose, Path const& prev)
{
  int prevSize=prev.xpts.size();
  if(prevSize<2)
    return pose.s;
  
  return prev.lastS;  
}

int LaneChangerFSM::closeToVehicleInFront(Pose const& pose, MapData const& mapData, Path const& prev, vector<Pose> const& cars)
{ 
  //Logic using predictions. Note, no need for prediction of ego veh because we know it from
  //prev
  double egoPredS = predictEgoS(pose, prev);
  int prevSize=prev.xpts.size();
  for(auto const& car:cars)
  {
    //Width of evey lane is 4. Note, our d is 4*_curLane + 2
    //Dont worry if this vehicle is not in our lane
    if(car.d < 4*_curLane || car.d > 4*(_curLane+1)) continue;//might wanna use car's predicted d?
    double carPredS = car.s + prevSize*0.02*car.spd;
    if(carPredS>egoPredS && carPredS-egoPredS<30)
      return car.id;
  }

  /*
  //Naive logic using current position
  for(auto const& car:cars)
  {
    if(car.d < 4*_curLane || car.d > 4*(_curLane+1)) continue;    
    double stoppingDistance = 30;// + abs(pose.spd*pose.spd-car.spd*car.spd)/2*acc;
    if(car.s > pose.s && car.s-pose.s < stoppingDistance)// && car.spd < pose.spd)    
      return car.id;
  }
  */
  
  return -1; 
    
}

LaneChangeResults LaneChangerFSM::isLaneChangePossible(int lane, Pose const& pose, MapData const& mapData, Path const& prev, vector<Pose> const& cars)
{ 
  if(lane<0 || lane>2)
    return LaneChangeResults{false, -1, 0};
  double egoPredS = predictEgoS(pose, prev);
  int prevSize=prev.xpts.size();
  int leadId=-1;
  double minLeadDis=10000;
  for(auto const& car:cars)
  {
    if(car.d < 4*lane || car.d > 4*(lane+1)) continue;//might wanna use car's predicted d?
    double carPredS = car.s + prevSize*0.02*car.spd;
    double dis=abs(carPredS-egoPredS);
    if(    (carPredS>egoPredS && dis<20) //Cannot change lane
        || (carPredS<egoPredS && dis<20)
      )
    {
      return LaneChangeResults{false, -1, 0};
    }

    if((carPredS>egoPredS && dis<minLeadDis))
    {
      leadId=car.id;
      minLeadDis=carPredS-egoPredS;
    }
  }

  return LaneChangeResults{true, leadId, minLeadDis}; 
    
}

Lane LaneChangerFSM::findBestLane(Pose const& pose, MapData const& mapData, Path const& prev, vector<Pose> const& cars) 
{ 
  auto leftResults  = isLaneChangePossible(_curLane-1, pose, mapData, prev, cars);
  auto rightResults = isLaneChangePossible(_curLane+1, pose, mapData, prev, cars);
  cout<<"LeftPossible:"<<leftResults.possible<<" rightPossible:"<<rightResults.possible
      <<" ldis:"<<leftResults.predDis2Lead<<" rdis:"<<rightResults.predDis2Lead<<endl;
  
  //Cost = s travelled over next 5sec
  double curCost = pose.spd*5;

  double leftLeadSpd;
  if(leftResults.possible)
    leftLeadSpd = leftResults.leadId==-1  ? HIGHWAY_SPEED_MPS:cars[leftResults.leadId].spd;
  else
    leftLeadSpd=0;

  double rightLeadSpd; 
  if(rightResults.possible)
    rightLeadSpd = rightResults.leadId==-1 ? HIGHWAY_SPEED_MPS:cars[rightResults.leadId].spd;
  else
    rightLeadSpd=0;
  
  double leftCost  = leftLeadSpd*5;
  double rightCost = rightLeadSpd*5;
  cout<<"LC:"<<leftCost<<" CC:"<<curCost<<" RC:"<<rightCost<<endl;

  if(leftCost>curCost)
  {
    if(leftCost>rightCost) return LEFT;
    else                   return RIGHT;
  }
  else
  {
    if(curCost>rightCost)  return CURRENT;
    else                   return RIGHT;
  }
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
        int leadId = closeToVehicleInFront(pose, mapData, prev, cars);
        
        if(leadId!=-1)
        {
          //auto const& lead=cars[leadId];
          //double leadspd=sqrt(lead.vx*lead.vx + lead.vy*lead.vy);
          //cout<<"LeadId"<<leadId<< " Leadspd:"<<leadspd<< " refvel:"<<_refvel<<endl;
          //if(leadspd<_refvel) 
          _refvel-=acc;
          //else
          //  _refvel=leadspd;
          //if(lead.s-pose.s<25)
          //  _refvel+=acc;


          Lane bestLane = findBestLane(pose, mapData, prev, cars);
          if(bestLane==LEFT) 
          {
            cout<<"LEFT"<<endl;
            _curLane = _curLane==0 ? _curLane : _curLane-1;
            //setState(PREP_LEFT_LANE_CHANGE);
            //continue;
          }
          else if(bestLane==RIGHT)
          {
            cout<<"RIGHT"<<endl;
            _curLane = _curLane==2 ? _curLane : _curLane+1;
            //setState(PREP_RIGHT_LANE_CHANGE);
            //continue;
          }
          else 
          {
            cout<<"CURRENT"<<endl;
            //do Nothing
          }
        } 
        else if(_refvel < HIGHWAY_SPEED_MPS)
        {
          _refvel+=acc;
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

  float longDis=50.0f;
  vector<double> xy1 = getXY(pose.s+longDis*1, 2+4*_curLane, mapData.waypoints_s, mapData.waypoints_x, mapData.waypoints_y);
  temp.xpts.emplace_back(xy1[0]); temp.ypts.emplace_back(xy1[1]);
  vector<double> xy2 = getXY(pose.s+longDis*2, 2+4*_curLane, mapData.waypoints_s, mapData.waypoints_x, mapData.waypoints_y);
  temp.xpts.emplace_back(xy2[0]); temp.ypts.emplace_back(xy2[1]);
  vector<double> xy3 = getXY(pose.s+longDis*3, 2+4*_curLane, mapData.waypoints_s, mapData.waypoints_x, mapData.waypoints_y);
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
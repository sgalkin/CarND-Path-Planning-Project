#include "catch.hpp"
#include <limits>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <random>
#include <cassert>

namespace {
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x1 - x2)*(x1-x2) + (y1-y2)*(y1-y2));
}
  
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  double closestLen = 100000; //large number
  size_t closestWaypoint = 0;

  for(size_t i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
    size_t closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

 	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
    angle = std::min(2*M_PI - angle, angle); // ???

    if(angle > M_PI/4)
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
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-M_PI/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}
}

TEST_CASE("Consistancy") {
  // Load up map values for waypoint's x,y,s and d normalized normal std::vectors
  std::vector<double> map_x;
  std::vector<double> map_y;
  std::vector<double> map_s;
//  std::vector<double> map_waypoints_dx;
//  std::vector<double> map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
//  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (std::getline(in_map_, line)) {
    std::istringstream iss(line);
  	double x;
  	double y;
  	float s;
//  	float d_x;
//  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
//  	iss >> d_x;
//  	iss >> d_y;
  	map_x.push_back(x);
  	map_y.push_back(y);
  	map_s.push_back(s);
//  	map_waypoints_dx.push_back(d_x);
//  	map_waypoints_dy.push_back(d_y);
  }

  std::mt19937 gen(42);
  std::uniform_int_distribution<> pnt(0, map_x.size() - 1);
  std::uniform_real_distribution<> dp(-15, 15);
  std::normal_distribution<> da(0, M_PI/4);
  for(size_t i = 0; i < 100000; ++i) {
    auto idx = pnt(gen);
    auto n = (idx + 1) % map_x.size();
    auto rx = map_x[idx] + dp(gen);
    auto ry = map_y[idx] + dp(gen);
    auto f = getFrenet(rx, ry,
                       atan2((map_y[n]-map_y[idx]),(map_x[n]-map_x[idx])) + da(gen),
                       map_x, map_y);
    auto xy = getXY(f[0], f[1], map_s, map_x, map_y);
    REQUIRE(rx == Approx(xy[0]).margin(71));
    REQUIRE(ry == Approx(xy[1]).margin(71));
  }
}


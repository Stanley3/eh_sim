#include <math.h>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <vector>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <opencv2/core.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace boost;
using namespace cv;

struct odometry_raw {
  double pos_x_raw;
  double pos_y_raw;
  double head_direction_raw;
};

void get_all_files(const char *, vector<string> &);
void get_all_infos(const vector<string> &, vector<odometry_raw> &);
void interpolate(int, double, int, vector<odometry_raw>, Mat &, Mat &);
void image_to_gray(string, Mat &);
void get_depth_image(string, Mat &);
void meshgrid(const vector<double>&, const vector<double>&, Mat &, Mat &);
void ind2sub(const int, const int, const int, int *, int *);
void shiftCol(Mat &, Mat, int);
void shiftRow(Mat &, Mat, int);

template<typename T>
inline void get_setting_from_ptree(T & var, boost::property_tree::ptree & settings, std::string name, T default_value)
{
  try
  {
    var = settings.get<T>(name);
  }
  catch(boost::property_tree::ptree_bad_path pbp)
  {
    var = default_value;
    std::cout << "SETTINGS(warning): " << name << " not found so default (" << default_value << ") used." << std::endl;
  }
}

inline bool get_setting_child(boost::property_tree::ptree & child, boost::property_tree::ptree & settings, std::string name, bool pause_on_error = true)
{
  try
  {
    child = settings.get_child(name);
  }
  catch(boost::property_tree::ptree_bad_path pbp)
  {
    std::cout << "SETTINGS(error): " << name << " child not found." << std::endl;
    //		if (pause_on_error)
    //			std::cin.get();
    return false;
  }
  return true;
}

// % Clip the input angle to between 0 and 2pi radians
inline double clip_rad_360(double angle)
{
  while (angle < 0)
    angle += 2.0 * M_PI;

  while (angle >= 2.0 * M_PI)
    angle -= 2.0 * M_PI;

  return angle;
}

// % Clip the input angle to between -pi and pi radians
inline double clip_rad_180(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;

  while (angle <= -M_PI)
    angle += 2.0 * M_PI;

  return angle;
}

inline double Get_Signed_Delta_Rad(double angle1, double angle2)
{
  double dir = clip_rad_180(angle2 - angle1);

  double delta_angle = clip_rad_360(angle1) - clip_rad_360(angle2);
  delta_angle = abs(delta_angle);

  if (delta_angle < 2.0 * M_PI - delta_angle)
  {
    if (dir > 0)
      return delta_angle;
    else
      return -delta_angle;
  }
  else
  {
    if (dir > 0)
      return 2.0 * M_PI - delta_angle;
    else
      return -(2.0 * M_PI - delta_angle);
  }
}

inline double Get_Min_Delta(double d1, double d2, double max) {
 double t1 = fabs(d1 - d2);
 double t2 = max - fabs(d1 - d2);
 return t1 < t2 ? t1: t2;
}

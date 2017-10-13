#ifndef _VISUAL_TEMPLATE_MATCH_H_
#define _VISUAL_TEMPLATE_MATCH_H_
#pragma warning( disable: 4275 ) // problem between std::vector and log4cxx
#pragma warning( disable: 4251 ) // problem between std::vector and log4cxx
#include <cstring>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/core.hpp>
#include <numeric>

#define _USE_MATH_DEFINES
#include "math.h"
// todo: replace this with iostream
#include <stdio.h>

#include <boost/property_tree/ini_parser.hpp>
using  boost::property_tree::ptree;
using namespace cv;
using namespace std;

#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
//#define SUPER_EXPERIENCES
namespace eh_sim
{


  typedef struct td_visual_template
  {
    unsigned int id;
    double gc_x, gc_y, hdc;
    std::vector<double> rgb_column_sum;
    std::vector<double> depth_column_sum;
    double decay;
    int first;
    std::vector<int> exps;
    int numexp;
    template <typename Archive>
      void serialize(Archive& ar, const unsigned int version)
      {
        ar & id;
        ar & gc_x & gc_y & hdc;
        ar & rgb_column_sum;
        ar & depth_column_sum;
        ar & decay;
        ar & first;
        ar & exps;
        ar & numexp;
      }

  } Visual_Template; 

  class Visual_Template_Match
  {
    public:
      Visual_Template_Match(ptree settings);

      ~Visual_Template_Match();
      void init_template_one(struct td_visual_template&);
      void set_depth_sum(int x_max, int y_max);

      // compares raw values not view templates
      double compare_views(double *vt1, double *vt2, int size);

      // create and add a visual template to the collection
      int create_template(double gc_x, double gc_y, double hdc);

      // compare a visual template to all the stored templates, allowing for 
      // slen pixel shifts in each direction
      // returns the matching template and the MSE
      void compare(double &vt_err, unsigned int &vt_match_id);

      double compare_two(int vt_id1, int vt_id2);

      void do_log(int frame, double *column_sum, int IMAGE_WIDTH);

      void set_x_range(int min, int max) { IMAGE_VT_X_RANGE_MIN = min; IMAGE_VT_X_RANGE_MAX = max;}
      void set_y_range(int min, int max) { IMAGE_VT_Y_RANGE_MIN = min; IMAGE_VT_Y_RANGE_MAX = max;}
      void set_rgb_weight(double weight) { VT_RGB_WEIGHT = weight; }
      void set_depth_weight(double weight) { VT_DEPTH_WEIGHT = weight; }
      void set_match_threshold(double thre) { VT_MATCH_THRESHOLD = thre; }
      void set_prev_vt_id(int vt_id) { prev_vt_id = vt_id; }

      int get_x_range_min() { return IMAGE_VT_X_RANGE_MIN; }
      int get_x_range_max() { return IMAGE_VT_X_RANGE_MAX; }
      int get_y_range_min() { return IMAGE_VT_Y_RANGE_MIN; }
      int get_y_range_max() { return IMAGE_VT_Y_RANGE_MAX; }
      double get_rgb_weight() { return VT_RGB_WEIGHT; }
      double get_depth_weight() { return VT_DEPTH_WEIGHT; }
      double get_math_threshold() { return VT_MATCH_THRESHOLD; }
      int get_image_width() { return IMAGE_WIDTH; }
      int get_image_height() { return IMAGE_HEIGHT; }
      int get_interpl_num() { return INTERPL_NUM; }
      int get_vt_matched() { return vt_matched; }

      int get_number_of() { return templates.size(); }

      int get_vt_shift_match() const { return VT_SHIFT_MATCH; }
      //int get_vt_step_match() const { return VT_STEP_MATCH; }

      double get_vt_active_delay() const { return VT_ACTIVE_DECAY; }
      double get_vt_match_threshold() const { return VT_MATCH_THRESHOLD; }

      double get_decay(int id) { return templates[id].decay; }
      double get_gc_x(int id) { return templates[id].gc_x; }
      double get_gc_y(int id) { return templates[id].gc_y; }
      struct td_visual_template get_prev_template();

      int get_current_vt() { return current_vt; }
      void set_current_vt(int id) {
        if (current_vt != id)
        {
          prev_vt_id = current_vt;
          templates[id].decay = VT_ACTIVE_DECAY;
        }
        else
          templates[id].decay += VT_ACTIVE_DECAY;

        current_vt = id;
      }

      unsigned int get_current_exp_size() { return templates[current_vt].exps.size(); }
      unsigned int get_current_exp_link(int id) { return templates[current_vt].exps[id]; }
      void add_exp_to_current(unsigned int id) { templates[current_vt].exps.push_back(id); }

      double get_current_gc_x() { return templates[current_vt].gc_x; }
      double get_current_gc_y() { return templates[current_vt].gc_y; }
      double get_current_hdc() { return templates[current_vt].hdc; }
      double get_depth_sum() { return DEPTH_SUM; }

      //void set_view_rgb(const unsigned char * view_rgb) { this->view_rgb = view_rgb; }
      //	const unsigned char * get_view_rgb() {	return view_rgb; }
      void convert_view_to_view_template();
      int image_compare_with_template(Mat, Mat, double, double, double);

      template <typename Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
          ar & VT_SHIFT_MATCH;
          ar & VT_ACTIVE_DECAY;
          ar & VT_MATCH_THRESHOLD;
          ar & IMAGE_WIDTH;
          ar & IMAGE_HEIGHT;
          ar & IMAGE_VT_X_RANGE_MIN;
          ar & IMAGE_VT_X_RANGE_MAX;
          ar & IMAGE_VT_Y_RANGE_MIN;
          ar & IMAGE_VT_Y_RANGE_MAX;

          ar & templates;
          ar & current_view;
          ar & current_vt;
          ar & prev_vt_id;

          //	ar & log_vt;
          //	ar & log_views;
        }

    private:
      friend class boost::serialization::access;

      Visual_Template_Match() { ; }
      void clip_view_x_y(int &x, int &y);
      void rs_compare_segments(vector<double>, vector<double>, vector<double>, vector<double>, int, int, vector<double>);

      int VT_SHIFT_MATCH;//20
      double VT_ACTIVE_DECAY;//1.0
      double VT_GLOBAL_DECAY;//0.1
      double VT_MATCH_THRESHOLD;//
      double VT_RGB_WEIGHT;
      double VT_DEPTH_WEIGHT;
      double DEPTH_SUM; //初始化
      double VT_REPEAT;//10
      int IMAGE_WIDTH; //x = 320
      int IMAGE_HEIGHT; //y = 240
      int GC_NEURONSHEET_X;//40
      int IMAGE_VT_X_RANGE_MIN; //1
      int IMAGE_VT_X_RANGE_MAX;// IMAGE_X_SIZE
      int IMAGE_VT_Y_RANGE_MIN;
      int IMAGE_VT_Y_RANGE_MAX;
      int INTERPL_NUM;

      std::vector<Visual_Template> templates;
      std::vector<double> current_view;

      int current_vt;
      //double vt_error;
      int prev_vt_id;//1
      int numvts;//1
      std::vector<int> vt_history;
      int vt_repeat;//0
      int vt_matched;//0
      int vt_conflict_num; //0

      //const unsigned char *view_rgb;



      // unsupported for the moment until fix for serialisation
      // although prob don't need either
      // log vt stuff can go into the ratslam log and log views can be its own program
      //	std::ofstream log_vt;
      //	std::ofstream log_views;
  };
}

#endif // _VISUAL_TEMPLATE_MATCH_H_

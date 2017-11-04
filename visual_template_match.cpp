#include "visual_template_match.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <iomanip>
#include <boost/foreach.hpp>
#include "utils/utils.h"

using namespace std;
namespace eh_sim
{

  Visual_Template_Match::Visual_Template_Match(ptree settings)
  {
    get_setting_from_ptree(VT_SHIFT_MATCH, settings, "vt_shift_match", 25);
    get_setting_from_ptree(VT_ACTIVE_DECAY, settings, "vt_active_decay", 1.0);
    get_setting_from_ptree(VT_GLOBAL_DECAY, settings, "vt_global_decay", 0.1);
    get_setting_from_ptree(IMAGE_WIDTH, settings, "image_width", 320);
    get_setting_from_ptree(IMAGE_HEIGHT, settings, "image_height", 240);
    get_setting_from_ptree(GC_NEURONSHEET_X, settings, "gc_neuronsheet_x", 40);
    get_setting_from_ptree(INTERPL_NUM, settings, "interpl_num", 10);

    templates.reserve(10000);

    current_vt = 0;
    prev_vt_id = 0;
    numvts     = 1;
    vt_repeat  = 0;
    vt_matched = 0;
    vt_history.push_back(1);

    //struct eh_sim::td_visual_template template1;
    //init_template_one(template1, *this);
    //templates.push_back(template1);
  }

  Visual_Template_Match::~Visual_Template_Match()
  {

    cout << "Visual_Template_Match 析构函数被调用\n";
  }


  void Visual_Template_Match::set_depth_sum(int x_max, int y_max) {
    DEPTH_SUM = 255 * (x_max + 1) * (y_max + 1);
  }

  struct td_visual_template Visual_Template_Match::get_prev_template() {
    return templates[prev_vt_id];
  }

  int Visual_Template_Match::image_compare_with_template(Mat gray_image, Mat depth_image, double x, double y, double th) {
    int vt_id;

    cout << "begin copyTo \n";
    cout << "gray_image.size() = " << gray_image.size() << "\n";
    cout << "depth_image.size() = " << depth_image.size() << "\n";
    printf("(IMAGE_VT_Y_RANGE_MIN, IMAGE_VT_Y_RANGE_MAX) = (%d, %d)\n", IMAGE_VT_Y_RANGE_MIN, IMAGE_VT_Y_RANGE_MAX);
    printf("(IMAGE_VT_X_RANGE_MIN, IMAGE_VT_X_RANGE_MAX) = (%d, %d)\n", IMAGE_VT_X_RANGE_MIN, IMAGE_VT_X_RANGE_MAX);

    Mat sub_gray_image;
    //gray_image.rowRange(IMAGE_VT_Y_RANGE_MIN, IMAGE_VT_Y_RANGE_MAX).
    //  colRange(IMAGE_VT_X_RANGE_MIN, IMAGE_VT_X_RANGE_MAX).
    //  copyTo(sub_gray_image);
    sub_gray_image = gray_image(Range(IMAGE_VT_Y_RANGE_MIN, IMAGE_VT_Y_RANGE_MAX),
        Range(IMAGE_VT_X_RANGE_MIN, IMAGE_VT_X_RANGE_MAX));
      //copyTo(sub_gray_image);

    Mat sub_depth_image;
    //depth_image.rowRange(IMAGE_VT_Y_RANGE_MIN, IMAGE_VT_Y_RANGE_MAX).
    //  colRange(IMAGE_VT_X_RANGE_MIN, IMAGE_VT_X_RANGE_MAX).
    //  copyTo(sub_depth_image);
    sub_depth_image = depth_image(Range(IMAGE_VT_Y_RANGE_MIN, IMAGE_VT_Y_RANGE_MAX),
        Range(IMAGE_VT_X_RANGE_MIN, IMAGE_VT_X_RANGE_MAX));
      //copyTo(sub_depth_image);
    cout << "after copyTo \n";

    vector<double> gray_image_x_sums, depth_image_x_sums;
    // TODO check
    cout << "sub_gray_image.size() = \n" << sub_gray_image.size() << "\n";
    cout << "sub_depth_image.size() = \n" << sub_depth_image.size() << "\n";
    gray_image_x_sums.reserve(sub_gray_image.cols);
    depth_image_x_sums.reserve(sub_depth_image.cols);
    cout << "begin reduce \n";
    reduce(sub_gray_image, gray_image_x_sums, 0, CV_REDUCE_SUM);
    reduce(sub_depth_image, depth_image_x_sums, 0, CV_REDUCE_SUM);
    cout << "after reduce \n";
    double gray_image_sums  = cv::sum(gray_image_x_sums)[0];

    for(int i=0; i<gray_image_x_sums.size(); ++i)
        gray_image_x_sums[i] /= gray_image_sums;
    for(int i=0; i<depth_image_x_sums.size(); ++i)
        depth_image_x_sums[i] *= (4096.0/255.0/DEPTH_SUM);

    cout << "two loop done!" << "\n";

    Mat min_offset = Mat::ones(numvts, 1, CV_64F);
    Mat min_diff   = Mat::ones(numvts, 1, CV_64F);

    vector<double> result;
    result.reserve(2);
    for (int i=0; i<numvts; ++i) {
      templates[i].decay -= VT_GLOBAL_DECAY;
      if (templates[i].decay < 0)
        templates[i].decay = 0;

      cout << "before rs_compare_segments i = " << i << "\n";
      result.clear();
      rs_compare_segments(gray_image_x_sums, depth_image_x_sums, templates[i].rgb_column_sum, templates[i].depth_column_sum, VT_SHIFT_MATCH, gray_image_x_sums.size(), result);
      cout << "after rs_compare_segments i = " << i << "\n";

      min_offset.at<double>(i, 0) = result[0];
      min_diff.at<double>(i, 0)   = result[1];
    }

    double diff, max_diff;
    int diff_id, max_diff_id;
    minMaxIdx(min_diff, &diff, &max_diff, &diff_id, &max_diff_id);

    if (diff * gray_image_x_sums.size() > VT_MATCH_THRESHOLD || diff_id == numvts-1) {
      numvts += 1;
      vt_repeat = 0;
      vt_matched = 0;
      templates.resize(templates.size() + 1);
      struct eh_sim::td_visual_template *new_temp = &(*(templates.end() - 1));
      new_temp->id = numvts;
      new_temp->rgb_column_sum = gray_image_x_sums;
      new_temp->depth_column_sum = depth_image_x_sums;
      new_temp->decay = VT_ACTIVE_DECAY;
      new_temp->gc_x = x;
      new_temp->gc_y = y;
      new_temp->hdc  = th;
      new_temp->first = 1;
      new_temp->numexp = 0;
      //templates.push_back(new_temp);
      vt_id = numvts - 1;
    } else {
      vt_id = diff_id;
      templates[vt_id].decay += VT_ACTIVE_DECAY;
      if (prev_vt_id != vt_id) {
        templates[vt_id].first = 0;
        vt_repeat = 0;
        vt_matched += 1;
      } else {
        vt_repeat += 1;
      }
    }

    vt_history.push_back(vt_id);
    return vt_id;
  }

  typedef vector<double> vd;
  void Visual_Template_Match::rs_compare_segments(vd mono_seg1, vd depth_seg1, vd mono_seg2, vd depth_seg2, int slen, int cwl, vd & result) {
    int mindiff   = pow(10 , 8);
    int minoffset = 0;

    for (int offset=0; offset<=slen; ++offset) {
      double mono_diff_sum = 0;
      double depth_diff_sum = 0;
      for (int i=offset; i<cwl; ++i) {
        mono_diff_sum  += abs(mono_seg1[i]  - mono_seg2[i-offset]);
        depth_diff_sum += abs(depth_seg1[i] - depth_seg2[i-offset]);
      }
      double cdiff = (mono_diff_sum * VT_RGB_WEIGHT +
        depth_diff_sum * VT_DEPTH_WEIGHT) / (cwl - offset);

      if (cdiff < mindiff) {
        mindiff = cdiff;
        minoffset = offset;
      }
    }

    for (int offset=1; offset<=slen; ++offset) {
      double mono_diff_sum = 0;
      double depth_diff_sum = 0;
      for (int i=offset; i<cwl; ++i) {
        mono_diff_sum  += abs(mono_seg1[i]  - mono_seg2[i-offset]);
        depth_diff_sum += abs(depth_seg1[i] - depth_seg2[i-offset]);
      }
      double cdiff = (mono_diff_sum * VT_RGB_WEIGHT +
          depth_diff_sum * VT_DEPTH_WEIGHT) / (cwl - offset);

      if (cdiff < mindiff) {
        mindiff = cdiff;
        minoffset = offset * -1;
      }
    }

    result.push_back(minoffset);
    result.push_back(mindiff);
  }

  void Visual_Template_Match::init_template_one(struct td_visual_template& temp_one) {
   temp_one.id = 1;
   srand(time(NULL));
   temp_one.gc_x = (double)rand() / RAND_MAX * GC_NEURONSHEET_X;
   temp_one.gc_y = (double)rand() / RAND_MAX * GC_NEURONSHEET_X;
   temp_one.hdc = 0;
   temp_one.first = 1;
   temp_one.numexp = 1;
   temp_one.exps.push_back(1);
   for(int i=0; i<IMAGE_VT_X_RANGE_MAX; ++i) {
     temp_one.rgb_column_sum.push_back(0);
     temp_one.depth_column_sum.push_back(0);
   }

   templates.push_back(temp_one);
  }

}

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
    vt_conflict_num = 0;
    vt_history.push_back(1);

    //struct eh_sim::td_visual_template template1;
    //init_template_one(template1, *this);
    //templates.push_back(template1);
  }

  Visual_Template_Match::~Visual_Template_Match()
  {

  }


  void Visual_Template_Match::clip_view_x_y(int &x, int &y)
  {
    //if (x < 0) x = 0;
    //else if (x > TEMPLATE_X_SIZE - 1) x = TEMPLATE_X_SIZE - 1;
    //else x = x;

    //if (y < 0) y = 0;
    //else if (y > TEMPLATE_Y_SIZE - 1) y = TEMPLATE_Y_SIZE - 1;
    //else y = y;
  }

  void Visual_Template_Match::set_depth_sum(int x_max, int y_max) {
    DEPTH_SUM = 255 * (x_max + 1) * (y_max + 1);
  }

  struct td_visual_template Visual_Template_Match::get_prev_template() {
    return templates[prev_vt_id];
  }

  void Visual_Template_Match::convert_view_to_view_template()
  {
    //int column_sum_next = 0;
    //current_view.clear();

    //int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;
    //int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;
    //int x_block_size = sub_range_x / TEMPLATE_X_SIZE;
    //int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;
    //int pos;
    //for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
    //{
    //  for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
    //  {
    //    for (int x = x_block; x < (x_block + x_block_size); x++)
    //    {
    //      for (int y = y_block; y < (y_block + y_block_size); y++)
    //      {
    //        pos = (x + y*IMAGE_WIDTH) * 3;
    //        current_view[column_sum_next] += ((double)(view_rgb[pos]) + (double)(view_rgb[pos + 1]) + (double)(view_rgb[pos + 2]));
    //      }
    //    }
    //    current_view[column_sum_next] /= (255.0*3.0);
    //    current_view[column_sum_next] /= (x_block_size*y_block_size);
    //    column_sum_next++;
    //  }
    //}

    //// now do patch normalisation
    //// make configurable
    //// +- patch size on teh pixel, ie 4 will give a 9x9
    //if (PATCH_NORMALISATION > 0)
    //{
    //  int patch_size = PATCH_NORMALISATION;
    //  int patch_total = (patch_size * 2 + 1)*(patch_size * 2 + 1);
    //  double patch_sum;
    //  double patch_mean;
    //  double patch_std;
    //  int patch_x_clip;
    //  int patch_y_clip;

    //  // first make a copy of the view
    //  std::vector<double> current_view_copy;
    //  current_view_copy.resize(current_view.size());
    //  for (int i = 0; i < current_view.size(); i++)
    //    current_view_copy[i] = current_view[i];

    //  // this code could be significantly optimimised ....
    //  for (int x = 0; x < TEMPLATE_X_SIZE; x++)
    //  {
    //    for (int y = 0; y < TEMPLATE_Y_SIZE; y++)
    //    {
    //      patch_sum = 0;
    //      for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
    //      {
    //        for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
    //        {
    //          patch_x_clip = patch_x;
    //          patch_y_clip = patch_y;
    //          clip_view_x_y(patch_x_clip, patch_y_clip);

    //          patch_sum += current_view_copy[patch_x_clip + patch_y_clip*TEMPLATE_X_SIZE];
    //        }
    //      }
    //      patch_mean = patch_sum / patch_total;

    //      patch_sum = 0;
    //      for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
    //      {
    //        for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
    //        {
    //          patch_x_clip = patch_x;
    //          patch_y_clip = patch_y;
    //          clip_view_x_y(patch_x_clip, patch_y_clip);

    //          patch_sum += ((current_view_copy[patch_x_clip + patch_y_clip*TEMPLATE_X_SIZE] - patch_mean)*(current_view_copy[patch_x_clip + patch_y_clip*TEMPLATE_X_SIZE] - patch_mean));
    //        }
    //      }

    //      patch_std = sqrt(patch_sum / patch_total);

    //      if (patch_std < MIN_PATCH_NORMALISATION_STD)
    //        current_view[x + y*TEMPLATE_X_SIZE] = 0.5;
    //      else
    //        current_view[x + y*TEMPLATE_X_SIZE] = max((double)0, min(1.0, (current_view_copy[x + y*TEMPLATE_X_SIZE] - patch_mean) / patch_std));
    //    }
    //  }
    //}
  }

  // compares raw values not view templates
  double Visual_Template_Match::compare_views(double *vt1, double *vt2, int size)
  {
    double cdiff = 0;

    for (int index = 0; index < size; index++)
    {
      cdiff += abs(vt1[index] - vt2[index]);
    }

    cdiff /= (double)(size);

    return cdiff;
  }

  // create and add a visual template to the collection
  int Visual_Template_Match::create_template(double x_pc, double y_pc, double th_pc)
  {
    //templates.resize(templates.size() + 1);
    //Visual_Template * new_template = &(*(templates.end() - 1));

    //new_template->id = templates.size() - 1;
    //double * column_sum_ptr = &current_view[0];
    //new_template->column_sum.reserve(TEMPLATE_SIZE);
    //for (int i = 0; i < TEMPLATE_SIZE; i++)
    //  new_template->column_sum.push_back(*(column_sum_ptr++));

    //new_template->x_pc = x_pc;
    //new_template->y_pc = y_pc;
    //new_template->th_pc = th_pc;
    //new_template->decay = VT_ACTIVE_DECAY;

    return templates.size() - 1;
  }

  // compare a visual template to all the stored templates, allowing for 
  // slen pixel shifts in each direction
  // returns the matching template and the MSE
  void Visual_Template_Match::compare(double &vt_err, unsigned int &vt_match_id)
  {
    //if (templates.size() == 0)
    //{
    //  vt_err = DBL_MAX;
    //  vt_error = vt_err;
    //  return;
    //}

    //double *column_sum = &current_view[0];
    //double mindiff, cdiff;
    //mindiff = DBL_MAX;

    //vt_err = DBL_MAX;
    //int min_template = 0;

    //double *template_ptr;
    //double *column_ptr;
    //double *template_row_ptr;
    //double *column_row_ptr;
    //double *template_start_ptr;
    //double *column_start_ptr;
    //int row_size;
    //int sub_row_size;
    //double *column_end_ptr;
    //Visual_Template vt;

    //int offset;

    //BOOST_FOREACH(vt, templates)
    //{
    //  // for each vt try matching the view at different offsets
    //  // try to fast break based on error alrady great than previous errors
    //  // handles 2d images shifting only in the x direction
    //  // note I haven't tested on a 1d yet.
    //  for (offset = 0; offset < VT_SHIFT_MATCH * 2 + 1; offset += VT_STEP_MATCH)
    //  {
    //    cdiff = 0;
    //    template_start_ptr = &vt.column_sum[0] + offset;
    //    column_start_ptr = &column_sum[0] + VT_SHIFT_MATCH;
    //    row_size = TEMPLATE_X_SIZE;
    //    column_end_ptr = &column_sum[0] + TEMPLATE_SIZE - VT_SHIFT_MATCH;
    //    sub_row_size = TEMPLATE_X_SIZE - 2 * VT_SHIFT_MATCH;

    //    for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr += row_size, template_row_ptr += row_size)
    //    {
    //      for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
    //      {
    //        cdiff += abs(*column_ptr - *template_ptr);
    //      }

    //      // fast breaks
    //      if (cdiff > mindiff)
    //        break;
    //    }

    //    if (cdiff < mindiff)
    //    {
    //      mindiff = cdiff;
    //      min_template = vt.id;
    //    }
    //  }

    //}

    //vt_err = mindiff / (double)(TEMPLATE_SIZE - 2 * VT_SHIFT_MATCH*TEMPLATE_Y_SIZE);
    //vt_match_id = min_template;

    //vt_error = vt_err;
  }

  double Visual_Template_Match::compare_two(int vt_id1, int vt_id2)
  {
    double cdiff = 0;
    //int index;
    //Visual_Template * template1, *template2;

    //template1 = &templates[vt_id1];
    //template2 = &templates[vt_id2];


    //for (index = 0; index < TEMPLATE_SIZE; index++)
    //{
    //  cdiff += abs(template2->column_sum[index] - template1->column_sum[index]);
    //}

    //// cdiff = sum(cdiff) / (cwl - offset);
    //cdiff /= (double)(TEMPLATE_SIZE);

    return cdiff;
  }

  int Visual_Template_Match::image_compare_with_template(Mat gray_image, Mat depth_image, double x, double y, double th) {
    int vt_id;
    Mat sub_gray_image = gray_image.rowRange(IMAGE_VT_Y_RANGE_MIN, IMAGE_VT_Y_RANGE_MAX).
      clone().
      colRange(IMAGE_VT_X_RANGE_MIN, IMAGE_VT_X_RANGE_MAX).
      clone();
    Mat sub_depth_image = depth_image.rowRange(IMAGE_VT_Y_RANGE_MIN, IMAGE_VT_Y_RANGE_MAX).
      clone().
      colRange(IMAGE_VT_X_RANGE_MIN, IMAGE_VT_X_RANGE_MAX).
      clone();

    vector<double> gray_image_x_sums, depth_image_x_sums;
    reduce(sub_gray_image, gray_image_x_sums, 0, CV_REDUCE_SUM);
    reduce(sub_depth_image, depth_image_x_sums, 0, CV_REDUCE_SUM);
    double gray_image_sums  = accumulate(gray_image_x_sums.begin(), gray_image_x_sums.end(), 0);
    //double depth_image_sums = accumulate(depth_image_x_sums.begin(), depth_image_x_sums.end(), 0);

    for(int i=0; i<gray_image_x_sums.size(); ++i)
        gray_image_x_sums[i] /= gray_image_sums;
    for(int i=0; i<depth_image_x_sums.size(); ++i)
        depth_image_x_sums[i] *= (4096.0/255.0/DEPTH_SUM);

    Mat min_offset = Mat::ones(numvts, 1, CV_64F);
    Mat min_diff   = Mat::ones(numvts, 1, CV_64F);

    for (int i=0; i<numvts; ++i) {
      templates[i].decay -= VT_GLOBAL_DECAY;
      if (templates[i].decay < 0)
        templates[i].decay = 0;

      vector<double> result;
      rs_compare_segments(gray_image_x_sums, depth_image_x_sums, templates[i].rgb_column_sum, templates[i].depth_column_sum, VT_SHIFT_MATCH, gray_image_x_sums.size(), result);

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
      struct eh_sim::td_visual_template new_temp;
      new_temp.id = numvts;
      new_temp.rgb_column_sum = gray_image_x_sums;
      new_temp.depth_column_sum = depth_image_x_sums;
      new_temp.decay = VT_ACTIVE_DECAY;
      new_temp.gc_x = x;
      new_temp.gc_y = y;
      new_temp.hdc  = th;
      new_temp.first = 1;
      new_temp.numexp = 0;
      templates.push_back(new_temp);
      vt_id = numvts;
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
  void Visual_Template_Match::rs_compare_segments(vd mono_seg1, vd depth_seg1, vd mono_seg2, vd depth_seg2, int slen, int cwl, vd result) {
    int mindiff   = pow(10 , 8);
    int minoffset = 0;
    for (int offset=0; offset<=slen; ++offset) {
      Mat mono_diff = abs(Mat(mono_seg1).rowRange(0+offset, cwl) - Mat(mono_seg2).rowRange(0, cwl-offset)).t();
      Mat depth_diff = abs(Mat(depth_seg1).rowRange(0+offset, cwl) - Mat(depth_seg2).rowRange(0, cwl-offset)).t();
      double cdiff = (accumulate(mono_diff.begin<double>(), mono_diff.end<double>(), 0) * VT_RGB_WEIGHT +
        accumulate(depth_diff.begin<double>(), depth_diff.end<double>(), 0) * VT_DEPTH_WEIGHT) / (cwl - offset);

      if (cdiff < mindiff) {
        mindiff = cdiff;
        minoffset = offset;
      }
    }

    for (int offset=1; offset<=slen; ++offset) {
      Mat mono_diff = abs(Mat(mono_seg1).rowRange(0, cwl-offset) - Mat(mono_seg2).rowRange(0+offset, cwl)).t();
      Mat depth_diff = abs(Mat(depth_seg1).rowRange(0, cwl-offset) - Mat(depth_seg2).rowRange(0+offset, cwl)).t();
      double cdiff = (accumulate(mono_diff.begin<double>(), mono_diff.end<double>(), 0) * VT_RGB_WEIGHT +
          accumulate(depth_diff.begin<double>(), depth_diff.end<double>(), 0) * VT_DEPTH_WEIGHT) / (cwl - offset);

      if (cdiff < mindiff) {
        mindiff = cdiff;
        minoffset = offset * -1;
      }
    }

    result.push_back(minoffset);
    result.push_back(mindiff);
  }

  void Visual_Template_Match::do_log(int frame, double *column_sum, int IMAGE_WIDTH)
  {

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

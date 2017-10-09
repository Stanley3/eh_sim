#include "utils/utils.h"
#include "gc_multi.h"
#include <iostream>
#include "visual_template_match.h"
#include <string.h>

using namespace std;
using namespace boost;

int main() {
  string nips_data_set;
  string running_mode;
  string file_path;
  ptree settings, vt_settings, general_settings, gc_settings;

  read_ini("config/configs.txt", settings);
  get_setting_child(general_settings, settings, "general", true);
  get_setting_child(vt_settings, settings, "vt", true);
  get_setting_child(gc_settings, settings, "gc", true);
  get_setting_from_ptree(nips_data_set, general_settings, "nips_data_set", (string)"");
  get_setting_from_ptree(running_mode, general_settings, "running_mode", (string)"");

  //int gc_neuronsheet_x;
  //get_setting_from_ptree(gc_neuronsheet_x, vt_settings, "gc_neuronsheet_x", 40);
  //gc_settings.put("gc_neuronsheet_x", gc_neuronsheet_x);

  eh_sim::Visual_Template_Match * vt_match;

  vt_match = new eh_sim::Visual_Template_Match(vt_settings);

  eh_sim::Grid_Cell * gc_multi;
  gc_multi = new eh_sim::Grid_Cell(gc_settings);
  double const array[] = {11,16.5,24.75,37.125,55.6875};
  gc_multi->set_lambda(array, sizeof(array)/sizeof(double));

  if (((string)"IJCAI").compare(nips_data_set) == 0) {
    file_path = "/home/jo/matlab/eh_slam_nips_20140523-11/data_set/20130806_RCT_2/";
    vt_match->set_rgb_weight(0.85);
    vt_match->set_depth_weight(0.15);
    vt_match->set_y_range(0, 119);

    gc_multi->set_gc_alpha(25);

    if ((((string)"RECORDING").compare(running_mode) == 0) ||
        ((string)"SIMULATION").compare(running_mode) == 0)
      vt_match->set_match_threshold(0.14);
    else
      vt_match->set_match_threshold(0.065);
  } else if (((string)"ICRA").compare(nips_data_set) == 0) {
    file_path = "/home/jo/matlab/eh_slam_nips_20140523-11/data_set/20130903_cw_02/";
    vt_match->set_rgb_weight(0.9);
    vt_match->set_depth_weight(0.1);
    vt_match->set_y_range(0, vt_match->get_image_height() - 1);

    gc_multi->set_gc_alpha(1);

    if ((((string)"RECORDING").compare(running_mode) == 0) ||
        ((string)"SIMULATION").compare(running_mode) == 0)
      vt_match->set_match_threshold(0.1);
    else
      vt_match->set_match_threshold(0.065);
  }

  vt_match->set_x_range(0, vt_match->get_image_width() - 1);
  vt_match->set_depth_sum(vt_match->get_x_range_max(), vt_match->get_y_range_max());
  struct eh_sim::td_visual_template first_temp;
  vt_match->init_template_one(first_temp);

  vector<string> file_names;
  get_all_files((file_path + "images/").c_str(), file_names);
  cout << "file path = "   << file_path + "images/" << "\n";
  cout << "image count = " << file_names.size() << "\n";

  vector<odometry_raw> image_infos;
  get_all_infos(file_names, image_infos);

  Mat pos;
  Mat hd;
  interpolate(file_names.size(), 500, vt_match->get_interpl_num(), image_infos, pos, hd);
  cout << "interpl_num = " << vt_match->get_interpl_num() << "\n";
  cout << "pos.size = " << pos.size() << "\n";
  cout << "hd.size = " << hd.size() << endl;

  int pre_image_counter = -1;
  int image_counter;
  int live_plot = 10;
  Mat gray_image, depth_image;
  for(int i=1; i<pos.cols; ++i) {
    double vtrans, vrot, head_direction;
    vtrans = sqrt(pow(pos.at<double>(0, i) - pos.at<double>(0, i-1), 2) +
        pow(pos.at<double>(1, i) - pos.at<double>(1, i-1), 2));
    vrot = clip_rad_360(hd.at<double>(0, i) - hd.at<double>(0, i-1));
    head_direction = clip_rad_360(vt_match->get_prev_template().hdc + vrot);

    gc_multi->set_gc_v(vtrans, head_direction);
    image_counter = floor((float)i / vt_match->get_interpl_num());

    if (image_counter != pre_image_counter) {
      image_to_gray(file_path + "images/" +  file_names[image_counter], gray_image);
      get_depth_image(file_path + "dimages/" + file_names[image_counter], depth_image);
      pre_image_counter = image_counter;
    }

    if ((i+1) % live_plot == 1) {
      double vtrans, vrot;
      vtrans = sqrt(pow(pos.at<double>(0, i) - pos.at<double>(0, i-live_plot), 2) +
          pow(pos.at<double>(1, i) - pos.at<double>(1, i-live_plot), 2));
      vrot = clip_rad_360(hd.at<double>(0, i) - hd.at<double>(0, i-live_plot));
      //[pc_x, pc_y] = gc_get_pos_xy(pc_activity);
      double pc_x, pc_y;
      int vt_id = vt_match->image_compare_with_template(gray_image, depth_image, pc_x, pc_y, head_direction);
      //eh_posecell_iteration(vt_id, vtrans, hd);
      //eh_gridcell_reset(pc_activity,vt_id,pc_x,pc_y);
      //eh_cognitive_map(vt_id,vtrans,vrot,pc_x,pc_y,hd);
      vt_match->set_prev_vt_id(vt_id);
      //pc_recording;
      //gc_plot;
    }
  }

  return 0;
}

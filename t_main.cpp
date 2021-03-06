#include "utils/utils.h"
#include "gc_multi.h"
#include "visual_template_match.h"
#include "eh_congitive_map.h"
#include "eh_posecell.h"
#include <execinfo.h>
#include <iostream>
#include <string.h>

using namespace std;
using namespace boost;

eh_sim::Visual_Template_Match * vt_match;
eh_sim::Grid_Cell * gc_multi;
eh_sim::Cognitive_Map * cg_map;
eh_sim::PoseCell * pc;

void handler(int sig) {
  void *array[100];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 100);
  cout << endl;

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

void eh_gridcell_reset(int vt_id, double gc_x, double gc_y) {
  double dx = vt_match->get_gc_x(vt_id) - gc_x;
  double dy = vt_match->get_gc_y(vt_id) - gc_y;

  if (vt_match->get_vt_matched()) {
    cout << "Reset!" << "\n";
    cout << "dx = " << dx << "\n";
    cout << "dy = " << dy << "\n";

    for (int i=0; i<gc_multi->get_number_of(); ++i) {
      Mat gca = gc_multi->get_gcs_s(i).reshape(0, gc_multi->get_gc_neuronsheet_x());
      Mat shift_row_gca;
      Mat shift_row_gca2;
      shiftRow(shift_row_gca, gca, floor(dx+1));
      shiftRow(shift_row_gca2, gca, floor(dx));
      gca = shift_row_gca.mul(dx - floor(dx)) + shift_row_gca2.mul(floor(dx+1)-dx);

      Mat shift_col_gca;
      Mat shift_col_gca2;
      shiftCol(shift_col_gca, gca, floor(dy+1));
      shiftCol(shift_col_gca2, gca, floor(dy));
      gca = shift_col_gca * (dy - floor(dy)) + shift_col_gca2 * (floor(dy+1) - dy);

      gc_multi->set_gcs_s(i, gca.reshape(0, gc_multi->get_gc_ncells()).t());

    }
  }
}


int main() {
  //signal(SIGSEGV, handler);
  //signal(SIGABRT, handler);
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


  vt_match = new eh_sim::Visual_Template_Match(vt_settings);

  gc_multi = new eh_sim::Grid_Cell(gc_settings);
  double const array[] = {11,16.5,24.75,37.125,55.6875};
  gc_multi->set_lambda(array, sizeof(array)/sizeof(double));

  if (((string)"IJCAI").compare(nips_data_set) == 0) {
    file_path = "/media/E/eh_slam_nips_20140523-11/data_set/data_set/20130806_RCT_2/";
    vt_match->set_rgb_weight(0.85);
    vt_match->set_depth_weight(0.15);
    vt_match->set_y_range(0, 120);

    gc_multi->set_gc_alpha(25);

    if ((((string)"RECORDING").compare(running_mode) == 0) ||
        ((string)"SIMULATION").compare(running_mode) == 0)
      vt_match->set_match_threshold(0.14);
    else
      vt_match->set_match_threshold(0.065);
  } else if (((string)"ICRA").compare(nips_data_set) == 0) {
    file_path = "/media/E/eh_slam_nips_20140523-11/data_set/20130903_cw_02/";
    vt_match->set_rgb_weight(0.9);
    vt_match->set_depth_weight(0.1);
    vt_match->set_y_range(0, vt_match->get_image_height());

    gc_multi->set_gc_alpha(1);

    if ((((string)"RECORDING").compare(running_mode) == 0) ||
        ((string)"SIMULATION").compare(running_mode) == 0)
      vt_match->set_match_threshold(0.1);
    else
      vt_match->set_match_threshold(0.065);
  }

  vt_match->set_x_range(0, vt_match->get_image_width());
  vt_match->set_depth_sum(vt_match->get_x_range_max(), vt_match->get_y_range_max());
  struct eh_sim::td_visual_template first_temp;
  vt_match->init_template_one(first_temp);
  cg_map = new eh_sim::Cognitive_Map(vt_match);

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

  gc_multi->gc_multi_init();

  int pre_image_counter = -1;
  int image_counter;
  int live_plot = 10;
  Mat gray_image, depth_image;
  for(int i=1; i<pos.cols; ++i) {
    cout << "i = " << i << endl;
    double vtrans, vrot, head_direction;
    vtrans = sqrt(pow(pos.at<double>(0, i) - pos.at<double>(0, i-1), 2) +
        pow(pos.at<double>(1, i) - pos.at<double>(1, i-1), 2));
    vrot = clip_rad_360(hd.at<double>(0, i) - hd.at<double>(0, i-1));
    head_direction = clip_rad_360(vt_match->get_prev_template().hdc + vrot);

    gc_multi->set_gc_v(vtrans, head_direction);
    cout << "vtrans = " << vtrans << ", head_direction = " << head_direction << "\n";
    cout << "gc_v = \n" << gc_multi->get_gc_v() << endl;
    image_counter = floor((float)i / vt_match->get_interpl_num());

    if (image_counter != pre_image_counter) {
      image_to_gray(file_path + "images/" +  file_names[image_counter], gray_image);
      get_depth_image(file_path + "dimages/" + file_names[image_counter], depth_image);
      pre_image_counter = image_counter;
    }

    gc_multi->gc_population_activity();
    gc_multi->pc_population_activity();
    cout << "pc_activity.size = " << gc_multi->get_pc_activity().size() << endl;

    if (i > 1) {
      gc_multi->gc_hebbian_learning();
    } else {
      gc_multi->gc_w_init(i);
    }

    //double vtrans, vrot;
    double pc_x, pc_y;
    if ((i+1) % live_plot == 1) {
      vtrans = sqrt(pow(pos.at<double>(0, i) - pos.at<double>(0, i-live_plot), 2) +
          pow(pos.at<double>(1, i) - pos.at<double>(1, i-live_plot), 2));
      vrot = clip_rad_360(hd.at<double>(0, i) - hd.at<double>(0, i-live_plot));
      gc_multi->gc_get_pos_xy(&pc_x, &pc_y);
      cout << "gc_get_pos_xy done" << "\n";
      int vt_id = vt_match->image_compare_with_template(gray_image, depth_image, pc_x, pc_y, head_direction);
      cout << "image_compare_with_template done" << "\n";
      //eh_posecell_iteration(vt_id, vtrans, hd);
      //eh_gridcell_reset(pc_activity,vt_id,pc_x,pc_y);
      //eh_cognitive_map(vt_id,vtrans,vrot,pc_x,pc_y,hd);
      //pc_activity not use in eh_gridcell_reset;
      pc = new eh_sim::PoseCell();
      cout << "New PoseCell done" << "\n";
      pc->eh_posecell_iteration(vt_id, vtrans, vrot, gc_multi->get_gc_alpha());
      cout << "eh_posecell_iteration done" << "\n";
      eh_gridcell_reset(vt_id, pc_x, pc_y);
      cout << "eh_gridcell_reset done" << "\n"; // after 1
      cg_map->start_cognitive_map(vt_id, vtrans, vrot, pc_x, pc_y, head_direction);
      cout << "start_cognitive_map done" << "\n";
      vt_match->set_prev_vt_id(vt_id);
      cout << "set_prev_vt_id done" << "\n";
      //pc_recording;
      //gc_plot; 
    }
  }

  return 0;
}

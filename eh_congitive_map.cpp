#include "eh_congitive_map.h"

namespace eh_sim {
  Cognitive_Map::Cognitive_Map(Visual_Template_Match vtm) {
    numexps = 1;
    curr_exp_id = 1;
    accum_delta_x = 0;
    accum_delta_y = 0;
    accum_delta_facing = 0;
    EXP_DELTA_PC_THRESHOLD = 3;
    EXP_CORRECTION = 0.5;
    EXP_LOOPS = 100;

    exp_history.push_back(1);
    struct ex_map first_map;
    first_map.gc_x = vtm.get_gc_x(0);
    first_map.gc_y = vtm.get_gc_y(0);
    first_map.hdc  = vtm.get_hdc(0);
    first_map.x_m  = 0;
    first_map.y_m  = 0;
    first_map.facing_rad = 0;
    first_map.vt_id = 1;
    first_map.numlinks = 0;
    exps.push_back(first_map);
  }

  void Cognitive_Map::Create_New_Exp(int curr_exp_id, int new_exp_id, double gc_x, double gc_y, double hdc, int vt_id) {
    exps[curr_exp_id].numlinks += 1;
    exps[curr_exp_id].links[exps[curr_exp_id].numlinks].d = sqrt(pow(accum_delta_x, 2) + pow(accum_delta_y, 2));
    exps[curr_exp_id].links[exps[curr_exp_id].numlinks].exp_id = new_exp_id;
    exps[curr_exp_id].links[exps[curr_exp_id].numlinks].heading_rad = Get_Signed_Delta_Rad(exps[curr_exp_id].facing_rad, atan2(accum_delta_y, accum_delta_x));
    exps[curr_exp_id].links[exps[curr_exp_id].numlinks].facing_rad = Get_Signed_Delta_Rad(exps[curr_exp_id].facing_rad, accum_delta_facing);

    struct ex_map new_exp;
    new_exp.gc_x = gc_x;
    new_exp.gc_y = gc_y;
    new_exp.hdc  = hdc;
    new_exp.vt_id = vt_id;
    new_exp.x_m   = exps[curr_exp_id].x_m + accum_delta_x;
    new_exp.y_m   = exps[curr_exp_id].y_m + accum_delta_y;
    new_exp.facing_rad = clip_rad_180(accum_delta_facing);
    vector<struct ex_map>::iterator it = exps.begin();
    exps.insert(it + new_exp_id, new_exp);

    vtm.set_numexp(vt_id, vtm.get_numexp(vt_id) + 1);
    vtm.add_exp_to_vt(vt_id, new_exp_id);
  }

  void Cognitive_Map::start_cognitive_map(int vt_id, double vtrans, double vrot, double gc_x, double gc_y, double hdc) {
    accum_delta_facing = clip_rad_180(accum_delta_facing + vrot);
    accum_delta_x += vtrans * cos(accum_delta_facing);
    accum_delta_y += vtrans * sin(accum_delta_facing);


    double delta_pc = sqrt(pow(Get_Min_Delta(exps[curr_exp_id].gc_y, gc_y, vtm.get_gc_neuronsheet_x()),2)+ \
                      pow(Get_Min_Delta(exps[curr_exp_id].gc_y, gc_y, vtm.get_gc_neuronsheet_x()),2));

    if (vtm.get_numexp(vt_id) == 0 || delta_pc > EXP_DELTA_PC_THRESHOLD || \
        vtm.get_vt_repeat() > VT_REPEAT) {
      ++numexps;
      Create_New_Exp(curr_exp_id, numexps, gc_x, gc_y, hdc, vt_id);

      prev_exp_id = curr_exp_id;
      curr_exp_id = numexps;

      accum_delta_x = 0;
      accum_delta_y = 0;
      accum_delta_facing = exps[curr_exp_id].facing_rad;

      if (vtm.get_vt_repeat() > 2)
        vtm.set_vt_repeat(0);
    } else if ( vt_id != prev_vt_id) {
      int matched_exp_id = 0;
      int matched_exp_count = 0;

      for (int search_id; i<vtm.get_numexp(vt_id); ++search_id) {
      }
    }
  }
}

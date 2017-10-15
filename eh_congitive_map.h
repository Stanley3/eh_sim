#include "utils/utils.h"
#include "visual_template_match.h"

using namespace std;

namespace eh_sim {
  struct exp_link {
    int exp_id;
    double d;
    double heading_rad;
    double facing_rad;
  };

  struct ex_map {
    double gc_x;
    double gc_y;
    double hdc;
    double x_m;
    double y_m;
    double facing_rad;

    int    vt_id;
    int    numlinks;

    vector<struct exp_link> links;

  };

  class Cognitive_Map {
    public:
      Cognitive_Map(eh_sim::Visual_Template_Match );
      ~Cognitive_Map();
      void start_cognitive_map(int, double, double, double, double, double);

    private:
      int curr_exp_id;
      int numexps;
      int prev_exp_id;
      int vt_conflict_num;
      int VT_REPEAT;
      double accum_delta_x;
      double accum_delta_y;
      double accum_delta_facing;
      double EXP_DELTA_PC_THRESHOLD;
      double EXP_CORRECTION;
      double EXP_LOOPS;

      Visual_Template_Match vtm;

      vector<int> exp_history;
      vector<struct ex_map> exps;

      void Create_New_Exp(int, int, double, double, double, int);
  };
}

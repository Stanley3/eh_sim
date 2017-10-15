#include "utils/utils.h"

using namespace cv;

namespace eh_sim {
  class PoseCell {
    public:
      PoseCell();
      ~PoseCell();

      Mat get_posecells() { return pcs; }
      void eh_posecell_iteration(int, double, double, double);
    private:
      int PC_DIM_XY;
      int PC_W_E_DIM;
      int PC_W_I_DIM;
      int PC_C_SIZE_TH;
      int PC_W_E_DIM_HALF;
      int PC_W_I_DIM_HALF;

      double PC_GLOBAL_INHIB;
      double PC_VT_INJECT_ENERGY;
      double PC_W_E_VAR;
      double PC_W_I_VAR;

      Mat PC_E_XY_WRAP;
      Mat PC_I_XY_WRAP;
      Mat PC_W_EXCITE;
      Mat PC_W_INHIB;
      Mat pcs;

      void eh_create_posecell_weights(Mat &, int, double);
  };
}

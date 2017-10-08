#include <opencv2/core.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <vector>
#include <math.h>

using boost::property_tree::ptree;
using namespace cv;
using namespace std;

namespace eh_sim {
  struct gc {
    Mat s;
    Mat W;
    Mat A;
    double w;
    template<typename Archive>
      void serialize(Archive& ar, const unsigned int version)
      {
        ar & s;
        ar & W;
        ar & A;
        ar & w;
      }
  };

  class Grid_Cell {
    public:
      Grid_Cell(ptree settings);
      ~Grid_Cell();
      void set_lambda(const double *,  int);

      void set_gc_alpha(double gc_alpha) { this->gc_alpha = gc_alpha; }

      int get_use_current_w() { return USE_CURRENT_W; }
      int get_num_gridcells() { return NUM_GRIDCELLS; }
      int get_load_w_if_possible() { return LOAD_W_IF_POSSIBLE; }
      int get_save_w() { return SAVE_W; }
      int get_gc_a()   { return GC_A;   }
      int get_gc_tau() { return GC_TAU; }
      double get_gc_dt() { return GC_DT; }
      double get_gc_alpha() { return gc_alpha; }
      int get_use_periodic_network() { return USE_CURRENT_W; }
      vector<double> get_lambda() { return lambda; }

      void gc_multi_init();

      template<typename Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
          ar & gc_ncells;
          ar & gcs;
        }
    private:
      int USE_CURRENT_W;
      int NUM_GRIDCELLS;
      int LOAD_W_IF_POSSIBLE;
      int SAVE_W;
      int GC_A;
      int GC_TAU;
      int USE_PERIODIC_NETWORK;
      double GC_DT;
      int GC_NEURONSHEET_X;

      double gc_alpha;
      Mat dir_vects;
      vector<double> gc_sin_lookup;
      vector<double> gc_cos_lookup;
      vector<double> lambda;

      int gc_ncells;
      vector<struct gc> gcs;

      Mat gc_w_histroy;

      void gc_weight_init(double, struct gc &);
  };
}

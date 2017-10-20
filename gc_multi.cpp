#include "gc_multi.h"
#include "utils/utils.h"
#include <assert.h>

namespace eh_sim {
  Grid_Cell::Grid_Cell(ptree settings) {
    get_setting_from_ptree(USE_CURRENT_W, settings, "use_current_w", 0);
    get_setting_from_ptree(NUM_GRIDCELLS, settings, "num_gridcells", 5);
    get_setting_from_ptree(LOAD_W_IF_POSSIBLE, settings, "load_w_if_possible", 0);
    get_setting_from_ptree(SAVE_W, settings, "save_w", 0);
    get_setting_from_ptree(GC_A, settings, "gc_a", 1);
    get_setting_from_ptree(GC_TAU, settings, "gc_tau", 10);
    get_setting_from_ptree(GC_DT, settings, "gc_dt", 0.5);
    get_setting_from_ptree(USE_PERIODIC_NETWORK, settings, "use_periodic_network", 1);

    get_setting_from_ptree(GC_NEURONSHEET_X, settings, "gc_neuronsheet_x", 40);

    gc_ncells = pow(GC_NEURONSHEET_X, 2);

    for(int i=1; i<=GC_NEURONSHEET_X; ++i) {
    gc_sin_lookup.push_back(sin(i * 2 * M_PI / GC_NEURONSHEET_X));
      gc_cos_lookup.push_back(cos(i * 2 * M_PI / GC_NEURONSHEET_X));
    }

    Mat dirs = (Mat_<double>(2,2) << 0, M_PI / 2, M_PI, 3 * M_PI / 2);  
    Mat tmp_dirs = ((Mat)(repeat(dirs, sqrt(gc_ncells) / 2, sqrt(gc_ncells) / 2).t())).reshape(0, 1);
    dir_vects = Mat(2, tmp_dirs.cols, CV_64F);
    for(int i=0; i<tmp_dirs.cols; ++i) {
      dir_vects.at<double>(0, i) = cos(tmp_dirs.at<double>(0, i));
      dir_vects.at<double>(1, i) = sin(tmp_dirs.at<double>(0, i));
    }

  }

  Grid_Cell::~Grid_Cell() {}

  void Grid_Cell::set_lambda(const double * lambda_array,  int size) {
    for(int i=0; i<size; ++i)
      lambda.push_back(lambda_array[i]);
  }

  void Grid_Cell::set_gc_v(double vtrans, double thre) {
    gc_v = Mat::zeros(2, 1, CV_64F);
    gc_v.at<double>(0, 0) = vtrans * cos(thre);
    gc_v.at<double>(1, 0) = vtrans * sin(thre);
  }

  void Grid_Cell::gc_multi_init() {
    if (USE_CURRENT_W == 1) {
      //加载序列化的数据
      vector<struct gc> serialized_gcs;
      int serialized_gc_ncells;
      gcs = serialized_gcs;
      gc_ncells = serialized_gc_ncells;

      NUM_GRIDCELLS    = serialized_gcs.size();
      GC_NEURONSHEET_X = sqrt(serialized_gc_ncells);
      gc_w_histroy = Mat::ones(2000, NUM_GRIDCELLS, CV_64F);
      for(int i=0; i<NUM_GRIDCELLS; ++i) {
        gcs[i].w = 1.0 / NUM_GRIDCELLS;
        gc_w_histroy.at<double>(0, i) = gcs[i].w;
      }
    } else {
      gc_w_histroy = Mat::ones(2000, NUM_GRIDCELLS, CV_64F);

      for (int i=0; i<NUM_GRIDCELLS; ++i) {
        struct gc t_gc;
        t_gc.w = 1.0 / NUM_GRIDCELLS;
        gc_weight_init(lambda[i], t_gc);
        gcs.push_back(t_gc);
      }
    }
  }

  void multi_shifts(Mat shifts, Mat result) {
    for (int i=0; i<shifts.cols; ++i) {
      result.at<double>(0, i) = pow(shifts.at<double>(0,i), 2) + pow(shifts.at<double>(1,i),2);
    }
  }

  void Grid_Cell::gc_weight_init(double lambda, struct gc & gc_item) {
    int ncells = gc_ncells;
    int a      = GC_A;
    double beta;
    double gamma;
    beta = 3.0 / pow(lambda, 2);
    gamma = beta * 1.1;

    //初始化s
    srand(time(NULL));
    gc_item.s = Mat::zeros(1, ncells, CV_64F);
    for(int i=0; i<ncells; ++i) {
      gc_item.s.at<double>(0, i) = rand() / RAND_MAX;
    }

    //初始化dir_vects
    Mat dirs = (Mat_<double>(2,2) << 0, M_PI / 2, M_PI, 3 * M_PI / 2);
    Mat tmp_dirs = ((Mat)(repeat(dirs, sqrt(ncells) / 2, sqrt(ncells) / 2).t())).reshape(0, 1);
    dir_vects = Mat(2, tmp_dirs.cols, CV_64F);
    for(int i=0; i<tmp_dirs.cols; ++i) {
      dir_vects.at<double>(0, i) = cos(tmp_dirs.at<double>(0, i));
      dir_vects.at<double>(1, i) = sin(tmp_dirs.at<double>(0, i));
    }

    vector<double> x;
    for (int i=0; i<GC_NEURONSHEET_X; ++i) {
      x.push_back(i - (double)(GC_NEURONSHEET_X - 1) / 2);
    }
    Mat X, Y;
    meshgrid(x, x, X, Y);

    Mat tmp_x = ((Mat)X.t()).reshape(0, 1);
    Mat tmp_y = ((Mat)Y.t()).reshape(0, 1);
    Mat m_x = Mat(2, tmp_x.cols, CV_64F);
    for(int i=0; i<tmp_x.cols; ++i) {
      m_x.at<double>(0, i) = tmp_x.at<double>(0, i);
      m_x.at<double>(1, i) = tmp_y.at<double>(0, i);
    }

    double ell = (Y.at<double>(1, 0) - Y.at<double>(0, 0)) * 2;
    vector<double> cell_dists;
    for (int i=0; i<m_x.cols; ++i)
      cell_dists.push_back(sqrt(pow(m_x.at<double>(0,i), 2) + pow(m_x.at<double>(1,i), 2)));

    double w_sparse_thresh = (-1) * pow(10, -6);
    Mat squared_shift_lengths = Mat(9, ncells, CV_64F);

    for (int i=0; i<ncells; ++i) {
      if ((i % (int)round(ncells/10)) == 0) {
        printf("Generating weight matrix. %d%% done.\n", (int)(i*1.0/ncells*100) + 10);
      }

      Mat min_squared_shift_lengths = Mat(1, ncells, CV_64F);

      if (USE_PERIODIC_NETWORK == 1) {
        Mat tmp = repeat(m_x.colRange(i, i+1), 1, ncells) - m_x - dir_vects.mul(ell);
        Mat shifts = tmp;
        multi_shifts(shifts, squared_shift_lengths.rowRange(0, 1));

        Mat dist_mat;
        dist_mat.release();
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F));
        dist_mat.push_back(Mat::zeros(1, ncells, CV_64F));

        shifts = tmp - dist_mat.mul(sqrt(ncells));
        multi_shifts(shifts, squared_shift_lengths.rowRange(1, 2));

        shifts = tmp + dist_mat.mul(sqrt(ncells));
        multi_shifts(shifts, squared_shift_lengths.rowRange(2, 3));

        dist_mat.release();
        dist_mat.push_back(Mat::zeros(1, ncells, CV_64F));
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F));
        shifts = tmp - dist_mat.mul(sqrt(ncells));
        multi_shifts(shifts, squared_shift_lengths.rowRange(3, 4));

        dist_mat.release();
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F));
        dist_mat.push_back(Mat::zeros(1, ncells, CV_64F));
        shifts = tmp +  dist_mat.mul(sqrt(ncells));
        multi_shifts(shifts, squared_shift_lengths.rowRange(4, 5));

        dist_mat.release();
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F));
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F));
        shifts = tmp + dist_mat.mul(sqrt(ncells)) ;
        multi_shifts(shifts, squared_shift_lengths.rowRange(5, 6));

        dist_mat.release();
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F)* (-1));
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F));
        shifts = tmp + dist_mat.mul(sqrt(ncells));
        multi_shifts(shifts, squared_shift_lengths.rowRange(6, 7));

        dist_mat.release();
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F));
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F)* (-1));
        shifts = tmp + dist_mat.mul(sqrt(ncells));
        multi_shifts(shifts, squared_shift_lengths.rowRange(7, 8));

        dist_mat.release();
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F)* (-1));
        dist_mat.push_back(Mat::ones(1, ncells, CV_64F)* (-1));
        shifts = tmp + dist_mat.mul(sqrt(ncells));
        multi_shifts(shifts, squared_shift_lengths.rowRange(8, 9));

        for(int k=0; k<squared_shift_lengths.cols; ++k) {
          double min;
          minMaxIdx(squared_shift_lengths.colRange(k, k+1), &min, 0, 0, 0);
          min_squared_shift_lengths.at<double>(0, k) = min;
        }
      } else {
        Mat shifts = repeat(m_x.colRange(i, i+1), 1, ncells) - m_x - dir_vects.mul(ell);
        multi_shifts(shifts, min_squared_shift_lengths);
      }

      Mat tmp = Mat(1, ncells, CV_64F);
      double value;
      for (int j=0; j<min_squared_shift_lengths.cols; ++j) {
        value = min_squared_shift_lengths.at<double>(0, j);
        value = a * exp((-1) * gamma * value) - exp((-1) * beta * value);
        value = value > w_sparse_thresh ? 0 : value;
        tmp.at<double>(0, j) = value;
      }
      gc_item.W.push_back(tmp);
    }

    if (USE_PERIODIC_NETWORK == 1) {
      gc_item.A = Mat::ones(1, cell_dists.size(), CV_64F);
    } else {
      double R = sqrt(ncells) / 2.0;
      double a0 = sqrt(ncells) / 32.0;
      double dr = sqrt(ncells) / 2.0;
      gc_item.A = Mat::zeros(1, cell_dists.size(), CV_64F);
      for (int i=0; i<cell_dists.size(); ++i) {
        gc_item.A.at<double>(0, i) = exp((pow((cell_dists[i] - R + dr) / dr, 2) * a0 * -1));
      }
      for (int i=0; i<cell_dists.size(); ++i) {
        if (cell_dists[i] < (R-dr))
          gc_item.A.at<double>(0, i) = 1;
      }
    }

  }
  void Grid_Cell::gc_population_activity() {
    for (int i=0; i<NUM_GRIDCELLS; ++i) {
      gc_pa_generation(gcs[i].s, gcs[i].W, gcs[i].A);
    }
  }

  void Grid_Cell::pc_population_activity() {
    pc_activity = Mat::zeros(gcs[0].s.size(), CV_64F);
    gc_activity = Mat::zeros(gcs[0].s.size(), CV_64F);

    for(int i=0; i<NUM_GRIDCELLS; ++i) {
      pc_activity = (pc_activity + gcs[i].s.mul(gcs[i].w));
      gc_activity = (gc_activity + gcs[i].s);
    }

    gc_mean_activity = gc_activity.mul(1.0 / NUM_GRIDCELLS);
    pc_mean_activity = pc_activity.mul(1.0 / NUM_GRIDCELLS);
  }

  void Grid_Cell::gc_hebbian_learning() {
    double temp_w = 0, learing_rate = 0.0005;
    double sum_w = 0;

    for (int i=0; i<NUM_GRIDCELLS; ++i) {
      gcs[i].w = gcs[i].w + ((Mat)(pc_activity.mul(learing_rate) * (gcs[i].s - gc_mean_activity).t())).at<double>(0, 0);

      if (gcs[i].w < 0)
        gcs[i].w = 0;

      if (temp_w < gcs[i].w)
        temp_w = gcs[i].w;

      sum_w += gcs[i].w * gcs[i].w;
    }

    for (int i=0; i<NUM_GRIDCELLS; ++i) 
      gcs[i].w = gcs[i].w / sqrt(sum_w);
  }

  void Grid_Cell::gc_w_init(int row) {
    for (int i=0; i<NUM_GRIDCELLS; ++i) {
      gcs[i].w = 1.0 / NUM_GRIDCELLS;
      gc_w_histroy.at<double>(row, i) = gcs[i].w;
    }
  }

  void Grid_Cell::gc_get_pos_xy(double *X, double *Y) {
    Scalar total = sum(pc_activity);
    pc_activity = pc_activity / total[0];

    cout << "total[0] = " << total[0] << endl;
    cout << "pc_activity.size = " << pc_activity.size() << endl;

    int GC_CEllS_TO_AVG = 3;
    vector<int> GC_AVG_XY_WRAP;

    for (int j=GC_NEURONSHEET_X-GC_CEllS_TO_AVG+1; j<=GC_NEURONSHEET_X; ++j)
      GC_AVG_XY_WRAP.push_back(j);
    for (int j=1; j<=GC_NEURONSHEET_X; ++j)
      GC_AVG_XY_WRAP.push_back(j);
    for (int j=1; j<=GC_CEllS_TO_AVG; ++j)
      GC_AVG_XY_WRAP.push_back(j);

    int max_index;
    minMaxIdx(pc_activity, 0, 0, 0, &max_index);

    int x, y;
    ind2sub(max_index, GC_NEURONSHEET_X, GC_NEURONSHEET_X, &y, &x);// 需测试
    pc_activity = pc_activity.reshape(0, GC_NEURONSHEET_X).t();
    Mat temp_pc_activity = Mat::ones(GC_NEURONSHEET_X, GC_NEURONSHEET_X, CV_64F);

    for (int i=x; i<=x+GC_CEllS_TO_AVG*2; ++i) {
      for (int j=x; j<=x+GC_CEllS_TO_AVG*2; ++j) {
        temp_pc_activity.at<double>(GC_AVG_XY_WRAP[i], GC_AVG_XY_WRAP[j]) =
          pc_activity.at<double>(GC_AVG_XY_WRAP[i], GC_AVG_XY_WRAP[j]);
      }
    }

    Mat x_sums, y_sums;
    for (int i=0; i<temp_pc_activity.cols; ++i) {
      x_sums.push_back(cv::sum(temp_pc_activity.colRange(i, i+1)));
      y_sums.push_back(cv::sum(temp_pc_activity.rowRange(i, i+1)));
    }
    x_sums = x_sums.t();
    y_sums = y_sums.t();

    assert(x_sums.rows == 1);
    assert(x_sums.cols == gc_sin_lookup.size());
    assert(x_sums.cols == gc_cos_lookup.size());
    assert(y_sums.rows == 1);
    assert(y_sums.cols == gc_sin_lookup.size());
    assert(y_sums.cols == gc_cos_lookup.size());

    double x_sin_sums = 0;
    double x_cos_sums = 0;
    double y_sin_sums = 0;
    double y_cos_sums = 0;

    for (int i=0; i<x_sums.cols; ++i) {
      x_sin_sums += x_sums.at<double>(0, i) * gc_sin_lookup[i];
      x_cos_sums += x_sums.at<double>(0, i) * gc_cos_lookup[i];
      y_sin_sums += y_sums.at<double>(0, i) * gc_sin_lookup[i];
      y_cos_sums += y_sums.at<double>(0, i) * gc_cos_lookup[i];
    }

    double t = atan2(x_sin_sums, x_cos_sums);
    *X = fmod(t * GC_NEURONSHEET_X / (2 * M_PI), GC_NEURONSHEET_X);
    *Y = fmod(atan2(y_sin_sums, y_cos_sums) * GC_NEURONSHEET_X / (2 * M_PI), GC_NEURONSHEET_X);
  }

  void Grid_Cell::gc_pa_generation(Mat & s, const Mat W, const Mat A) {
    double alpha = gc_alpha;
    Mat v        = gc_v;
    double dt    = 0.5;
    double tau   = 5;

    Mat B = (1 +  dir_vects.mul(alpha).t() * v.mul(alpha)).t();
    Mat s_inputs = (W * s.t()).t() + B;

    for (int i=0; i<s_inputs.rows; ++i) {
      for (int j=0; j<s_inputs.cols; ++j) {
        double value = s_inputs.at<double>(i, j);
        s_inputs.at<double>(i, j) = value > 0 ? value : 0;
      }
    }
    s += dt * (s_inputs - s ) * (1.0 / tau);
  }
}

#include "eh_posecell.h"

namespace eh_sim {
  PoseCell::PoseCell() {
    PC_VT_INJECT_ENERGY = 0.1;
    PC_DIM_XY = 32;
    PC_W_E_DIM = 14;
    PC_W_I_DIM = 10;
    PC_GLOBAL_INHIB = 0.00002;
    PC_W_E_VAR = 1;
    PC_W_I_VAR = 2;

    cout << "****new PoseCell****" << "\n";

    eh_create_posecell_weights(PC_W_EXCITE, PC_W_E_DIM, PC_W_E_VAR);
    eh_create_posecell_weights(PC_W_INHIB, PC_W_I_DIM, PC_W_I_VAR);

    PC_W_E_DIM_HALF = floor(PC_W_E_DIM / 2);
    PC_W_I_DIM_HALF = floor(PC_W_I_DIM / 2);

    vector<int> v(PC_W_E_DIM_HALF * 2 + PC_DIM_XY);
    std::vector<int>::iterator v_b = std::begin(v);
    iota(v_b, v_b + PC_W_E_DIM_HALF, PC_DIM_XY - PC_W_E_DIM_HALF + 1);
    iota(v_b + PC_W_E_DIM_HALF, v_b + PC_W_E_DIM_HALF + PC_DIM_XY, 1);
    iota(v_b + PC_W_E_DIM_HALF + PC_DIM_XY, std::end(v), 1);
    PC_E_XY_WRAP = Mat(v, true).t();
    cout << "PC_E_XY_WRAP = \n" << PC_E_XY_WRAP << "\n";

    vector<int> z(PC_W_I_DIM_HALF * 2 + PC_DIM_XY);
    std::vector<int>::iterator z_b = std::begin(z);
    iota(z_b, z_b + PC_W_I_DIM_HALF, PC_DIM_XY - PC_W_I_DIM_HALF + 1);
    iota(z_b + PC_W_I_DIM_HALF, z_b + PC_W_I_DIM_HALF + PC_DIM_XY, 1);
    iota(z_b + PC_W_I_DIM_HALF + PC_DIM_XY, std::end(z), 1);
    PC_I_XY_WRAP = Mat(z, true).t();
    cout << "PC_I_XY_WRAP = \n" << PC_I_XY_WRAP << endl;

    pcs = Mat::zeros(PC_DIM_XY, PC_DIM_XY, CV_64F);
    int x_pc = floor(PC_DIM_XY / 2) + 1;
    int y_pc = floor(PC_DIM_XY / 2) + 1;
    pcs.at<double>(x_pc, y_pc) = 1;
  }

  PoseCell::~PoseCell() {
    cout << "PoseCell 析构函数被调用\n";
  }

  void PoseCell::eh_create_posecell_weights(Mat &weight, int dim, double var) {
    int dim_centre = floor(dim/2) + 1;

    weight = Mat::zeros(dim, dim, CV_64F);
    double sum = 0;
    for(int x=0; x<dim; ++x) {
      for (int y=0; y<dim; ++y) {
        double value = 1/(var*sqrt(2*M_PI))*exp((-pow((x-dim_centre), 2)-pow((y-dim_centre) ,2))/(2*var*var)); 
        weight.at<double>(x, y) = value;
        sum += value;
      }
    }
    weight.mul(1.0/sum);
  }

  void PoseCell::eh_posecell_iteration(int vt_id, double vtrans, double vrot, double gc_alpha) {
    vtrans = gc_alpha * vtrans;

    Mat pca_new = Mat::zeros(PC_DIM_XY, PC_DIM_XY, CV_64F);
    for (int x=0; x<PC_DIM_XY; ++x) {
      for (int y=0; y<PC_DIM_XY; ++y) {
        if (fabs(pcs.at<double>(x,y) - 0) >= 0.000001) {
          for(int t_x=x; t_x<PC_W_E_DIM; ++t_x) {
            for (int t_y=y; t_y<PC_W_E_DIM; ++t_y) {
              pca_new.at<double>(PC_E_XY_WRAP.at<int>(0, t_x), PC_E_XY_WRAP.at<int>(0, t_y)) +=
                PC_W_EXCITE.at<double>(t_x - x, t_y -y) * pcs.at<double>(x, y);
            }
          }
        }
      }
    }

    pcs = pca_new;
    pca_new = Mat::zeros(PC_DIM_XY, PC_DIM_XY, CV_64F);
    for (int x=0; x<PC_DIM_XY; ++x) {
      for (int y=0; y<PC_DIM_XY; ++y) {
        if (fabs(pcs.at<double>(x, y) - 0) >= 0.000001) {
          for (int t_x=x; t_x<PC_W_I_DIM; ++t_x) {
            for (int t_y=y; t_y<PC_W_I_DIM; ++t_y) {
              pca_new.at<double>(PC_I_XY_WRAP.at<int>(0, t_x), PC_I_XY_WRAP.at<int>(0, t_y)) +=
                pcs.at<double>(x, y) * PC_W_INHIB.at<double>(t_x - x, t_y -y );
            }
          }
        }
      }
    }

    pcs = pcs - pca_new;
    for(int i=0; i<pcs.rows; ++i) {
      for (int j=0; j<pcs.cols; ++j) {
        if (pcs.at<double>(i, j) >= PC_GLOBAL_INHIB) 
          pcs.at<double>(i, j) -= PC_GLOBAL_INHIB;
        else
          pcs.at<double>(i, j) = 0;
      }
    }

    double total = cv::sum(pcs)[0];
    pcs = pcs.mul(1.0 / total);

    double dir = vrot;
    Mat tmp;
    if (fabs(dir - 0) <= 0.000001) {
      shiftRow(tmp, pcs, 1);
      pcs = pcs.mul(1 - vtrans) + tmp;
    } else if (fabs(dir - M_PI / 2) <= 0.000001) {
      shiftCol(tmp, pcs, 1);
      pcs = pcs.mul(1 - vtrans) + tmp;
    } else if (fabs(dir - 3 * M_PI / 2) <= 0.000001) {
      shiftRow(tmp, pcs, -1);
      pcs = pcs.mul(1 - vtrans) + tmp;
    } else {

      Mat pca90 = pcs;
      dir = fmod(dir, 2 * M_PI);
      int counter = ceil(dir * 2 / M_PI) - 1;
      double dir90 = dir - floor(dir * 2 / M_PI) * M_PI / 2;
      if (counter >=0 && counter <= 2) 
        rotate(pcs, pca90, counter);
      Mat pca_new = Mat::zeros(PC_DIM_XY + 2, PC_DIM_XY + 2, CV_64F);
      pca_new.rowRange(1, pca_new.rows-1).colRange(1, pca_new.cols-1) = pca90;
      double weight_sw = vtrans * vtrans * cos(dir90) * sin(dir90);
      double weight_se = vtrans * sin(dir90) - weight_sw;
      double weight_nw = vtrans * cos(dir90) - weight_sw;
      double weight_ne = 1.0 - weight_sw - weight_se - weight_nw;

      Mat tmp_pca_new1;
      Mat tmp_pca_new2;
      Mat tmp_pca_new3;
      shiftRow(tmp_pca_new1, pca_new, 1);
      shiftRow(tmp_pca_new2, pca_new, 1);
      shiftCol(tmp_pca_new3, tmp_pca_new2, 1);
      pca_new = pca_new.mul(weight_ne) + tmp_pca_new1.mul(weight_se) + tmp_pca_new3.mul(weight_sw);

      pca90 = pca_new.rowRange(1, pca_new.rows-1).colRange(1, pca_new.cols-1);
      pca90.rowRange(1, pca90.rows).colRange(0,1) = pca90.rowRange(1, pca90.rows).colRange(0,1) + pca_new.rowRange(2, pca_new.rows-1).colRange(pca_new.cols-1, pca_new.cols);
      pca90.rowRange(0, 1).colRange(1, pca90.cols) = pca90.rowRange(0, 1).colRange(1, pca90.cols) +
        pca_new.rowRange(pca_new.rows-1, pca_new.rows).colRange(2, pca_new.cols-1);
      pca90.at<double>(0, 0) += pca_new.at<double>(pca_new.rows-1, pca_new.cols-1);

      if (counter >=0 && counter << 2) 
        rotate(pca90, pcs, 2 - counter);

    }

  }

}

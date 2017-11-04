#include "utils.h"
void get_all_files( const char * dir_name, vector<string>& file_names ) {
  // check the parameter !
  if( NULL == dir_name )
  {
    cout<<" dir_name is null ! "<<endl;
    return;
  }

  // check if dir_name is a valid dir
  struct stat s;
  lstat( dir_name , &s );
  if( ! S_ISDIR( s.st_mode ) )
  {
    cout<<"dir_name is not a valid directory !"<<endl;
    return;
  }

  struct dirent * filename;    // return value for readdir()  
  DIR * dir;                   // return value for opendir()  
  dir = opendir( dir_name );
  if( NULL == dir )
  {
    cout<<"Can not open dir "<<dir_name<<endl;
    return;
  }
  cout<<"Successfully opened the dir !"<<endl;

  /* read all the files in the dir ~ */
  while( ( filename = readdir(dir) ) != NULL )
  {
    // get rid of "." and ".."  
    if( strcmp( filename->d_name , "." ) == 0 ||
        strcmp( filename->d_name , "..") == 0 ||
        !ends_with(filename->d_name, ".jpg")   )
      continue;  
    file_names.push_back(filename->d_name);
  }
  cout << endl;

  sort(file_names.begin(), file_names.end());
}

void get_all_infos(const vector<string> &names, vector<odometry_raw>& infos) {

  vector<string> result;
  string::size_type sz;
  struct odometry_raw odo;
  for (int i=0; i<names.size(); ++i) {
    split(result, names[i], is_any_of("_"));

    odo.pos_x_raw = strtod(result[1].c_str(), NULL);
    odo.pos_y_raw = strtod(result[2].c_str(), NULL);
    ierase_all(result[3], ".jpg");
    odo.head_direction_raw = strtod(result[3].c_str(), NULL);

    infos.push_back(odo);

  }
}

void image_to_gray(string image_path, Mat & image) {
  Mat source_image = imread(image_path, CV_LOAD_IMAGE_COLOR);
  Mat gray_image;

  cvtColor(source_image, gray_image, CV_BGR2GRAY);
  Mat ones_mat = Mat::ones(240, 320, CV_8U) * 255;

  image = ones_mat - gray_image;
}

void get_depth_image(string image_path, Mat & image) {
  Mat source_image = imread(image_path);
  cvtColor(source_image, image, CV_BGR2GRAY);
}

/*
 * img_count    图片数量
 * recording_dt 两张图片之间的时间间隔(ms)
 * interpl      插值
 * infos        所有图片的x y theta信息
 * hd           headDirections
 */
void interpolate(int img_count, double recording_dt, int interpl, vector<odometry_raw> infos, Mat& pos, Mat& hd) {

  pos = Mat::zeros(3, (img_count - 1) * interpl + 1, CV_64F);
  Mat hd_t  = Mat::zeros(1, (img_count - 1) * interpl + 1, CV_64F);

  for (int i=0; i<hd_t.cols; ++i) {
    hd_t.at<double>(0, i) = infos[floor(i * 1.0 / interpl)].head_direction_raw;
  }

  for (int i=0; i<=infos.size()-2; ++i) {
    double sub_x = infos[i+1].pos_x_raw - infos[i].pos_x_raw;
    double sub_y = infos[i+1].pos_y_raw - infos[i].pos_y_raw;
    double sub_theta = infos[i+1].head_direction_raw - infos[i].head_direction_raw;

    for(int j=0; j<interpl; ++j) {
      pos.at<double>(0, i*interpl+j) = infos[i].pos_x_raw + sub_x / interpl * j;
      pos.at<double>(1, i*interpl+j) = infos[i].pos_y_raw + sub_y / interpl * j;
      pos.at<double>(2, i*interpl+j) = recording_dt / interpl * j + i * recording_dt;
    }
  }

  int last = infos.size() - 1;
  pos.at<double>(0, (img_count - 1) * interpl) = infos[last].pos_x_raw;
  pos.at<double>(1, (img_count - 1) * interpl) = infos[last].pos_y_raw;
  pos.at<double>(2, (img_count - 1) * interpl) = recording_dt *(img_count - 1);
  hd = hd_t.t();
}

void meshgrid(const vector<double> &xgv, const vector<double> &ygv, cv::Mat &X, cv::Mat &Y) {
  std::vector<double> t_x, t_y;
  for(int i = 0; i <xgv.size(); i++) t_x.push_back(xgv[i]);
  for(int i = 0; i <ygv.size(); i++) t_y.push_back(ygv[i]);

  cv::repeat(cv::Mat(t_x).t(), t_y.size(), 1, X);
  cv::repeat(cv::Mat(t_y), 1, t_x.size(), Y);
}

void ind2sub(const int sub,const int cols, const int rows,int *row,int *col) {
  *row = sub / cols;
  *col = sub % cols;
}

void shiftCol(Mat& out, Mat in, int numRight){
  if(numRight == 0){ 
    in.copyTo(out);
    return;
  }

  int ncols = in.cols;
  int nrows = in.rows;

  out = Mat::zeros(in.size(), in.type());

  numRight = numRight%ncols;
  if(numRight < 0)
    numRight = ncols+numRight;

  in(cv::Rect(ncols-numRight,0, numRight,nrows)).copyTo(out(cv::Rect(0,0,numRight,nrows)));
  in(cv::Rect(0,0, ncols-numRight,nrows)).copyTo(out(cv::Rect(numRight,0,ncols-numRight,nrows)));
}

void shiftRow(Mat& out, Mat in, int numRight){
  int ncols = in.cols;
  int nrows = in.rows;
  numRight = numRight%nrows;

  out = Mat::zeros(in.size(), in.type());

  if(numRight == 0){
    in.copyTo(out);
    return;
  }

  if(numRight < 0)
    numRight = nrows+numRight;

  in(cv::Rect(0, nrows-numRight, ncols,numRight)).copyTo(out(cv::Rect(0,0,ncols,numRight)));
  in(cv::Rect(0,0, ncols,nrows-numRight)).copyTo(out(cv::Rect(0,numRight,ncols,nrows-numRight)));
}

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <random>

#include <cmath>
#include <time.h>
#include <direct.h>
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#include <process.h>

#include <future>
#include <thread>


#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/ximgproc.hpp>


#define PI	3.141592

#define	IMG_XSIZE			640
#define	IMG_YSIZE			480

#define	MIN_MATCH_COUNT		15
#define	STEREO_Z_DEPTH_MAX	1.0e4
#define ESTIMATE_RT_RANSAC_RANDOM_SAMPLING_TIMES	3000
#define ESTIMATE_RT_RANSAC_VOTE_MIN_THRESH	100
#define ESTIMATE_RT_RANSAC_OUTLIER_TRESH	3000


#define ESTIMATE_PLANE_RANSAC_RANDOM_SAMPLING_TIMES		300
#define ESTIMATE_PLANE_RANSAC_VOTE_MIN_THRESH			5000 //[pixel]
//#define ESTIMATE_PLANE_RANSAC_OUTLIER_TRESH				50 //[mm]
#define ESTIMATE_PLANE_DISTANCE_COLOR_IMAGE_RANGE_MAX	2000
#define ESTIMATE_PLANE_DISTANCE_COLOR_IMAGE_RANGE_MIN	-5000

#define ESTIMATE_T_MATRIX_RANSAC_RANDOM_SAMPLING_TIMES		500
#define ESTIMATE_T_MATRIX_Z_RANGE_FROM_CAMERA		30000 //30000
#define ESTIMATE_T_MATRIX_RANSAC_VOTE_THRESHOLD		6
#define ESTIMATE_T_MATRIX_OUTLIER_DISTANCE_THRESHOLD	10000

#define STEREO_BLOCK_MATCHING_MINIMUM_DISPARITIES	0
#define STEREO_BLOCK_MATCHING_DISPARITIES_SIZE		16*5//16*3 //16*5
#define STEREO_BLOCK_MATCHING_SADWINDOW_SIZE		3//3-11    21 //17
#define STEREO_BLOCK_MATCHING_P1	8*3*STEREO_BLOCK_MATCHING_SADWINDOW_SIZE*STEREO_BLOCK_MATCHING_SADWINDOW_SIZE				
#define STEREO_BLOCK_MATCHING_P2	32*3*STEREO_BLOCK_MATCHING_SADWINDOW_SIZE*STEREO_BLOCK_MATCHING_SADWINDOW_SIZE
#define STEREO_BLOCK_MATCHING_DISP12MAX_DIFF		16*5
#define STEREO_BLOCK_MATCHING_PRE_FILTER_CAP		63
#define STEREO_BLOCK_MATCHING_UNIQUENESS_RATIO		15
#define STEREO_BLOCK_MATCHING_SPECKLE_WINDOW_SIZE	200//200
#define STEREO_BLOCK_MATCHING_SPECKLE_RANGE			1
#define STEREO_BLOCK_MATCHING_WLS_LAMBDA			8000.0
#define STEREO_BLOCK_MATCHING_WLS_SIGMA				1.5




#define CLOUD_REGION_HSV_S_UPPER_THRESH				87 //blue sky 52,99
#define CLOUD_REGION_HSV_V_LOWER_THRESH				40//85//96

#define SURFACE_NORMAL_PATCH_SIZE_WIDTH					10
#define SURFACE_NORMAL_PATCH_SIZE_HEIGTH				10


struct interested_point_xy {

	int x;
	int y;

};

struct RGB_color {

	unsigned char R;
	unsigned char G;
	unsigned char B;
};




using namespace std;
using namespace cv;
//using namespace cv::xfeatures2d;
using namespace cv::ximgproc;






string win_src_cam1 = "cam1";
string win_src_cam2 = "cam2";
//string win_src_cam3 = "cam3";
//string win_src_cam3 = "cam4";


string win_dst1 = "dst1";
string win_dst2 = "dst2";

string win_dst0 = "dst0";
string win_dst_mask = "mask";
string win_dst_wall_mask = "wall_mask";
string win_dst_non_texture_region_mask = "non-texture region";
string win_Disparity = "Disparity";

string sMainFolder = "d:/MyWork/";
string sFolder_save_stereo_3D = "3D";
string sFolder_save_stereo_3D_matched = "3Dmatched";
string sFolder_save_keypoints_depth = "Keypoints_depth";
string sFolder_save_keyimages = "DB_Keyimages";

string sFilePath1, sFilePath2, sFilePath3;
string sMovieFname;
string sFilePath_movie1, sFilePath_movie2;

string sFilePath_stereo_calibration_parameters;
string sSaveFilePath_cam1_parameters;
string sSaveFilePath_cam2_parameters;
string sSaveFilePath_stereo_calibration;

string sFolder_save_stereo_results;
string sSaveFilePath_stereo_3D;
string sSaveFilePath_moive_stereo;
string sSaveFilePath_moive_mask_sub_image;
string sSaveFilePath_DB_image_left;
string sSaveFilePath_DB_image_right;


string sFilePath_ground_plane_estimation_parameters;


//string sDate;
string sFname1 = "";;
string sFname2 = "";;
string sFname_cam1_paramters = "";
string sFname_cam2_paramters = "";;
string sFname_stereo_calibration = "";
string point_cloud_filename = "";
string corresponding_points_filename = "";
string keypoints_depth_filename = "";
string save_file_path_coordinate_RT = "";
string save_file_path_monitoring_zone_obstacle_pixels = "";
string save_file_path_monitoring_zone_ditch_pixels = "";
string save_file_path_camera_height_from_ground = "";
string save_file_path_surface_path_normal = "";
string save_file_path_matrix_T_use_3D_points = "";

string sFrameNo;


//HANDLE mutex;
//HANDLE hThread;











HANDLE hThread_gnuplot;
int n_flag_thread_gnuplot_exit;



Mat img_cam1_remap;
Mat img_cam2_remap;

Mat img_hsv;


Mat xyz(IMG_YSIZE, IMG_XSIZE, CV_64FC1);
Mat xyz_0(IMG_YSIZE, IMG_XSIZE, CV_64FC1);
//Mat xyz_update;
Mat mask_with_xyz(IMG_YSIZE, IMG_XSIZE, CV_8UC1, cv::Scalar(0));
Mat mask_with_xyz_0(IMG_YSIZE, IMG_XSIZE, CV_8UC1, cv::Scalar(0));
Mat ground_based_distance(IMG_YSIZE, IMG_XSIZE, CV_64FC1);
Mat camera_coordinate_origin_based_distance(IMG_YSIZE, IMG_XSIZE, CV_64FC1);

Mat estimated_R_T(12, 1, CV_64FC1);

vector<cv::KeyPoint > Keypoints_0;
vector<cv::KeyPoint > Keypoints_1;
std::vector<cv::Point2f> Keypoint0_with_xyz;
std::vector<cv::Point2f> Homography_keypoint0;
std::vector<cv::Point2f> Homography_keypoint1;
//std::vector<cv::Point2f> matched_keypoint0_with_xyz;
//std::vector<cv::Point2f> matched_keypoint1_with_xyz;

cv::Mat Descriptor0;
cv::Mat Descriptor1;
std::vector<uint>  Good_descriptor0_No;
std::vector<uint>  Good_descriptor1_No;



Mat axis_1(3, 1, CV_64FC1);
Mat axis_2(3, 1, CV_64FC1);
Mat axis_3(3, 1, CV_64FC1);
Mat axis_1n(3, 1, CV_64FC1);
Mat axis_2n(3, 1, CV_64FC1);
Mat axis_3n(3, 1, CV_64FC1);


Mat axis_dim4_1(4, 1, CV_64FC1);
Mat axis_dim4_2(4, 1, CV_64FC1);
Mat axis_dim4_3(4, 1, CV_64FC1);
Mat axis_dim4_1n(4, 1, CV_64FC1);
Mat axis_dim4_2n(4, 1, CV_64FC1);
Mat axis_dim4_3n(4, 1, CV_64FC1);

Mat axis_dim4_origin(4, 1, CV_64FC1);
Mat axis_dim4_origin_n(4, 1, CV_64FC1);




int n_frame_count_3D_data;
int n_frame_count;
int n_frame_count_keypoints_XYZ;





int n_flag_done_good_matching_keypoints;
int n_flag_done_calc_homography_matrix;
int n_flag_done_calc_essential_matrix;
int n_flag_detected_point_num_enough_to_estimate_3D_plane;

int n_flag_done_estimation_default_ground_normal_vector;




struct interested_point_xy targeted_3D_Point[IMG_YSIZE*IMG_XSIZE];
struct interested_point_xy all_3D_Point[IMG_YSIZE*IMG_XSIZE];
struct RGB_color color_table_RGB[360];





void HSV_to_RGB(double H, double S, double V, unsigned char *R, unsigned char *G, unsigned char *B);
string make_n_digit_number(int in_No, int n_digit_num);


static void saveXYZRGB(const char* filename, const Mat& mat, const Mat& color, const Mat& mask);
static void save_corresponding_points(const char* filename, const Mat& mat0, const Mat& mat1, const std::vector<cv::Point2f> &points0, const std::vector<cv::Point2f> &points1, const Mat& mask0, const Mat& mask1, const Mat& descriptor0, std::vector<uint> &descriptor0_no, const Mat& descriptor1, std::vector<uint> &descriptor1_no, int n_descriptor_vector_size);
//static void save_corresponding_points(const char* filename, const Mat& mat0, const std::vector<cv::KeyPoint > &points0, const Mat& mask0, const Mat& descriptor0, int n_descriptor_vector_size);
static void save_keypoints_depth(const char* filename, const Mat& mat0, const std::vector<cv::KeyPoint > &points0, const Mat& mask0, const Mat& descriptor0, int n_descriptor_vector_size);
int calc_and_save_T_matrix_using_3D_points(const Mat& R_mat, const Mat& T_mat, const Mat& xyz_mat0, const Mat& xyz_mat1, const std::vector<cv::Point2f> &points0, const std::vector<cv::Point2f> &points1, const Mat& mask0, const Mat& mask1, int n_ransac_loop_times, double d_ransac_thresh, double *d_Tx_estimated, double *d_Ty_estimated, double *d_Tz_estimated, double *d_alpha_estimated, double *d_T_abs_estimated, int *ransac_n_T_voted_num);
void Iterative_Closest_Point_ICP(int n_mesh_size, const Mat& xyz_mat0, const Mat& xyz_mat1, const Mat& mask0, const Mat& mask1, const Mat& cloud_mask, const Mat& R_mat, const Mat& T_mat, double d_angle_thresh_to_detect_mesh_point, std::vector<cv::Point> &mesh_img_points0, std::vector<cv::Vec3f>& mesh_point0, std::vector<cv::Vec3f>& mesh_normal0, std::vector<cv::Vec3f>& mesh_point1, double *optimum_scale_alpha);
void Iterative_Closest_Point_ICP_to_estimate_T_matrix(int n_mesh_size, const Mat& xyz_mat0, const Mat& xyz_mat1, const Mat& mask0, const Mat& mask1, const Mat& cloud_mask, const Mat& R_mat, const Mat& T_mat, double d_angle_thresh_to_detect_mesh_point, std::vector<cv::Point> &mesh_img_points0, std::vector<cv::Vec3f>& mesh_point0, std::vector<cv::Vec3f>& mesh_normal0, std::vector<cv::Vec3f>& mesh_point1, double *d_Tx_estimated, double *d_Ty_estimated, double *d_Tz_estimated);

void ICP_particle_filter_init_particles(const Mat& R_mat, const Mat& T_mat, int n_particles_num, std::vector<cv::Vec3d> &particles_T, std::vector<double> &particles_Weight, std::vector<double> &particles_Weight_accumulated, std::vector<cv::Vec3d> &particles_T_next, std::vector<double> &particles_Weight_next, std::vector<double> &particles_Weight_accumulated_next);
void ICP_particle_filter_resampling(int n_particles_num, std::vector<cv::Vec3d> &particles_T, std::vector<double> &particles_Weight, std::vector<double> &particles_Weight_accumulated, std::vector<cv::Vec3d> &particles_T_next, std::vector<double> &particles_Weight_next, std::vector<double> &particles_Weight_accumulated_next);
void ICP_particle_filter_predict(int n_particles_num, std::vector<cv::Vec3d> &particles_T, double d_rand_normal_dist_stdev);
void ICP_particle_filter_measure_calc_likelihood(int n_particles_num, std::vector<cv::Vec3d> &particles_T, std::vector<double> &particles_Weight, std::vector<double> &particles_Weight_accumulated, const Mat& R_mat, const Mat& T_mat, std::vector<cv::Vec3f>& mesh_point0, std::vector<cv::Vec3f>& mesh_normal0, std::vector<cv::Vec3f>& mesh_point1, cv::Vec3d &estimated_TxTyTz);
void ICP_particle_filter_update(int n_particles_num, std::vector<cv::Vec3d> &particles_T_next, std::vector<double> &particles_Weight_next, std::vector<double> &particles_Weight_accumulated_next, std::vector<cv::Vec3d> &particles_T, std::vector<double> &particles_Weight, std::vector<double> &particles_Weight_accumulated);

int check_step_region_pixel_using_3D_surface_patch(int x, int y, int n_mesh_width, int n_mesh_height, const Mat& xyz_mat, const Mat& mask, cv::Vec3f vec_ground_normal, cv::Vec3f vec_x_axis, double d_targeted_angle_to_detect_step, double d_targeted_angle_tolerance, double d_targeted_angle_tolerance_to_x_axis);
void Make_ground_candidated_mask_image_using_3D_surface_patch(int n_mesh_width, int n_mesh_height, const Mat& xyz_mat, const Mat& mask, double d_targeted_angle_to_detect_wall, double d_targeted_angle_tolerance, Mat& mask_ground_candidated);
void Erase_xyz_outliers_on_non_texture_region(Mat& xyz_Mat, Mat& mask_xyz, Mat& mask_non_texture_image, int n_wnd_width, int n_wnd_height, double d_thresh_depth_to_detect_outlier);

int thread_save_XYZRGB(const char* filename);////unsigned _stdcall thread_save_XYZRGB(void *p);
int thread_save_corresponding_points(const char* filename);
int thread_save_Keypoints_depth(const char* filename);
unsigned _stdcall thread_gnuplot(void *p);









int main(int argc, char **argv)
{


	std::future<int> result_thread;
	std::future<int> result_thread_3Dmatched;
	std::future<int> result_thread_xyXYZdepth;

	std::random_device rd;
	std::mt19937 mt(rd());






	const int BOARD_W = 6;
	const int BOARD_H = 5;
	const Size BOARD_SIZE = Size(BOARD_W, BOARD_H);
	const int N_CORNERS = BOARD_W * BOARD_H;
	const int N_BOARDS = 20;
	const float SCALE = 30;
	const Size IM_SIZE = Size(IMG_XSIZE, IMG_YSIZE);



	DWORD	dwSpentTime;
	DWORD	dwTime0, dwTime1;
	double	dProcessingSpeed;
	double	dProcessingSpeed_3D_data_saving;
	//int		n_processed_file_number_3D_data;



	int key;



	time_t timer;
	struct tm local;
	string s_month, s_day, s_hour, s_min, s_sec;



	timer = time(NULL);
	localtime_s(&local, &timer);


	ostringstream sout;

	sout << std::setfill('0') << std::setw(2) << local.tm_mon + 1; s_month = sout.str(); sout.str(""); sout.clear(stringstream::goodbit);
	sout << std::setfill('0') << std::setw(2) << local.tm_mday; 	s_day = sout.str(); sout.str(""); sout.clear(stringstream::goodbit);
	sout << std::setfill('0') << std::setw(2) << local.tm_hour; 	s_hour = sout.str(); sout.str(""); sout.clear(stringstream::goodbit);
	sout << std::setfill('0') << std::setw(2) << local.tm_min; 	s_min = sout.str(); sout.str(""); sout.clear(stringstream::goodbit);
	sout << std::setfill('0') << std::setw(2) << local.tm_sec;	s_sec = sout.str(); sout.str(""); sout.clear(stringstream::goodbit);


	//string sDate = to_string(1900 + local.tm_year) + to_string(local.tm_mon+1) + to_string(local.tm_mday) + to_string(local.tm_hour) + to_string(local.tm_min) + to_string(local.tm_sec);
	string sDate = to_string(1900 + local.tm_year) + s_month + s_day + s_hour + s_min + s_sec;






	Mat img_cam1(IMG_YSIZE, IMG_XSIZE, CV_8UC3);
	Mat img_cam2(IMG_YSIZE, IMG_XSIZE, CV_8UC3);



	Mat img_dst1, img_dst2;
	Mat img_gray1, img_gray2;
	Mat img_edge1, img_edge2;

	//Mat img_cloud;
	Mat img_cloud_remap;
	Mat img_wall;
	Mat img_wall_remap;
	//Mat img_edge1_sobel;
	Mat img_edge1_sobel_remap;


	Mat edge1_sobel_dx, edge1_sobel_dy;
	Mat edge1_sobel;


	//8bit mask image
	Mat img_mask_cloud1(IMG_YSIZE, IMG_XSIZE, CV_8UC1);
	Mat img_mask_cloud2(IMG_YSIZE, IMG_XSIZE, CV_8UC1);
	Mat img_mask_cloud1_remap(IMG_YSIZE, IMG_XSIZE, CV_8UC1);;
	Mat img_mask_wall(IMG_YSIZE, IMG_XSIZE, CV_8UC1);
	Mat img_mask_wall_remap(IMG_YSIZE, IMG_XSIZE, CV_8UC1);
	Mat img_mask_non_texture_region(IMG_YSIZE, IMG_XSIZE, CV_8UC1);
	Mat img_mask_non_texture_region_remap(IMG_YSIZE, IMG_XSIZE, CV_8UC1);

	Mat img_temp;

	Mat img_wrk1, img_wrk2;
	Mat img_feature_points;







	Mat map11, map12, map21, map22;


	Mat img_disparity16S = Mat(IMG_YSIZE, IMG_XSIZE, CV_16S);
	Mat img_disparity8U = Mat(IMG_YSIZE, IMG_XSIZE, CV_8UC1);



	// must be 24 bit image
	//=> cvtColor(img_temp, img_mask_non_texture_region, CV_GRAY2BGR);

	Mat	img_combined(IMG_YSIZE * 2, IMG_XSIZE * 2, CV_8UC3);
	Mat	img_view1(img_combined, cv::Rect(0, 0, IMG_XSIZE, IMG_YSIZE));
	Mat	img_view2(img_combined, cv::Rect(IMG_XSIZE, 0, IMG_XSIZE, IMG_YSIZE));
	Mat	img_view3(img_combined, cv::Rect(0, IMG_YSIZE, IMG_XSIZE, IMG_YSIZE));
	Mat	img_view4(img_combined, cv::Rect(IMG_XSIZE, IMG_YSIZE, IMG_XSIZE, IMG_YSIZE));
	Mat	img_merge(IMG_YSIZE, IMG_XSIZE, CV_8UC3);


	// must be 24 bit image
	//=>  cvtColor(img_temp, img_mask_non_texture_region, CV_GRAY2BGR);

	Mat	img2_combined(IMG_YSIZE * 2, IMG_XSIZE * 2, CV_8UC3);
	Mat	img2_view1(img2_combined, cv::Rect(0, 0, IMG_XSIZE, IMG_YSIZE));
	Mat	img2_view2(img2_combined, cv::Rect(IMG_XSIZE, 0, IMG_XSIZE, IMG_YSIZE));
	Mat	img2_view3(img2_combined, cv::Rect(0, IMG_YSIZE, IMG_XSIZE, IMG_YSIZE));
	Mat	img2_view4(img2_combined, cv::Rect(IMG_XSIZE, IMG_YSIZE, IMG_XSIZE, IMG_YSIZE));











	vector<vector<Point2f>>imagePoints1, imagePoints2;
	vector<Point2f> imageCorners1, imageCorners2;
	vector<vector < Point3f >> objectPoints;
	vector<Point3f>objectCorners;

	Mat K1, K2; //Mat cameraMatrix1, cameraMatrix2;
	Mat D1, D2; //Mat distCoeffs1, distCoeffs2;
	vector<Mat> rvecs1, rvecs2;
	vector<Mat> tvecs1, tvecs2;

	Mat R, F, E;
	Vec3d T;

	Mat R1, R2, P1, P2, Q;








	//double rms1, rms2;




	double sensor1_physical_width = 0;
	double sensor1_physical_height = 0;
	double sensor1_fovx = 0, sensor1_fovy = 0;
	double sensor1_focalLength = 0;
	Point2d sensor1_principalPoint = (0, 0);
	double sensor1_aspectRatio = 0;

	double sensor2_physical_width = 0;
	double sensor2_physical_height = 0;
	double sensor2_fovx = 0, sensor2_fovy = 0;
	double sensor2_focalLength = 0;
	Point2d sensor2_principalPoint = (0, 0);
	double sensor2_aspectRatio = 0;



	//bool found1, found2;
	//int  n_found_count;


	//int i, j;



	//int n_From_movie_frame;
	//int n_To_movie_frame;
	//int n_SLAM_frame_step;
	//int n_3D_data_saving_step;





	int n_frame_count = 0;
	//int n_current_frame_No;


	//int n_do_stereo_timing;
	//int n_save_3D_data_timing;




	/*

	int	n_Keypoints_Num_Max;


	double d_RANSAC_outlier_thresh;
	double d_RANSAC_outlier_thresh_T_matrix = 0;

	double d_object_height_minus_thresh;
	double d_object_height_plus_thresh;
	double d_ditch_height_low_thresh;
	double d_ditch_height_high_thresh;

	double d_object_distance_from_camera_thresh_min;
	double d_object_distance_from_camera_thresh_max;

	double d_object_FOV_limit_height_min = -1000;
	double d_object_FOV_limit_distance = 3000;




	int n_estimate_plane_image_x_range_min;
	int n_estimate_plane_image_x_range_max;
	int n_estimate_plane_image_y_range_min;
	int n_estimate_plane_image_y_range_max;

	int n_cloud_HSV_S_upper_thresh = int(255.0 * CLOUD_REGION_HSV_S_UPPER_THRESH / 100.0); //0-100 => 0-255
	int n_cloud_HSV_V_lower_thresh = int(255.0 * CLOUD_REGION_HSV_V_LOWER_THRESH / 100.0); //0-100 => 0-255


	int n_sky_detction_canny_edge_thresh_low = 50;
	int n_sky_detction_canny_edge_thresh_high = 200;


	int n_non_texure_region_detction_sobel_edge_thresh = 10;


	//estimate 3D plane
	int n_all_3D_points_num;
	int n_targeted_3D_point_num;

	Vec3f dominant_plane_origin;
	Vec3f dominant_plane_vector1, dominant_plane_vector2;
	Vec3f dominant_plane_normal;



	Vec3f vec_camera_x_axis = Vec3f(1, 0, 0);
	Vec3f vec_camera_y_axis = Vec3f(0, 1, 0);
	Vec3f vec_camera_z_axis = Vec3f(0, 0, 1);

	Mat backup_pca_eigen_vectors, backup_pca_eigen_values;
	Mat backup_pca_eigen_org;
	//Mat temp_pca_eigen_vectors, temp_pca_eigen_values;
	//Mat temp_pca_eigen_org;


	
	/// Obstacle detection parameters ///
	int n_voted_num_to_detect_obstacles;
	int n_voted_num_to_detect_ditch;
	//int n_voted_num_to_detect_obstacles_thresh;
	int n_obstacle_monitoring_zone_x_min;
	int n_obstacle_monitoring_zone_x_max;
	int n_obstacle_monitoring_zone_z_min;
	int n_obstacle_monitoring_zone_z_max;
	int n_obstacle_monitoring_zone_y_min;
	int n_obstacle_monitoring_zone_y_max;
	int n_obstacle_monitoring_zone_range_x;
	int n_obstacle_monitoring_zone_range_z;
	double d_obstacle_monitoring_zone_block_x_width;
	double d_obstacle_monitoring_zone_block_z_width;
	int n_obstacle_monitoring_zone_block_x_num;
	int n_obstacle_monitoring_zone_block_z_num;
	int n_obstacle_monitoring_zone_x_block_no;
	int n_obstacle_monitoring_zone_z_block_no;
	//int n_cloud_region_detection_region_y_max;

	int n_wall_mask_pixel_num_in_lower_image_region;


	int n_object_mesh_width_to_detect_step;
	int n_object_mesh_height_to_detect_step;
	double d_object_angle_tolerance_between_mesh_normal_and_ground_normal;
	double d_object_angle_tolerance_between_mesh_normal_and_x_axis;

	int n_mesh_width_to_detect_wall;
	int n_mesh_height_to_detect_wall;
	double d_angle_tolerance_between_y_axis_and_surface_patch_normal;

	double d_dot_product_between_ground_normal_and_y_axis;
	double d_angle_between_ground_normal_and_y_axis;


	double d_danger_zone_x_side_length;
	double d_danger_zone_z_forward_length;


	int		n_lower_thresh_RANSAC_data_number_to_check_credibility_estimated_ground_normal;
	int		n_upper_thresh_wall_region_pixel_number_to_check_credibility_estimated_ground_normal;


	int n_window_width_to_extract_non_texture_region;
	int n_window_height_to_extract_non_texture_region;
	double d_thresh_for_z_coordinate_difference_to_detect_depth_outlier;

	*/








	std::cout << "Stereo camera paramters file path = ";
	cin >> sFilePath_stereo_calibration_parameters;











	FileStorage fs_stereo_param(sFilePath_stereo_calibration_parameters, FileStorage::READ);


	fs_stereo_param["K1"] >> K1;

	fs_stereo_param["K2"] >> K2;
	fs_stereo_param["D1"] >> D1;
	fs_stereo_param["D2"] >> D2;
	fs_stereo_param["R"] >> R;
	//fs_stereo_param["T"] >> T;
	//fs_stereo_param["E"] >> E;
	//fs_stereo_param["F"] >> F;

	fs_stereo_param["R1"] >> R1;
	fs_stereo_param["R2"] >> R2;
	fs_stereo_param["P1"] >> P1;
	fs_stereo_param["P2"] >> P2;
	fs_stereo_param["Q"] >> Q;

	fs_stereo_param.release();

	std::cout << "The stereo camera parameters have been loaded!\n";













	std::cout << "Main folder : ";
	std::cin >> sMainFolder;


	std::cout << "movie file (***_cam1.mov, ***_cam2.mov) ex. movie_stereo: ";
	std::cin >> sMovieFname;



	sFolder_save_stereo_3D = sMainFolder + "/3D";
	_mkdir(sFolder_save_stereo_3D.c_str());




	sFilePath_movie1 = sMainFolder + "/" + sMovieFname + "_cam1.mov";
	sFilePath_movie2 = sMainFolder + "/" + sMovieFname + "_cam2.mov";


	VideoCapture capture1(sFilePath_movie1);

	if (!capture1.isOpened()) {
		std::cout << "cannot open the movie file of cam1";
		return -1;
	}

	VideoCapture capture2(sFilePath_movie2);
	if (!capture2.isOpened()) {
		std::cout << "cannot open the movie file of cam2";
		return -1;
	}









	// set movie 1  filepath
	//sSaveFilePath_moive_stereo = sMainFolder + "/stereo_EstimatePlane_SLAM_results_start" + to_string(n_From_movie_frame) + "step" + to_string(n_SLAM_frame_step) + "ORB" + to_string(n_Keypoints_Num_Max) + "_RansacTh" + to_string((int)d_RANSAC_outlier_thresh) + "RngX" + to_string(n_estimate_plane_image_x_range_min) + "_" + to_string(n_estimate_plane_image_x_range_max) + "Y" + to_string(n_estimate_plane_image_y_range_min) + "_" + to_string(n_estimate_plane_image_y_range_max) + "_ObjHeightTh" + to_string(int(d_object_height_minus_thresh)) + "_" + to_string(int(d_object_height_plus_thresh)) + to_string(int(d_ditch_height_low_thresh)) + "_" + to_string(int(d_ditch_height_high_thresh)) + "DistThfrom" + to_string(int(d_object_distance_from_camera_thresh_min)) + "to" + to_string(int(d_object_distance_from_camera_thresh_max)) + "Cloud_HSV_S" + to_string(CLOUD_REGION_HSV_S_UPPER_THRESH) + "V" + to_string(CLOUD_REGION_HSV_V_LOWER_THRESH) + "y" + to_string(n_cloud_region_detection_region_y_max) + ".mov";
	sSaveFilePath_moive_stereo = sMainFolder + "/stereo.mov";


	VideoWriter writer(sSaveFilePath_moive_stereo, VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, Size(IMG_XSIZE, IMG_YSIZE), true);
	if (!writer.isOpened()) {
		cout << "error: movie writer" << endl;
		return -1;
	}











	initUndistortRectifyMap(K1, D1, R1, P1, IM_SIZE, CV_16SC2, map11, map12);
	initUndistortRectifyMap(K2, D2, R2, P2, IM_SIZE, CV_16SC2, map21, map22);






















	/// stereo ///

	//int minDisparity;
	//int ndisparities;
	//int SADWindowSize;
	//double minVal, maxVal;











	//StereoBM//
	//ndisparities = STEREO_BLOCK_MATCHING_DISPARITIES_SIZE;//16 * 5;
	//SADWindowSize = STEREO_BLOCK_MATCHING_SADWINDOW_SIZE;
	//Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);



	//StereoSGBM//
	//minDisparity = STEREO_BLOCK_MATCHING_MINIMUM_DISPARITIES;
	//ndisparities = STEREO_BLOCK_MATCHING_DISPARITIES_SIZE;// 16 * 10;// 16 * 5;
	//SADWindowSize = STEREO_BLOCK_MATCHING_SADWINDOW_SIZE;//21;//17
	//
	//Ptr<StereoSGBM> sgbm = StereoSGBM::create(minDisparity, ndisparities, SADWindowSize, STEREO_BLOCK_MATCHING_P1, STEREO_BLOCK_MATCHING_P2, STEREO_BLOCK_MATCHING_DISP12MAX_DIFF, STEREO_BLOCK_MATCHING_PRE_FILTER_CAP, STEREO_BLOCK_MATCHING_UNIQUENESS_RATIO);
	//sgbm->setSpeckleWindowSize(STEREO_BLOCK_MATCHING_SPECKLE_WINDOW_SIZE);
	//sgbm->setSpeckleRange(STEREO_BLOCK_MATCHING_SPECKLE_RANGE);


	












	// StereoSGBM with Weighted Least Squares filter //

	int sgbm_minDisparity = 0;
	int sgbm_numDisparities = 16 * 4;
	int sgbm_window_size = 3;
	int sgbm_P1 = 8*3*sgbm_window_size*sgbm_window_size;
	int sgbm_P2 = 32*3* sgbm_window_size*sgbm_window_size;
	int sgbm_disp12MaxDiff = 1000;// 16 * 5;// 16 * 4;
	int sgbm_preFilterCap = 63;
	int sgbm_uniquenessRatio = 15;//15
	int sgbm_speckleWindowSize = 200;
	int sgbm_speckleRange = 1;
	double wls_lmbda = 200;// 80000.0;//200; 130;
	double wls_sigma = 10.0;
	



	Mat left_for_matcher, right_for_matcher;
	Mat left_disp, right_disp;
	Mat filtered_disp;
	Mat raw_disp_vis;
	Mat filtered_disp_vis;
	Mat image_filterd_disp = Mat(IMG_YSIZE, IMG_XSIZE, CV_8UC3);
	double vis_mult = 3.0;





	Ptr<DisparityWLSFilter> wls_filter;

	Ptr<StereoSGBM> left_matcher = StereoSGBM::create(sgbm_minDisparity, sgbm_numDisparities, sgbm_window_size);




	left_matcher->setP1(sgbm_P1);
	left_matcher->setP2(sgbm_P2);
	left_matcher->setPreFilterCap(sgbm_preFilterCap);
	left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
	
	wls_filter = createDisparityWLSFilter(left_matcher);

	
	
	
	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
	





	



	////////////////////////////////////












	dwTime0 = timeGetTime();








	//genarate window
	cv::namedWindow(win_src_cam1, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(win_src_cam2, CV_WINDOW_AUTOSIZE);


	cv::namedWindow(win_dst1, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(win_dst2, CV_WINDOW_AUTOSIZE);

	cv::namedWindow(win_dst0, CV_WINDOW_AUTOSIZE);









	////////////////////////////////////
	/////////// Main loop /////////////
	////////////////////////////////////


	n_frame_count = 0;








	while (1) {






		sFrameNo = to_string(n_frame_count);









		cout << "[" << n_frame_count << "]" << endl;


		capture1 >> img_cam1;
		capture2 >> img_cam2;
















		key = waitKey(1);

		if (img_cam1.empty() || capture1.get(CV_CAP_PROP_POS_AVI_RATIO) == 1 || key == 'q') { //If EOF, CV_CAP_PROP_POS_AVI_RATIO = 1 


																							  //n_processed_file_number_3D_data = result_thread.get();

			break;

		}






		





















		remap(img_cam1, img_cam1_remap, map11, map12, INTER_LINEAR);
		remap(img_cam2, img_cam2_remap, map21, map22, INTER_LINEAR);




		cv::imshow(win_src_cam1, img_cam1_remap);
		cv::imshow(win_src_cam2, img_cam2_remap);




		cvtColor(img_cam1_remap, img_gray1, CV_BGR2GRAY);
		cvtColor(img_cam2_remap, img_gray2, CV_BGR2GRAY);










		









		/////////////////////////////////////////////////////////////////////////////////
		//// STEREO VISION 
		/////////////////////////////////////////////////////////////////////////////////




		//calc disparity
		//sbm->compute(img_gray1, img_gray2, img_disparity16S);
		//sgbm->compute(img_gray1, img_gray2, img_disparity16S);

		

		//minMaxLoc(img_disparity16S, &minVal, &maxVal);
		//img_disparity16S.convertTo(img_disparity8U, CV_8UC1, 255 / (maxVal - minVal));


		//calc 3D coordinates
		//reprojectImageTo3D(img_disparity16S, xyz, Q, true);





		//gray to BGR
		//cvtColor(img_disparity8U, img_merge, CV_GRAY2BGR);



		//cv::putText(img_merge, sFrameNo, cv::Point(10, 450), cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 255, 0), 1, CV_AA);



		//namedWindow(win_Disparity, CV_WINDOW_AUTOSIZE);
		//imshow(win_Disparity, img_merge);









		// Calc the disparity filtered by using weighted Least Squares filter //


		left_for_matcher = img_gray1.clone();
		right_for_matcher = img_gray2.clone();




		left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
		right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);


		wls_filter->setLambda(wls_lmbda);
		wls_filter->setSigmaColor(wls_sigma);
		wls_filter->filter(left_disp, img_gray1, filtered_disp, right_disp); // output: filtered_disp



		
		

		//creating a disparity map visualization (clamped CV_8U image) form CV_16S depth

		getDisparityVis(left_disp, raw_disp_vis, vis_mult);

		
		getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);


		


		cv::imshow(win_dst1, raw_disp_vis);
		cv::imshow(win_dst2, filtered_disp_vis);





		// gray to BGR //
		cvtColor(filtered_disp_vis, image_filterd_disp, CV_GRAY2BGR);








		// calc 3D corrdinates //


		reprojectImageTo3D(filtered_disp, xyz, Q, true);

	



		for (int y = 0; y < xyz.rows; y++) {
			for (int x = 0; x < xyz.cols; x++) {


				Vec3f point_xyz = xyz.at<Vec3f>(y, x);

				xyz.at<Vec3f>(y, x) *= 16.0;



				if (fabs(point_xyz[2] - STEREO_Z_DEPTH_MAX) < FLT_EPSILON || fabs(point_xyz[2]) > STEREO_Z_DEPTH_MAX) {


					mask_with_xyz.at<uchar>(y, x) = 0;
					


				}
				else {

					mask_with_xyz.at<uchar>(y, x) = 1;






				}




			}
		}




		// save XYZ and RGB data //
		if (key == 's') {

			point_cloud_filename = sFolder_save_stereo_3D + "/" + "XYZRGB" + make_n_digit_number(n_frame_count, 7) + ".txt";
			result_thread = std::async(std::launch::async, thread_save_XYZRGB, point_cloud_filename.c_str());


		}




		


























		// writer 1 //







		writer << image_filterd_disp;































































		n_frame_count++;














		BufferPoolController* c = cv::ocl::getOpenCLAllocator()->getBufferPoolController();
		if (c) {
			c->setMaxReservedSize(0);
		}















	}






	dwTime1 = timeGetTime();

	dwSpentTime = dwTime1 - dwTime0;

	dProcessingSpeed = n_frame_count / (double)dwSpentTime * 1000;
	dProcessingSpeed_3D_data_saving = n_frame_count_3D_data / (double)dwSpentTime * 1000;


	std::cout << "Processing speed:" << dProcessingSpeed << "Hz[" << n_frame_count << "files]\n" << "Saving speed:" << dProcessingSpeed_3D_data_saving << "Hz [" << n_frame_count_3D_data << "files]\n";




	writer.release();








	return 0;

}







unsigned _stdcall thread_gnuplot(void *p)
{


	FILE *gid;


	gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");


	//fprintf_s(gid, "set xrange [-2.5:2.5]\n");
	//fprintf_s(gid, "set yrange [-2.5:2.5]\n");
	//fprintf_s(gid, "set zrange [-2.5:2.5]\n");//for camera coordinates

	fprintf_s(gid, "set xrange [-100:100]\n");
	fprintf_s(gid, "set yrange [-100:100]\n");
	fprintf_s(gid, "set zrange [100:-100]\n");

	//fprintf_s(gid, "set xrange [-30000:30000]\n");
	//fprintf_s(gid, "set yrange [-30000:30000]\n");
	//fprintf_s(gid, "set zrange [-10000:10000]\n");

	//fprintf_s(gid, "set zrange [-2.5:2.5]\n"); //for WAA010
	fprintf_s(gid, "set xlabel 'camera-x'\n");
	fprintf_s(gid, "set ylabel 'camera-z'\n");
	fprintf_s(gid, "set zlabel 'camera-y'\n");
	//fprintf_s(gid, "set ylabel 'waa010-y'\n");
	//fprintf_s(gid, "set zlabel 'waa010-z'\n");
	//fprintf_s(gid, "set xyplane 0\n");
	//fprintf_s(gid, "set size ratio -1\n");
	fprintf_s(gid, "set view equal xyz\n");
	fprintf_s(gid, "set view 360,0\n");
	fflush(gid);




	while (1) {







		//fprintf_s(gid, "splot '-' w vectors t 'camera-X', '-' w vectors t 'camera-Z', '-' w vectors t 'camera-Y', '-' w vectors t 'waa010-X', '-' w vectors t 'waa010-Z','-' w vectors t 'waa010-Y'\n");



		fprintf_s(gid, "splot '-' w vectors lw 10 t 'camera-X', '-' w vectors lw 10 t 'camera-Z', '-' w vectors lw 10 t 'camera-Y'\n");
		//fprintf_s(gid, "splot '-' w p ps 5 t 'camera-X', '-' w vectors t 'camera-Z', '-' w vectors t 'camera-Y'\n");

		/*
		fprintf_s(gid, "0\t0\t0\t%lf\t%lf\t%lf\n", axis_1n.at<double>(0, 0), axis_1n.at<double>(2, 0), axis_1n.at<double>(1, 0));
		fprintf_s(gid, "e\n");
		fprintf_s(gid, "0\t0\t0\t%lf\t%lf\t%lf\n", axis_3n.at<double>(0, 0), axis_3n.at<double>(2, 0), axis_3n.at<double>(1, 0));
		fprintf_s(gid, "e\n");
		fprintf_s(gid, "0\t0\t0\t%lf\t%lf\t%lf\n", axis_2n.at<double>(0, 0), axis_2n.at<double>(2, 0), axis_2n.at<double>(1, 0));
		fprintf_s(gid, "e\n");
		*/



		double delta_x[3], delta_y[3], delta_z[3];

		delta_x[0] = axis_dim4_1n.at<double>(0, 0) - axis_dim4_origin_n.at<double>(0, 0);
		delta_y[0] = axis_dim4_1n.at<double>(2, 0) - axis_dim4_origin_n.at<double>(2, 0);
		delta_z[0] = axis_dim4_1n.at<double>(1, 0) - axis_dim4_origin_n.at<double>(1, 0);
		delta_x[1] = axis_dim4_2n.at<double>(0, 0) - axis_dim4_origin_n.at<double>(0, 0);
		delta_y[1] = axis_dim4_2n.at<double>(2, 0) - axis_dim4_origin_n.at<double>(2, 0);
		delta_z[1] = axis_dim4_2n.at<double>(1, 0) - axis_dim4_origin_n.at<double>(1, 0);
		delta_x[2] = axis_dim4_3n.at<double>(0, 0) - axis_dim4_origin_n.at<double>(0, 0);
		delta_y[2] = axis_dim4_3n.at<double>(2, 0) - axis_dim4_origin_n.at<double>(2, 0);
		delta_z[2] = axis_dim4_3n.at<double>(1, 0) - axis_dim4_origin_n.at<double>(1, 0);

		fprintf_s(gid, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", axis_dim4_origin_n.at<double>(0, 0), axis_dim4_origin_n.at<double>(2, 0), axis_dim4_origin_n.at<double>(1, 0), delta_x[0], delta_z[0], delta_y[0]);
		fprintf_s(gid, "e\n");
		fprintf_s(gid, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", axis_dim4_origin_n.at<double>(0, 0), axis_dim4_origin_n.at<double>(2, 0), axis_dim4_origin_n.at<double>(1, 0), delta_x[2], delta_z[2], delta_y[2]);
		fprintf_s(gid, "e\n");
		fprintf_s(gid, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", axis_dim4_origin_n.at<double>(0, 0), axis_dim4_origin_n.at<double>(2, 0), axis_dim4_origin_n.at<double>(1, 0), delta_x[1], delta_z[1], delta_y[1]);
		fprintf_s(gid, "e\n");


		//fprintf_s(gid, "%lf\t%lf\t%lf\n", axis_dim4_origin_n.at<double>(0, 0), axis_dim4_origin_n.at<double>(2, 0), axis_dim4_origin_n.at<double>(1, 0));
		//fprintf_s(gid, "e\n");
		//fprintf_s(gid, "%lf\t%lf\t%lf\n", axis_dim4_origin_n.at<double>(0, 0), axis_dim4_origin_n.at<double>(2, 0), axis_dim4_origin_n.at<double>(1, 0));
		//fprintf_s(gid, "e\n");
		//fprintf_s(gid, "%lf\t%lf\t%lf\n", axis_dim4_origin_n.at<double>(0, 0), axis_dim4_origin_n.at<double>(2, 0), axis_dim4_origin_n.at<double>(1, 0));
		//fprintf_s(gid, "e\n");



		fflush(gid);


		if (n_flag_thread_gnuplot_exit == 1) break;



		Sleep(10);

	}


	fclose(gid);



	_endthreadex(0);



	return 0;


}




/*
unsigned _stdcall thread_save_XYZRGB(void* p)
{

Mat buf_xyz;
Mat buf_remap;

buf_xyz = xyz.clone();
buf_remap = img_cam1_remap.clone();




saveXYZRGB(point_cloud_filename.c_str(), buf_xyz, buf_remap);


n_frame_count_3D_data++;

return n_frame_count_3D_data;


}
*/

int thread_save_XYZRGB(const char* filename)
{

	Mat buf_xyz;
	Mat buf_remap;
	Mat buf_mask;

	buf_xyz = xyz.clone();
	buf_remap = img_cam1_remap.clone();
	buf_mask = mask_with_xyz.clone();




	saveXYZRGB(filename, buf_xyz, buf_remap, buf_mask);


	n_frame_count_3D_data++;

	return n_frame_count_3D_data;


}



static void saveXYZRGB(const char* filename, const Mat& mat, const Mat& color, const Mat& mask)
{

	cv::Vec3b		intensity;





	FILE* fp;
	fopen_s(&fp, filename, "wt");

	for (int y = 0; y < mat.rows; y++)

	{

		for (int x = 0; x < mat.cols; x++)

		{

			intensity = color.at<cv::Vec3b>(y, x);




			Vec3f point = mat.at<Vec3f>(y, x);



			if (mask.at<uchar>(y, x) != 1) continue;







			fprintf_s(fp, "%d\t%d\t%f\t%f\t%f\t%d\t%d\t%d\n", x,y, point[0], point[1], point[2], intensity[2], intensity[1], intensity[0]);


		}

	}

	fclose(fp);

}


void Erase_xyz_outliers_on_non_texture_region(Mat& xyz_Mat, Mat& mask_xyz, Mat& mask_non_texture_image, int n_wnd_width, int n_wnd_height, double d_thresh_depth_to_detect_outlier)
{


	int i, j;
	int p, q;

	int n_pixel_count;
	double d_max_depth, d_min_depth;
	double delta_depth;

	cv::Vec3f p0, p1;



	for (j = 0; j < IMG_YSIZE - n_wnd_height - 1; j += n_wnd_height / 2) {
		for (i = 0; i < IMG_XSIZE - n_wnd_width - 1; i += n_wnd_width / 2) {





			n_pixel_count = 0;
			d_max_depth = -1;
			d_min_depth = 1000000;


			for (q = j; q < j + n_wnd_height; q++) {
				for (p = i; p < i + n_wnd_width; p++) {

					if (mask_non_texture_image.at<uchar>(q, p) == 255 && mask_xyz.at<uchar>(q, p) == 1) {

						p0 = xyz_Mat.at<Vec3f>(q, p);


						if (d_max_depth < p0[2]) d_max_depth = p0[2];
						if (d_min_depth > p0[2]) d_min_depth = p0[2];

						n_pixel_count++;

					}


				}
			}



			delta_depth = d_max_depth - d_min_depth;





			if (n_pixel_count >= 2 && delta_depth > d_thresh_depth_to_detect_outlier) {


				for (q = j; q < j + n_wnd_height; q++) {
					for (p = i; p < i + n_wnd_width; p++) {


						if (mask_non_texture_image.at<uchar>(q, p) == 255) {


							mask_xyz.at<uchar>(q, p) = 0;



						}



					}
				}



			}//if






		}
	}



}


void Make_ground_candidated_mask_image_using_3D_surface_patch(int n_mesh_width, int n_mesh_height, const Mat& xyz_mat, const Mat& mask, double d_targeted_angle_to_detect_wall, double d_targeted_angle_tolerance, Mat& mask_ground_candidated)
{



	cv::Vec3f p0, p1, p2;
	cv::Vec3f vec_1, vec_2;
	cv::Vec3f patch_normal;

	cv::Vec3f vec_axis_x = cv::Vec3f(1, 0, 0);
	cv::Vec3f vec_axis_y = cv::Vec3f(0, 1, 0);
	cv::Vec3f vec_axis_z = cv::Vec3f(0, 0, 1);



	double d_dot_product_x_axis;
	double d_dot_product_y_axis;
	double d_dot_product_z_axis;

	double d_angle_nomal_to_x_axis;
	double d_angle_nomal_to_y_axis;
	double d_angle_nomal_to_z_axis;



	// (x2,y2)p2 :vec_2
	// |
	// (x, y)p0 -- (x1, y1)p1  : vec_1


	// (x2,y2)p0 -- (x3, y3)p2  vec_2
	// |
	// (x, y)p1 ; vec_1


	int n_x1, n_y1;
	int n_x2, n_y2;
	int n_x3, n_y3;


	int i, j;
	int p, q;





	mask_ground_candidated = cv::Scalar(0);




	for (j = n_mesh_height; j < IMG_YSIZE; j += n_mesh_height / 2) {
		for (i = 0; i < IMG_XSIZE - n_mesh_width; i += n_mesh_width / 2) {



			n_x1 = i + n_mesh_width;
			n_y1 = j;
			n_x2 = i;
			n_y2 = j - n_mesh_height;
			n_x3 = i + n_mesh_width;
			n_y3 = j - n_mesh_height;


			if (n_x1 < IMG_XSIZE && n_y2 >= 0 && mask.at<uchar>(n_y1, n_x1) == 1 && mask.at<uchar>(n_y2, n_x2) == 1) {



				p0 = xyz_mat.at<Vec3f>(j, i);
				p1 = xyz_mat.at<Vec3f>(n_y1, n_x1);
				p2 = xyz_mat.at<Vec3f>(n_y2, n_x2);



				vec_1 = p1 - p0;
				vec_2 = p2 - p0;

				patch_normal = vec_1.cross(vec_2);
				patch_normal = patch_normal / norm(patch_normal);


				//calc angle to x axis
				d_dot_product_x_axis = patch_normal.dot(vec_axis_x);
				d_dot_product_y_axis = patch_normal.dot(vec_axis_y);
				d_dot_product_z_axis = patch_normal.dot(vec_axis_z);

				d_angle_nomal_to_x_axis = acos(d_dot_product_x_axis) / PI*180.0;
				d_angle_nomal_to_y_axis = acos(d_dot_product_y_axis) / PI*180.0;
				d_angle_nomal_to_z_axis = acos(d_dot_product_z_axis) / PI*180.0;





				if (fabs(d_angle_nomal_to_y_axis - d_targeted_angle_to_detect_wall) < d_targeted_angle_tolerance) {


					for (q = n_y2; q <= j; q++) {
						for (p = i; p <= n_x3; p++) {

							mask_ground_candidated.at<uchar>(q, p) = 255;

						}
					}


				}//if






			}// i
		}// j



	}





}






int check_step_region_pixel_using_3D_surface_patch(int x, int y, int n_mesh_width, int n_mesh_height, const Mat& xyz_mat, const Mat& mask, cv::Vec3f vec_ground_normal, cv::Vec3f vec_x_axis, double d_targeted_angle_to_detect_step, double d_targeted_angle_tolerance, double d_targeted_angle_tolerance_to_x_axis)
{





	cv::Vec3f p0, p1, p2;
	cv::Vec3f vec_1, vec_2;
	cv::Vec3f patch_normal;


	double d_dot_product_normal_vectors;
	double d_angle_between_normal_vectors;

	double d_dot_product_normal_vector_x;
	double d_angle_between_normal_vector_x;

	int n_flag_step_angle, n_flag_step_angle2;


	// (x2,y2)p2 :vec_2
	// |
	// (x, y)p0 -- (x1, y1)p1  : vec_1


	// (x2,y2)p0 -- (x3, y3)p2  vec_2
	// |
	// (x, y)p1 ; vec_1


	int n_x1, n_y1;
	int n_x2, n_y2;
	int n_x3, n_y3;



	n_x1 = x + n_mesh_width;
	n_y1 = y;
	n_x2 = x;
	n_y2 = y - n_mesh_height;
	n_x3 = x + n_mesh_width;
	n_y3 = y - n_mesh_height;






	n_flag_step_angle = n_flag_step_angle2 = 0;



	if (n_x1 < IMG_XSIZE && n_y2 >= 0 && mask.at<uchar>(n_y1, n_x1) == 1 && mask.at<uchar>(n_y2, n_x2) == 1) {



		p0 = xyz_mat.at<Vec3f>(y, x);
		p1 = xyz_mat.at<Vec3f>(n_y1, n_x1);
		p2 = xyz_mat.at<Vec3f>(n_y2, n_x2);



		vec_1 = p1 - p0;
		vec_2 = p2 - p0;

		patch_normal = vec_1.cross(vec_2);
		patch_normal = patch_normal / norm(patch_normal);


		//calc angle
		d_dot_product_normal_vectors = patch_normal.dot(vec_ground_normal);
		d_dot_product_normal_vector_x = patch_normal.dot(vec_x_axis);



		d_angle_between_normal_vectors = acos(d_dot_product_normal_vectors) / PI*180.0;
		d_angle_between_normal_vector_x = acos(d_dot_product_normal_vector_x) / PI*180.0;


		if (fabs(d_angle_between_normal_vectors - d_targeted_angle_to_detect_step) < d_targeted_angle_tolerance && fabs(d_angle_between_normal_vector_x - d_targeted_angle_to_detect_step) < d_targeted_angle_tolerance_to_x_axis) {

			n_flag_step_angle = 1;

		}



	}





	if (n_x3 < IMG_XSIZE &&  n_y3 >= 0 && mask.at<uchar>(n_y2, n_x2) == 1 && mask.at<uchar>(n_y3, n_x3) == 1) {


		p0 = xyz_mat.at<Vec3f>(n_y2, n_x2);
		p1 = xyz_mat.at<Vec3f>(y, x);
		p2 = xyz_mat.at<Vec3f>(n_y3, n_x3);



		vec_1 = p1 - p0;
		vec_2 = p2 - p0;

		patch_normal = vec_1.cross(vec_2);
		patch_normal = patch_normal / norm(patch_normal);


		//calc angle
		d_dot_product_normal_vectors = patch_normal.dot(vec_ground_normal);
		d_dot_product_normal_vector_x = patch_normal.dot(vec_x_axis);


		d_angle_between_normal_vectors = acos(d_dot_product_normal_vectors) / PI*180.0;
		d_angle_between_normal_vector_x = acos(d_dot_product_normal_vector_x) / PI*180.0;


		if (fabs(d_angle_between_normal_vectors - d_targeted_angle_to_detect_step) < d_targeted_angle_tolerance && fabs(d_angle_between_normal_vector_x - d_targeted_angle_to_detect_step) < d_targeted_angle_tolerance_to_x_axis) {

			n_flag_step_angle2 = 1;

		}


	}





	if (n_flag_step_angle == 1 || n_flag_step_angle2 == 1) return 1;
	else return -1;





}



void ICP_particle_filter_init_particles(const Mat& R_mat, const Mat& T_mat, int n_particles_num, std::vector<cv::Vec3d> &particles_T, std::vector<double> &particles_Weight, std::vector<double> &particles_Weight_accumulated, std::vector<cv::Vec3d> &particles_T_next, std::vector<double> &particles_Weight_next, std::vector<double> &particles_Weight_accumulated_next)
{

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<double>generate_rand_Translate(0, 200); //std::uniform_real_distribution<double>generate_rand_Translate(-1.0, 1.0);
	std::uniform_real_distribution<double>rand_0_to_1(0.0, 1.0);

	int k;


	cv::Vec3d vec_TxTyTz;
	cv::Vec3f targeted_xyz;

	double d_Tx, d_Ty, d_Tz;
	double d_scale_factor_alpha;


	//generate particles



	//particle filter paramters



	double d_likelihood;
	double d_Sum_particle_weigtht;





	d_Sum_particle_weigtht = 0;
	d_likelihood = 1.0 / n_particles_num;

	for (k = 0; k < n_particles_num; k++) {




		d_scale_factor_alpha = generate_rand_Translate(mt);

		d_Tx = d_scale_factor_alpha*T_mat.at<double>(0);
		d_Ty = d_scale_factor_alpha*T_mat.at<double>(1);
		d_Tz = d_scale_factor_alpha*T_mat.at<double>(2);

		vec_TxTyTz[0] = d_Tx;
		vec_TxTyTz[1] = d_Ty;
		vec_TxTyTz[2] = d_Tz;


		//vec_TxTyTz[0] = generate_rand_Translate(mt) * 500;
		//vec_TxTyTz[1] = generate_rand_Translate(mt) * 500;
		//vec_TxTyTz[2] = generate_rand_Translate(mt) * 500;






		d_Sum_particle_weigtht += d_likelihood;






		particles_T.push_back(vec_TxTyTz);
		particles_T_next.push_back(vec_TxTyTz);

		particles_Weight.push_back(d_likelihood);
		particles_Weight_next.push_back(d_likelihood);

		particles_Weight_accumulated.push_back(d_Sum_particle_weigtht);
		particles_Weight_accumulated_next.push_back(d_Sum_particle_weigtht);






	}







}


void ICP_particle_filter_resampling(int n_particles_num, std::vector<cv::Vec3d> &particles_T, std::vector<double> &particles_Weight, std::vector<double> &particles_Weight_accumulated, std::vector<cv::Vec3d> &particles_T_next, std::vector<double> &particles_Weight_next, std::vector<double> &particles_Weight_accumulated_next)
{

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<double>rand_0_to_1(0.0, 1.0);


	int k;
	int t;




	// resampling


	int n_picked_up_particle_No;
	double d_picked_up_point_in_weight_array;

	double d_Sum_weight_picked_up_point;





	d_Sum_weight_picked_up_point = 0;



	for (k = 0; k < n_particles_num; k++) {




		d_picked_up_point_in_weight_array = rand_0_to_1(mt);


		for (t = 0; t < n_particles_num; t++) {

			if (particles_Weight_accumulated[t] > d_picked_up_point_in_weight_array) {

				n_picked_up_particle_No = t;
				break;
			}

		}


		d_Sum_weight_picked_up_point += particles_Weight[k];



		particles_T_next[k] = particles_T[k];
		particles_Weight_next[k] = particles_Weight[k];

		particles_Weight_accumulated_next[k] = d_Sum_weight_picked_up_point;






	}





	// normalized weight //

	for (k = 0; k < n_particles_num; k++) {

		particles_Weight_next[k] /= d_Sum_weight_picked_up_point;

	}







}



void ICP_particle_filter_predict(int n_particles_num, std::vector<cv::Vec3d> &particles_T, double d_rand_normal_dist_stdev)
{

	int k;

	std::random_device rd;
	std::mt19937 mt(rd());
	std::normal_distribution<double>normal_rand(0, d_rand_normal_dist_stdev);


	cv::Vec3d vec_TxTyTz;


	for (k = 0; k < n_particles_num; k++) {




		vec_TxTyTz = particles_T[k];

		vec_TxTyTz[0] += normal_rand(mt);
		vec_TxTyTz[1] += normal_rand(mt);
		vec_TxTyTz[2] += normal_rand(mt);

		particles_T[k] = vec_TxTyTz;


	}


}

void ICP_particle_filter_measure_calc_likelihood(int n_particles_num, std::vector<cv::Vec3d> &particles_T, std::vector<double> &particles_Weight, std::vector<double> &particles_Weight_accumulated, const Mat& R_mat, const Mat& T_mat, std::vector<cv::Vec3f>& mesh_point0, std::vector<cv::Vec3f>& mesh_normal0, std::vector<cv::Vec3f>& mesh_point1, cv::Vec3d &estimated_TxTyTz)
{

	int i, j;
	int k;

	cv::Vec3d vec_TxTyTz;
	cv::Vec3f targeted_xyz;
	cv::Vec3f targeted_xyz0;

	//cv::Vec3f targeted_mesh_point;


	double d_Tx, d_Ty, d_Tz;
	double d_Error;
	double d_Error_min;
	double d_likelihood;
	double d_Energy;
	double d_Sum_weight;

	int n_point_num_to_estimate_T;





	cv::Vec3d sum_TxTyTz;





	Mat R4x4_mat = Mat::eye(4, 4, CV_64FC1);

	R4x4_mat.at<double>(0, 0) = R_mat.at<double>(0, 0);		R4x4_mat.at<double>(0, 1) = R_mat.at<double>(0, 1);		R4x4_mat.at<double>(0, 2) = R_mat.at<double>(0, 2);		R4x4_mat.at<double>(0, 3) = 0;// T_cam.at<double>(0);
	R4x4_mat.at<double>(1, 0) = R_mat.at<double>(1, 0);		R4x4_mat.at<double>(1, 1) = R_mat.at<double>(1, 1);		R4x4_mat.at<double>(1, 2) = R_mat.at<double>(1, 2);		R4x4_mat.at<double>(1, 3) = 0;// T_cam.at<double>(1);
	R4x4_mat.at<double>(2, 0) = R_mat.at<double>(2, 0);		R4x4_mat.at<double>(2, 1) = R_mat.at<double>(2, 1);		R4x4_mat.at<double>(2, 2) = R_mat.at<double>(2, 2);		R4x4_mat.at<double>(2, 3) = 0;// T_cam.at<double>(2);





	d_Sum_weight = 0;

	for (k = 0; k < n_particles_num; k++) {






		Mat TR_mat = Mat::eye(4, 4, CV_64FC1);
		Mat TR_inv_mat = Mat::eye(4, 4, CV_64FC1);

		Mat T4x4_mat = Mat::eye(4, 4, CV_64FC1);

		Mat xyz_4x1_mat(4, 1, CV_64FC1);
		Mat xyz_0_4x1_mat(4, 1, CV_64FC1);




		vec_TxTyTz = particles_T[k];






		d_Tx = vec_TxTyTz[0];
		d_Ty = vec_TxTyTz[1];
		d_Tz = vec_TxTyTz[2];





		T4x4_mat.at<double>(0, 3) = d_Tx;
		T4x4_mat.at<double>(1, 3) = d_Ty;
		T4x4_mat.at<double>(2, 3) = d_Tz;





		TR_mat = T4x4_mat * R4x4_mat;
		TR_inv_mat = TR_mat.inv();







		d_Energy = 0;

		n_point_num_to_estimate_T = 0;





		for (j = 0; j < (int)mesh_point1.size(); j++) {






			targeted_xyz = mesh_point1[j];




			xyz_4x1_mat.at<float>(0, 0) = targeted_xyz[0];
			xyz_4x1_mat.at<float>(1, 0) = targeted_xyz[1];
			xyz_4x1_mat.at<float>(2, 0) = targeted_xyz[2];
			xyz_4x1_mat.at<float>(3, 0) = 1;






			xyz_0_4x1_mat = TR_inv_mat*xyz_4x1_mat;






			targeted_xyz0[0] = xyz_0_4x1_mat.at<float>(0, 0);
			targeted_xyz0[1] = xyz_0_4x1_mat.at<float>(1, 0);
			targeted_xyz0[2] = xyz_0_4x1_mat.at<float>(2, 0);









			//search a mathed point

			//d_Error_min = 1000000;



			for (i = 0; i < (int)mesh_point0.size(); i++) {

				d_Error = (targeted_xyz0 - mesh_point0[i]).dot(mesh_normal0[i]);


				d_Error = fabs(d_Error);


				if (i == 0) {
					d_Error_min = d_Error;
					continue;
				}

				if (d_Error_min > d_Error) {

					d_Error_min = d_Error;
				}


			}//loop mesh point 0







			 //if (d_Error_min > ESTIMATE_T_MATRIX_OUTLIER_DISTANCE_THRESHOLD) continue;



			d_Energy += d_Error_min;

			n_point_num_to_estimate_T++;


		}//matching points







		 //if (n_point_num_to_estimate_T == 0) continue;


		d_Energy = d_Energy / (double)n_point_num_to_estimate_T;



		d_likelihood = 1.0 / d_Energy;









		// set a particle //


		particles_T[k] = vec_TxTyTz;
		particles_Weight[k] = d_likelihood;






	}




	//estimate the TxTyTz


	sum_TxTyTz[0] = sum_TxTyTz[1] = sum_TxTyTz[2] = 0;

	d_Sum_weight = 0;

	for (k = 0; k < n_particles_num; k++) {

		sum_TxTyTz += particles_T[k] * particles_Weight[k];

		d_Sum_weight += particles_Weight[k];


	}



	estimated_TxTyTz = sum_TxTyTz / d_Sum_weight;





}


void ICP_particle_filter_update(int n_particles_num, std::vector<cv::Vec3d> &particles_T_next, std::vector<double> &particles_Weight_next, std::vector<double> &particles_Weight_accumulated_next, std::vector<cv::Vec3d> &particles_T, std::vector<double> &particles_Weight, std::vector<double> &particles_Weight_accumulated)
{
	int k;


	for (k = 0; k < n_particles_num; k++) {

		particles_T[k] = particles_T_next[k];
		particles_Weight[k] = particles_Weight_next[k];

		particles_Weight_accumulated[k] = particles_Weight_accumulated_next[k];
	}


}


void Iterative_Closest_Point_ICP_to_estimate_T_matrix_using_particle_filter(int n_mesh_size, const Mat& xyz_mat0, const Mat& xyz_mat1, const Mat& mask0, const Mat& mask1, const Mat& cloud_mask, const Mat& R_mat, const Mat& T_mat, double d_angle_thresh_to_detect_mesh_point, std::vector<cv::Point> &mesh_img_points0, std::vector<cv::Vec3f>& mesh_point0, std::vector<cv::Vec3f>& mesh_normal0, std::vector<cv::Vec3f>& mesh_point1, double *d_Tx_estimated, double *d_Ty_estimated, double *d_Tz_estimated)
{

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<double>generate_rand_Translate(-1.0, 1.0);
	std::uniform_real_distribution<double>rand_0_to_1(0.0, 1.0);


	int i, j;
	int k;

	//std::vector<cv::Vec3f> mesh_point;
	//std::vector<cv::Vec3f> mesh_normal;
	cv::Vec3f targeted_mesh_point;
	cv::Point2i targeted_mesh_img_point;
	cv::Vec3f p0, p1, p2;
	cv::Vec3f vec_1, vec_2;
	cv::Vec3f patch_normal0;
	cv::Vec3f patch_normal1;

	double d_dot_product_normal_vectors;
	double d_angle_between_normal_vectors;
















	//camera0
	for (j = 0; j < IMG_YSIZE - n_mesh_size * 2; j += n_mesh_size / 2) {
		for (i = 0; i < IMG_XSIZE - n_mesh_size * 2; i += n_mesh_size / 2) {


			if (mask0.at<uchar>(j, i) == 1 && mask0.at<uchar>(j, i + n_mesh_size) == 1 && mask0.at<uchar>(j + n_mesh_size, i) == 1 && mask0.at<uchar>(j, i + n_mesh_size * 2) == 1 && mask0.at<uchar>(j + n_mesh_size * 2, i) == 1 && cloud_mask.at<uchar>(j, i) == 0) {


				targeted_mesh_point = xyz_mat0.at<Vec3f>(j, i);
				targeted_mesh_img_point.x = i;
				targeted_mesh_img_point.y = j;



				//normal0
				p0 = xyz_mat0.at<Vec3f>(j, i);
				p1 = xyz_mat0.at<Vec3f>(j, i + n_mesh_size);
				p2 = xyz_mat0.at<Vec3f>(j + n_mesh_size, i);

				vec_1 = p1 - p0;
				vec_2 = p2 - p0;

				patch_normal0 = vec_2.cross(vec_1);
				patch_normal0 = patch_normal0 / norm(patch_normal0);



				//normal1
				p0 = xyz_mat0.at<Vec3f>(j, i + n_mesh_size);
				p1 = xyz_mat0.at<Vec3f>(j, i + n_mesh_size * 2);
				p2 = xyz_mat0.at<Vec3f>(j + n_mesh_size * 2, i);

				vec_1 = p1 - p0;
				vec_2 = p2 - p0;

				patch_normal1 = vec_2.cross(vec_1);
				patch_normal1 = patch_normal1 / norm(patch_normal1);



				//calc angle
				d_dot_product_normal_vectors = patch_normal0.dot(patch_normal1);


				d_angle_between_normal_vectors = acos(d_dot_product_normal_vectors) / PI*180.0;





				if (d_angle_between_normal_vectors > d_angle_thresh_to_detect_mesh_point) {


					mesh_point0.push_back(targeted_mesh_point);
					mesh_img_points0.push_back(targeted_mesh_img_point);
					mesh_normal0.push_back(patch_normal0);


				}



			}


		}
	}




	//camera1
	for (j = 0; j < IMG_YSIZE - n_mesh_size; j += 10) {
		for (i = 0; i < IMG_XSIZE - n_mesh_size; i += 10) {



			if (mask1.at<uchar>(j, i) == 1) {

				targeted_mesh_point = xyz_mat1.at<Vec3f>(j, i);

				mesh_point1.push_back(targeted_mesh_point);



			}
		}
	}











	//particle filters

	int n_particles_num = 1000;

	std::vector<cv::Vec3d> particles_T;
	std::vector<double> particles_Weight;
	std::vector<double> particles_Weight_accumulated;
	std::vector<cv::Vec3d> particles_T_next;
	std::vector<double> particles_Weight_next;
	std::vector<double> particles_Weight_accumulated_next;


	cv::Vec3d estimated_TxTyTz;

	int n_loop_times = 5;







	//init particles


	ICP_particle_filter_init_particles(R_mat, T_mat, n_particles_num, particles_T, particles_Weight, particles_Weight_accumulated, particles_T_next, particles_Weight_next, particles_Weight_accumulated_next);





	for (k = 0; k < n_loop_times; k++) {




		ICP_particle_filter_resampling(n_particles_num, particles_T, particles_Weight, particles_Weight_accumulated, particles_T_next, particles_Weight_next, particles_Weight_accumulated_next);

		ICP_particle_filter_update(n_particles_num, particles_T_next, particles_Weight_next, particles_Weight_accumulated_next, particles_T, particles_Weight, particles_Weight_accumulated);

		ICP_particle_filter_predict(n_particles_num, particles_T, 50);

		ICP_particle_filter_measure_calc_likelihood(n_particles_num, particles_T, particles_Weight, particles_Weight_accumulated, R_mat, T_mat, mesh_point0, mesh_normal0, mesh_point1, estimated_TxTyTz);

		*d_Tx_estimated = estimated_TxTyTz[0];
		*d_Ty_estimated = estimated_TxTyTz[1];
		*d_Tz_estimated = estimated_TxTyTz[2];


	}













}




void Iterative_Closest_Point_ICP_to_estimate_T_matrix(int n_mesh_size, const Mat& xyz_mat0, const Mat& xyz_mat1, const Mat& mask0, const Mat& mask1, const Mat& cloud_mask, const Mat& R_mat, const Mat& T_mat, double d_angle_thresh_to_detect_mesh_point, std::vector<cv::Point> &mesh_img_points0, std::vector<cv::Vec3f>& mesh_point0, std::vector<cv::Vec3f>& mesh_normal0, std::vector<cv::Vec3f>& mesh_point1, double *d_Tx_estimated, double *d_Ty_estimated, double *d_Tz_estimated)
{

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<double>generate_rand_Translate(0, 1.0);

	int i, j;

	//std::vector<cv::Vec3f> mesh_point;
	//std::vector<cv::Vec3f> mesh_normal;
	cv::Vec3f targeted_mesh_point;
	cv::Point2i targeted_mesh_img_point;
	cv::Vec3f p0, p1, p2;
	cv::Vec3f vec_1, vec_2;
	cv::Vec3f patch_normal0;
	cv::Vec3f patch_normal1;

	double d_dot_product_normal_vectors;
	double d_angle_between_normal_vectors;


	//camera0
	for (j = 0; j < IMG_YSIZE - n_mesh_size * 2; j += n_mesh_size / 2) {
		for (i = 0; i < IMG_XSIZE - n_mesh_size * 2; i += n_mesh_size / 2) {


			if (mask0.at<uchar>(j, i) == 1 && mask0.at<uchar>(j, i + n_mesh_size) == 1 && mask0.at<uchar>(j + n_mesh_size, i) == 1 && mask0.at<uchar>(j, i + n_mesh_size * 2) == 1 && mask0.at<uchar>(j + n_mesh_size * 2, i) == 1 && cloud_mask.at<uchar>(j, i) == 0) {


				targeted_mesh_point = xyz_mat0.at<Vec3f>(j, i);
				targeted_mesh_img_point.x = i;
				targeted_mesh_img_point.y = j;



				//normal0
				p0 = xyz_mat0.at<Vec3f>(j, i);
				p1 = xyz_mat0.at<Vec3f>(j, i + n_mesh_size);
				p2 = xyz_mat0.at<Vec3f>(j + n_mesh_size, i);

				vec_1 = p1 - p0;
				vec_2 = p2 - p0;

				patch_normal0 = vec_2.cross(vec_1);
				patch_normal0 = patch_normal0 / norm(patch_normal0);



				//normal1
				p0 = xyz_mat0.at<Vec3f>(j, i + n_mesh_size);
				p1 = xyz_mat0.at<Vec3f>(j, i + n_mesh_size * 2);
				p2 = xyz_mat0.at<Vec3f>(j + n_mesh_size * 2, i);

				vec_1 = p1 - p0;
				vec_2 = p2 - p0;

				patch_normal1 = vec_2.cross(vec_1);
				patch_normal1 = patch_normal1 / norm(patch_normal1);



				//calc angle
				d_dot_product_normal_vectors = patch_normal0.dot(patch_normal1);


				d_angle_between_normal_vectors = acos(d_dot_product_normal_vectors) / PI*180.0;





				if (d_angle_between_normal_vectors > d_angle_thresh_to_detect_mesh_point) {


					mesh_point0.push_back(targeted_mesh_point);
					mesh_img_points0.push_back(targeted_mesh_img_point);
					mesh_normal0.push_back(patch_normal0);


				}



			}


		}
	}




	//camera1
	for (j = 0; j < IMG_YSIZE - n_mesh_size; j += 5) {
		for (i = 0; i < IMG_XSIZE - n_mesh_size; i += 10) {



			if (mask1.at<uchar>(j, i) == 1) {

				targeted_mesh_point = xyz_mat1.at<Vec3f>(j, i);

				mesh_point1.push_back(targeted_mesh_point);



			}
		}
	}







	int k;
	//int k_x, k_y, k_z;
	int n_step_num = 15;
	double d_step_distance = 50;

	double d_Tx, d_Ty, d_Tz;
	cv::Vec3f targeted_xyz;
	cv::Vec3f targeted_xyz0;







	int n_point_num_to_estimate_T;


	double d_Error_min;
	double d_Error;
	double d_Energy;
	double d_Energy_min;

	int n_flag_first_processing = -1;;





	Mat R4x4_mat = Mat::eye(4, 4, CV_64FC1);

	R4x4_mat.at<double>(0, 0) = R_mat.at<double>(0, 0);		R4x4_mat.at<double>(0, 1) = R_mat.at<double>(0, 1);		R4x4_mat.at<double>(0, 2) = R_mat.at<double>(0, 2);		R4x4_mat.at<double>(0, 3) = 0;// T_cam.at<double>(0);
	R4x4_mat.at<double>(1, 0) = R_mat.at<double>(1, 0);		R4x4_mat.at<double>(1, 1) = R_mat.at<double>(1, 1);		R4x4_mat.at<double>(1, 2) = R_mat.at<double>(1, 2);		R4x4_mat.at<double>(1, 3) = 0;// T_cam.at<double>(1);
	R4x4_mat.at<double>(2, 0) = R_mat.at<double>(2, 0);		R4x4_mat.at<double>(2, 1) = R_mat.at<double>(2, 1);		R4x4_mat.at<double>(2, 2) = R_mat.at<double>(2, 2);		R4x4_mat.at<double>(2, 3) = 0;// T_cam.at<double>(2);
















																																																		  //for (k_z = -n_step_num/2; k_z <= n_step_num/2; k_z++){

																																																		  //for (k_y = -n_step_num/3; k_y <= n_step_num/3; k_y++){

																																																		  //for (k_x = -n_step_num/2; k_x <= n_step_num/2; k_x++){



																																																		  //for (k = 0; k < n_step_num; k++){
	for (k = 0; k < 1000; k++) {





		Mat TR_mat = Mat::eye(4, 4, CV_64FC1);
		Mat TR_inv_mat = Mat::eye(4, 4, CV_64FC1);

		Mat T4x4_mat = Mat::eye(4, 4, CV_64FC1);

		Mat xyz_4x1_mat(4, 1, CV_64FC1);
		Mat xyz_0_4x1_mat(4, 1, CV_64FC1);





		double d_scale_factor_alpha;



		//d_scale_factor_alpha = k * d_step_distance;


		//d_Tx = generate_rand_Translate(mt) * 300;
		//d_Ty = generate_rand_Translate(mt) * 300;
		//d_Tz = generate_rand_Translate(mt) * 300;



		d_scale_factor_alpha = generate_rand_Translate(mt) * 500;









		//cout << d_Tx << ", " << d_Ty << ", " << d_Tz << endl;


		d_Tx = d_scale_factor_alpha*T_mat.at<double>(0);
		d_Ty = d_scale_factor_alpha*T_mat.at<double>(1);
		d_Tz = d_scale_factor_alpha*T_mat.at<double>(2);

		//d_Tx = k_x * d_step_distance;
		//d_Ty = k_y * d_step_distance;
		//d_Tz = k_z * d_step_distance;





		T4x4_mat.at<double>(0, 3) = d_Tx;
		T4x4_mat.at<double>(1, 3) = d_Ty;
		T4x4_mat.at<double>(2, 3) = d_Tz;





		TR_mat = T4x4_mat * R4x4_mat;
		TR_inv_mat = TR_mat.inv();



		d_Energy = 0;

		n_point_num_to_estimate_T = 0;





		for (j = 0; j < (int)mesh_point1.size(); j++) {






			targeted_xyz = mesh_point1[j];




			xyz_4x1_mat.at<float>(0, 0) = targeted_xyz[0];
			xyz_4x1_mat.at<float>(1, 0) = targeted_xyz[1];
			xyz_4x1_mat.at<float>(2, 0) = targeted_xyz[2];
			xyz_4x1_mat.at<float>(3, 0) = 1;






			xyz_0_4x1_mat = TR_inv_mat*xyz_4x1_mat;
			//xyz_0_4x1_mat = TR_mat*xyz_4x1_mat;





			targeted_xyz0[0] = xyz_0_4x1_mat.at<float>(0, 0);
			targeted_xyz0[1] = xyz_0_4x1_mat.at<float>(1, 0);
			targeted_xyz0[2] = xyz_0_4x1_mat.at<float>(2, 0);









			//search a mathed point

			//d_Error_min = 1000000;



			for (i = 0; i < (int)mesh_point0.size(); i++) {

				d_Error = (targeted_xyz0 - mesh_point0[i]).dot(mesh_normal0[i]);


				d_Error = fabs(d_Error);


				if (i == 0) {
					d_Error_min = d_Error;
					continue;
				}

				if (d_Error_min > d_Error) {

					d_Error_min = d_Error;
				}


			}//loop mesh point 0







			 //if (d_Error_min > ESTIMATE_T_MATRIX_OUTLIER_DISTANCE_THRESHOLD) continue;



			d_Energy += d_Error_min;

			n_point_num_to_estimate_T++;


		}//matching points







		if (n_point_num_to_estimate_T == 0) continue;


		d_Energy = d_Energy / (double)n_point_num_to_estimate_T;













		if (n_flag_first_processing < 0) {

			d_Energy_min = d_Energy;


			*d_Tx_estimated = d_Tx;
			*d_Ty_estimated = d_Ty;
			*d_Tz_estimated = d_Tz;


			n_flag_first_processing = 1;

			continue;



		}




		if (d_Energy_min > d_Energy) {

			d_Energy_min = d_Energy;


			*d_Tx_estimated = d_Tx;
			*d_Ty_estimated = d_Ty;
			*d_Tz_estimated = d_Tz;



		}




	}


	//}//k_x
	//}//k_y

	//}//k_z











}




void Iterative_Closest_Point_ICP(int n_mesh_size, const Mat& xyz_mat0, const Mat& xyz_mat1, const Mat& mask0, const Mat& mask1, const Mat& cloud_mask, const Mat& R_mat, const Mat& T_mat, double d_angle_thresh_to_detect_mesh_point, std::vector<cv::Point> &mesh_img_points0, std::vector<cv::Vec3f>& mesh_point0, std::vector<cv::Vec3f>& mesh_normal0, std::vector<cv::Vec3f>& mesh_point1, double *optimum_scale_alpha)
{

	int i, j;

	//std::vector<cv::Vec3f> mesh_point;
	//std::vector<cv::Vec3f> mesh_normal;
	cv::Vec3f targeted_mesh_point;
	cv::Point2i targeted_mesh_img_point;
	cv::Vec3f p0, p1, p2;
	cv::Vec3f vec_1, vec_2;
	cv::Vec3f patch_normal0;
	cv::Vec3f patch_normal1;

	double d_dot_product_normal_vectors;
	double d_angle_between_normal_vectors;


	//camera0
	for (j = 0; j < IMG_YSIZE - n_mesh_size * 2; j += n_mesh_size) {
		for (i = 0; i < IMG_XSIZE - n_mesh_size * 2; i += n_mesh_size) {


			if (mask0.at<uchar>(j, i) == 1 && mask0.at<uchar>(j, i + n_mesh_size) == 1 && mask0.at<uchar>(j + n_mesh_size, i) == 1 && mask0.at<uchar>(j, i + n_mesh_size * 2) == 1 && mask0.at<uchar>(j + n_mesh_size * 2, i) == 1 && cloud_mask.at<uchar>(j, i) == 0) {


				targeted_mesh_point = xyz_mat0.at<Vec3f>(j, i);
				targeted_mesh_img_point.x = i;
				targeted_mesh_img_point.y = j;



				//normal0
				p0 = xyz_mat0.at<Vec3f>(j, i);
				p1 = xyz_mat0.at<Vec3f>(j, i + n_mesh_size);
				p2 = xyz_mat0.at<Vec3f>(j + n_mesh_size, i);

				vec_1 = p1 - p0;
				vec_2 = p2 - p0;

				patch_normal0 = vec_2.cross(vec_1);
				patch_normal0 = patch_normal0 / norm(patch_normal0);



				//normal1
				p0 = xyz_mat0.at<Vec3f>(j, i + n_mesh_size);
				p1 = xyz_mat0.at<Vec3f>(j, i + n_mesh_size * 2);
				p2 = xyz_mat0.at<Vec3f>(j + n_mesh_size * 2, i);

				vec_1 = p1 - p0;
				vec_2 = p2 - p0;

				patch_normal1 = vec_2.cross(vec_1);
				patch_normal1 = patch_normal1 / norm(patch_normal1);



				//calc angle
				d_dot_product_normal_vectors = patch_normal0.dot(patch_normal1);


				d_angle_between_normal_vectors = acos(d_dot_product_normal_vectors) / PI*180.0;





				if (d_angle_between_normal_vectors > d_angle_thresh_to_detect_mesh_point) {


					mesh_point0.push_back(targeted_mesh_point);
					mesh_img_points0.push_back(targeted_mesh_img_point);
					mesh_normal0.push_back(patch_normal0);


				}



			}


		}
	}




	//camera1
	for (j = 0; j < IMG_YSIZE - n_mesh_size; j += 2) {
		for (i = 0; i < IMG_XSIZE - n_mesh_size; i += 2) {



			if (mask1.at<uchar>(j, i) == 1) {

				targeted_mesh_point = xyz_mat1.at<Vec3f>(j, i);

				mesh_point1.push_back(targeted_mesh_point);



			}
		}
	}


	int k;
	int n_step_num = 10;
	double d_step_distance = 100;
	double d_scale_factor_alpha;
	double d_Tx, d_Ty, d_Tz;
	cv::Vec3f targeted_xyz;
	cv::Vec3f targeted_xyz0;







	int n_point_num_to_estimate_T;

	double  d_optimum_scale_factor_alpha;
	double d_Error_min;
	double d_Error;
	double d_Energy;
	double d_Energy_min;





	for (k = 0; k < n_step_num; k++) {


		Mat TR_mat = Mat::eye(4, 4, CV_64FC1);
		Mat TR_inv_mat = Mat::eye(4, 4, CV_64FC1);
		Mat R4x4_mat = Mat::eye(4, 4, CV_64FC1);
		Mat T4x4_mat = Mat::eye(4, 4, CV_64FC1);
		Mat xyz_4x1_mat(4, 1, CV_64FC1);
		Mat xyz_0_4x1_mat(4, 1, CV_64FC1);





		d_scale_factor_alpha = k * d_step_distance;

		d_Tx = d_scale_factor_alpha*T_mat.at<double>(0);
		d_Ty = d_scale_factor_alpha*T_mat.at<double>(1);
		d_Tz = d_scale_factor_alpha*T_mat.at<double>(2);



		R4x4_mat.at<double>(0, 0) = R_mat.at<double>(0, 0);		R4x4_mat.at<double>(0, 1) = R_mat.at<double>(0, 1);		R4x4_mat.at<double>(0, 2) = R_mat.at<double>(0, 2);		R4x4_mat.at<double>(0, 3) = 0;// T_cam.at<double>(0);
		R4x4_mat.at<double>(1, 0) = R_mat.at<double>(1, 0);		R4x4_mat.at<double>(1, 1) = R_mat.at<double>(1, 1);		R4x4_mat.at<double>(1, 2) = R_mat.at<double>(1, 2);		R4x4_mat.at<double>(1, 3) = 0;// T_cam.at<double>(1);
		R4x4_mat.at<double>(2, 0) = R_mat.at<double>(2, 0);		R4x4_mat.at<double>(2, 1) = R_mat.at<double>(2, 1);		R4x4_mat.at<double>(2, 2) = R_mat.at<double>(2, 2);		R4x4_mat.at<double>(2, 3) = 0;// T_cam.at<double>(2);



		T4x4_mat = Mat::eye(4, 4, CV_64FC1);

		T4x4_mat.at<double>(0, 3) = d_Tx;
		T4x4_mat.at<double>(1, 3) = d_Ty;
		T4x4_mat.at<double>(2, 3) = d_Tz;

		TR_mat = T4x4_mat * R4x4_mat;
		TR_inv_mat = TR_mat.inv();



		d_Energy = 0;

		n_point_num_to_estimate_T = 0;


		for (j = 0; j < (int)mesh_point1.size(); j++) {


			targeted_xyz = mesh_point1[j];

			xyz_4x1_mat.at<float>(0, 0) = targeted_xyz[0];
			xyz_4x1_mat.at<float>(1, 0) = targeted_xyz[1];
			xyz_4x1_mat.at<float>(2, 0) = targeted_xyz[2];
			xyz_4x1_mat.at<float>(3, 0) = 1;

			xyz_0_4x1_mat = TR_inv_mat*xyz_4x1_mat;
			//xyz_0_4x1_mat = TR_mat*xyz_4x1_mat;

			targeted_xyz0[0] = xyz_0_4x1_mat.at<float>(0, 0);
			targeted_xyz0[1] = xyz_0_4x1_mat.at<float>(1, 0);
			targeted_xyz0[2] = xyz_0_4x1_mat.at<float>(2, 0);




			//search a mathed point

			d_Error_min = 1000000;

			for (i = 0; i < (int)mesh_point0.size(); i++) {

				d_Error = (targeted_xyz0 - mesh_point0[i]).dot(mesh_normal0[i]);
				d_Error = fabs(d_Error);

				if (d_Error_min > d_Error) {

					d_Error_min = d_Error;
				}


			}//loop mesh point 0



			 //if (d_Error_min > ESTIMATE_T_MATRIX_OUTLIER_DISTANCE_THRESHOLD) continue;



			d_Energy += d_Error_min;

			n_point_num_to_estimate_T++;


		}//matching points


		d_Energy = d_Energy / (double)n_point_num_to_estimate_T;

		if (k == 0) {
			d_Energy_min = d_Energy;
			d_optimum_scale_factor_alpha = d_scale_factor_alpha;

			continue;
		}



		if (d_Energy_min > d_Energy) {

			d_Energy_min = d_Energy;
			d_optimum_scale_factor_alpha = d_scale_factor_alpha;

		}






	}//loop alpha




	*optimum_scale_alpha = d_optimum_scale_factor_alpha;



}



int calc_and_save_T_matrix_using_3D_points(const Mat& R_mat, const Mat& T_mat, const Mat& xyz_mat0, const Mat& xyz_mat1, const std::vector<cv::Point2f> &points0, const std::vector<cv::Point2f> &points1, const Mat& mask0, const Mat& mask1, int n_ransac_loop_times, double d_ransac_thresh, double *d_Tx_estimated, double *d_Ty_estimated, double *d_Tz_estimated, double *d_alpha_estimated, double *d_T_abs_estimated, int *ransac_n_T_voted_num)

{



	//FILE* fp;
	//fopen_s(&fp, filename, "wt");


	int i, j;

	int x0, y0;
	int x1, y1;


	int n_total_point0_num;


	double d_Tx, d_Ty, d_Tz;
	double d_estimated_Tx, d_estimated_Ty, d_estimated_Tz;
	double d_estimated_T_abs;


	std::vector<double> vec_Tx;
	std::vector<double> vec_Ty;
	std::vector<double> vec_Tz;








	n_total_point0_num = 0;

	for (auto i = 0; i < (int)points0.size(); i++) {


		x0 = (int)points0[i].x;
		y0 = (int)points0[i].y;
		x1 = (int)points1[i].x;
		y1 = (int)points1[i].y;


		Vec3f pt3D_0 = xyz_mat0.at<Vec3f>(y0, x0);
		Vec3f pt3D_1 = xyz_mat1.at<Vec3f>(y1, x1);


		if (pt3D_0[2] > ESTIMATE_T_MATRIX_Z_RANGE_FROM_CAMERA) continue;



		if (mask0.at<uchar>(y0, x0) != 1) continue;
		if (mask1.at<uchar>(y1, x1) != 1) continue;



		d_Tx = pt3D_1[0] - (R_mat.at<double>(0, 0)*pt3D_0[0] + R_mat.at<double>(0, 1)*pt3D_0[1] + R_mat.at<double>(0, 2)*pt3D_0[2]);
		d_Ty = pt3D_1[1] - (R_mat.at<double>(1, 0)*pt3D_0[0] + R_mat.at<double>(1, 1)*pt3D_0[1] + R_mat.at<double>(1, 2)*pt3D_0[2]);
		d_Tz = pt3D_1[2] - (R_mat.at<double>(2, 0)*pt3D_0[0] + R_mat.at<double>(2, 1)*pt3D_0[1] + R_mat.at<double>(2, 2)*pt3D_0[2]);


		vec_Tx.push_back(d_Tx);
		vec_Ty.push_back(d_Ty);
		vec_Tz.push_back(d_Tz);


		//fprintf_s(fp, "%lf\t%lf\t%lf\n", d_Tx, d_Ty, d_Tz);



		n_total_point0_num++;


	}

	if (n_total_point0_num < ESTIMATE_T_MATRIX_RANSAC_VOTE_THRESHOLD) return -1;




	//ransac
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<int> do_sampling_estimate_T_matrix(0, n_total_point0_num - 1);

	int n_ransac_sample_No;
	int n_ransac_voted_num;

	int n_ransac_voted_num_max;

	double d_alpha;
	double d_candidate_alpha;

	double d_error;


	n_ransac_voted_num_max = 0;


	for (i = 0; i < n_ransac_loop_times; i++) {



		n_ransac_sample_No = do_sampling_estimate_T_matrix(mt);

		d_alpha = T_mat.at<double>(0)*vec_Tx[n_ransac_sample_No] + T_mat.at<double>(1)*vec_Ty[n_ransac_sample_No] + T_mat.at<double>(2)*vec_Tz[n_ransac_sample_No];



		//////////////
		/////////////
		if (d_alpha < 0 || d_alpha > 1500) continue;
		/////////////
		/////////////




		n_ransac_voted_num = 0;

		for (j = 0; j < n_total_point0_num; j++) {

			d_error = (d_alpha*T_mat.at<double>(0) - vec_Tx[j])*(d_alpha*T_mat.at<double>(0) - vec_Tx[j]) + (d_alpha*T_mat.at<double>(1) - vec_Ty[j])*(d_alpha*T_mat.at<double>(1) - vec_Ty[j]) + (d_alpha*T_mat.at<double>(2) - vec_Tz[j])*(d_alpha*T_mat.at<double>(2) - vec_Tz[j]);
			d_error = sqrt(d_error);


			if (d_error < d_ransac_thresh) n_ransac_voted_num++;


		}

		if (n_ransac_voted_num_max < n_ransac_voted_num) {
			n_ransac_voted_num_max = n_ransac_voted_num;
			d_candidate_alpha = d_alpha;
		}


	}



	int n_candidate_data_num;
	double d_sum_T_dot_t;
	double d_estimated_alpha;


	n_candidate_data_num = 0;
	d_sum_T_dot_t = 0;

	for (j = 0; j < n_total_point0_num; j++) {

		d_error = (d_candidate_alpha*T_mat.at<double>(0) - vec_Tx[j])*(d_candidate_alpha*T_mat.at<double>(0) - vec_Tx[j]) + (d_candidate_alpha*T_mat.at<double>(1) - vec_Ty[j])*(d_candidate_alpha*T_mat.at<double>(1) - vec_Ty[j]) + (d_candidate_alpha*T_mat.at<double>(2) - vec_Tz[j])*(d_candidate_alpha*T_mat.at<double>(2) - vec_Tz[j]);
		d_error = sqrt(d_error);


		if (d_error < d_ransac_thresh) {


			d_sum_T_dot_t += T_mat.at<double>(0)*vec_Tx[j] + T_mat.at<double>(1)*vec_Ty[j] + T_mat.at<double>(2)*vec_Tz[j];


			n_candidate_data_num++;
		}

	}

	if (n_candidate_data_num < ESTIMATE_T_MATRIX_RANSAC_VOTE_THRESHOLD) return -1;




	d_estimated_alpha = d_sum_T_dot_t / (double)n_candidate_data_num;




	//d_estimated_alpha = 200;



	d_estimated_Tx = d_estimated_alpha * T_mat.at<double>(0);
	d_estimated_Ty = d_estimated_alpha * T_mat.at<double>(1);
	d_estimated_Tz = d_estimated_alpha * T_mat.at<double>(2);


	d_estimated_T_abs = d_estimated_Tx*d_estimated_Tx + d_estimated_Ty*d_estimated_Ty + d_estimated_Tz*d_estimated_Tz;




	*d_T_abs_estimated = sqrt(d_estimated_T_abs);

	*d_alpha_estimated = d_estimated_alpha;

	*d_Tx_estimated = d_estimated_Tx;
	*d_Ty_estimated = d_estimated_Ty;
	*d_Tz_estimated = d_estimated_Tz;

	*ransac_n_T_voted_num = n_candidate_data_num;



	//fprintf(fp, "%lf\t%lf\t%lf\n", d_candidate_Tx, d_candidate_Ty, d_candidate_Tz);


	//fclose(fp);


	return 1;

}



int thread_save_corresponding_points(const char* filename)
{




	Mat buf_xyz0;
	Mat buf_xyz1;
	std::vector<cv::Point2f> buf_point1;
	std::vector<cv::Point2f> buf_point0;
	//std::vector<cv::KeyPoint > buf_point1;

	Mat buf_mask0;
	Mat buf_mask1;


	std::vector<uint> buf_descriptor0_No;
	std::vector<uint> buf_descriptor1_No;
	Mat buf_descriptor0;
	Mat buf_descriptor1;


	//Mat buf_estimated_R_T;


	buf_xyz0 = xyz_0.clone();
	buf_xyz1 = xyz.clone();



	buf_point0 = Homography_keypoint0;
	buf_point1 = Homography_keypoint1;
	//buf_point1 = Keypoints_1;

	buf_mask0 = mask_with_xyz_0.clone();
	buf_mask1 = mask_with_xyz.clone();


	buf_descriptor0_No = Good_descriptor0_No;
	buf_descriptor1_No = Good_descriptor1_No;

	buf_descriptor0 = Descriptor0.clone();
	buf_descriptor1 = Descriptor1.clone();



	save_corresponding_points(filename, buf_xyz0, buf_xyz1, buf_point0, buf_point1, buf_mask0, buf_mask1, buf_descriptor0, buf_descriptor0_No, buf_descriptor1, buf_descriptor1_No, (int)buf_descriptor0.size().width);
	//save_corresponding_points(filename, buf_xyz1, buf_point1, buf_mask1, buf_descriptor1, (int)buf_descriptor1.size().width);

	n_frame_count_3D_data++;

	return n_frame_count_3D_data;


}





static void save_corresponding_points(const char* filename, const Mat& mat0, const Mat& mat1, const std::vector<cv::Point2f> &points0, const std::vector<cv::Point2f> &points1, const Mat& mask0, const Mat& mask1, const Mat& descriptor0, std::vector<uint> &descriptor0_no, const Mat& descriptor1, std::vector<uint> &descriptor1_no, int n_descriptor_vector_size)

{
	int x0, y0;
	int x1, y1;



	//const double max_z = STEREO_Z_DEPTH_MAX;// 1.0e4;



	FILE* fp;
	fopen_s(&fp, filename, "wt");


	if ((int)points0.size() != descriptor0_no.size()) std::cout << "keypoint0 descriptor mismatched!! " << to_string((int)points0.size()) << "!=" << to_string(descriptor0_no.size()) << "\n" << std::string(filename) << endl;
	if ((int)points1.size() != descriptor1_no.size()) std::cout << "keipoint1 descriptor mismatched!! " << to_string((int)points1.size()) << "!=" << to_string(descriptor1_no.size()) << "\n" << std::string(filename) << endl;




	for (auto i = 0; i < (int)points0.size(); i++) {


		x0 = (int)points0[i].x;
		y0 = (int)points0[i].y;
		x1 = (int)points1[i].x;
		y1 = (int)points1[i].y;


		Vec3f pt3D_0 = mat0.at<Vec3f>(y0, x0);
		Vec3f pt3D_1 = mat1.at<Vec3f>(y1, x1);



		if (mask0.at<uchar>(y0, x0) != 1) continue;
		if (mask1.at<uchar>(y1, x1) != 1) continue;



		fprintf_s(fp, "%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f", x0, y0, x1, y1, pt3D_0[0], pt3D_0[1], pt3D_0[2], pt3D_1[0], pt3D_1[1], pt3D_1[2]);


		fprintf_s(fp, "\t%d", n_descriptor_vector_size);
		for (auto j = 0; j < n_descriptor_vector_size; j++) {
			fprintf_s(fp, "\t%d", descriptor0.at<uchar>(descriptor0_no[i], j));
		}

		fprintf_s(fp, "\t%d", n_descriptor_vector_size);
		for (auto j = 0; j < n_descriptor_vector_size; j++) {
			fprintf_s(fp, "\t%d", descriptor1.at<uchar>(descriptor1_no[i], j));
		}

		fprintf_s(fp, "\n");


	}








	fclose(fp);

}



/*
static void save_corresponding_points(const char* filename, const Mat& mat0, const std::vector<cv::KeyPoint > &points0, const Mat& mask0, const Mat& descriptor0, int n_descriptor_vector_size)

{
int x0, y0;
//int x1, y1;



//const double max_z = STEREO_Z_DEPTH_MAX;// 1.0e4;



FILE* fp;
fopen_s(&fp, filename, "wt");




for (auto i = 0; i < (int)points0.size(); i++){


x0 = (int)points0[i].pt.x;
y0 = (int)points0[i].pt.y;


if (mask0.at<uchar>(y0, x0) != 1) continue;




Vec3f pt3D_0 = mat0.at<Vec3f>(y0, x0);


fprintf_s(fp, "%d\t%d\t%f\t%f\t%f", x0, y0, pt3D_0[0], pt3D_0[1], pt3D_0[2]);


fprintf_s(fp, "\t%d", n_descriptor_vector_size);

for (auto j = 0; j < n_descriptor_vector_size; j++){
fprintf_s(fp, "\t%d", descriptor0.at<uchar>(i, j));
}



fprintf_s(fp, "\n");


}






fclose(fp);

}


*/

int thread_save_Keypoints_depth(const char* filename)
{




	Mat buf_xyz;
	std::vector<cv::KeyPoint > buf_point;
	Mat buf_mask;
	Mat buf_descriptor;


	buf_xyz = xyz.clone();
	buf_point = Keypoints_1;
	buf_mask = mask_with_xyz;
	buf_descriptor = Descriptor1.clone();


	save_keypoints_depth(filename, buf_xyz, buf_point, buf_mask, buf_descriptor, (int)buf_descriptor.size().width);




	n_frame_count_keypoints_XYZ++;

	return n_frame_count_keypoints_XYZ;


}



static void save_keypoints_depth(const char* filename, const Mat& mat0, const std::vector<cv::KeyPoint > &points0, const Mat& mask0, const Mat& descriptor0, int n_descriptor_vector_size)

{
	int x0, y0;
	//int x1, y1;



	//const double max_z = STEREO_Z_DEPTH_MAX;// 1.0e4;



	FILE* fp;
	fopen_s(&fp, filename, "wt");




	for (auto i = 0; i < (int)points0.size(); i++) {


		x0 = (int)points0[i].pt.x;
		y0 = (int)points0[i].pt.y;


		if (mask0.at<uchar>(y0, x0) != 1) continue;




		Vec3f pt3D_0 = mat0.at<Vec3f>(y0, x0);


		fprintf_s(fp, "%d\t%d\t%f\t%f\t%f", x0, y0, pt3D_0[0], pt3D_0[1], pt3D_0[2]);


		fprintf_s(fp, "\t%d", n_descriptor_vector_size);

		for (auto j = 0; j < n_descriptor_vector_size; j++) {
			fprintf_s(fp, "\t%d", descriptor0.at<uchar>(i, j));
		}



		fprintf_s(fp, "\n");


	}






	fclose(fp);

}











string make_n_digit_number(int in_No, int n_digit_num)
{
	string s_out_number;

	ostringstream oss;

	oss.setf(ios::right);

	oss.fill('0');
	oss.width(n_digit_num);

	oss << in_No;
	s_out_number = oss.str();


	return s_out_number;


}


void HSV_to_RGB(double H, double S, double V, unsigned char *R, unsigned char *G, unsigned char *B) // H:0-360, S:0-255, V:0-255
{

	double Max;
	double Min;


	Max = V;
	Min = Max - ((S / 255.0)*Max);

	if (H >= 0 && H <= 60) {

		*R = (unsigned char)Max;
		*G = (unsigned char)((H / 60.0)*(Max - Min) + Min);
		*B = (unsigned char)Min;
	}
	if (H > 60 && H <= 120) {
		*R = (unsigned char)(((120 - H) / 60.0) * (Max - Min) + Min);
		*G = (unsigned char)Max;
		*B = (unsigned char)Min;
	}
	if (H > 120 && H <= 180) {
		*R = (unsigned char)Min;
		*G = (unsigned char)Max;
		*B = (unsigned char)(((H - 120) / 60.0)*(Max - Min) + Min);
	}
	if (H > 180 && H <= 240) {
		*R = (unsigned char)Min;
		*G = (unsigned char)(((240 - H) / 60.0)* (Max - Min) + Min);
		*B = (unsigned char)Max;
	}
	if (H > 240 && H <= 300) {
		*R = (unsigned char)(((H - 240) / 60.0)* (Max - Min) + Min);
		*G = (unsigned char)Min;
		*B = (unsigned char)Max;
	}
	if (H > 300 && H <= 360) {
		*R = (unsigned char)Max;
		*G = (unsigned char)Min;
		*B = (unsigned char)(((360 - H) / 60.0)*(Max - Min) + Min);
	}



}
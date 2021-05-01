#define _USE_MATH_DEFINES


//#define STEREO_BM
#define STEREO_SGBM
//#define USE_YOLO_BOUNDING_BOX_INFO
//#define DETECT_

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

#include "Func_Kalman.h"




#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/ximgproc.hpp>


#


#define PI	3.141592


#define	IMG_XSIZE			672
#define	IMG_YSIZE			376




#define HISTOGRAM_LEVEL_MAX	256

#define	MIN_MATCH_COUNT		15
#define	STEREO_Z_DEPTH_MAX	1.0e4
#define THRESH_TO_DETECT_OUTLIER_OF_MOVING_DISTANCE		5000   //for Kalman Filter
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

#define STEREO_BLOCK_MATCHING_DISPARITIES_SIZE		16*2 //16*3 16*5
#define STEREO_BLOCK_MATCHING_SADWINDOW_SIZE		21//21 //17
#define STEREO_BLOCK_MATCHING_SPECKLE_WINDOW_SIZE	200
#define STEREO_BLOCK_MATCHING_SPECKLE_RANGE			1

//#define CLOUD_REGION_HSV_S_UPPER_THRESH				87 //blue sky 52,99
//#define CLOUD_REGION_HSV_V_LOWER_THRESH				40//85//96

#define SURFACE_NORMAL_PATCH_SIZE_WIDTH					10
#define SURFACE_NORMAL_PATCH_SIZE_HEIGTH				10




#define SLAM_STEREO_3D_DATA_OUTPUT_STEP					10

#define YOLO_FILE_LINE_MAX_BUFFER_SIZE	1024
#define YOLO_LABEL_STRING_MAX_LENGTH	100
#define YOLO_BOUNDING_BOX_OBJECT_NUM_MAX				50
#define YOLO_BOUNDING_BOX_3DPOINT_OUTPUT_DATA_STEP		30//4
#define YOLO_BOUNDING_BOX_INTERSECTION_VOTE_DISTANCE_MAX		50  //meter

//#define YOLO_BOUNDING_BOX_DETECT_OBJECT_DISTANCE_LOWER_LIMIT	1000
//#define YOLO_BOUNDING_BOX_DETECT_OBJECT_DISTANCE_UPPER_LIMIT	1500000 //mm



#define DAIJKSTRA_MAP_BLOCK_PITCH	0.2
#define DAIJKSTRA_MAP_X_RANGE_MAX	500 // m
#define DAIJKSTRA_MAP_X_RANGE_MIN	-500 // m
#define DAIJKSTRA_MAP_Z_RANGE_MAX	500 // m
#define DAIJKSTRA_MAP_Z_RANGE_MIN	-500 // m
#define DAIJKSTRA_MAP_X_BLOCK_NUM	int((DAIJKSTRA_MAP_X_RANGE_MAX - DAIJKSTRA_MAP_X_RANGE_MIN)/DAIJKSTRA_MAP_BLOCK_PITCH)
#define DAIJKSTRA_MAP_Z_BLOCK_NUM	int((DAIJKSTRA_MAP_Z_RANGE_MAX - DAIJKSTRA_MAP_Z_RANGE_MIN)/DAIJKSTRA_MAP_BLOCK_PITCH)
#define DAIJKSTRA_MAP_ORIGIN_X_BLOCK_NO	DAIJKSTRA_MAP_X_BLOCK_NUM/2
#define DAIJKSTRA_MAP_ORIGIN_Z_BLOCK_NO	DAIJKSTRA_MAP_Z_BLOCK_NUM/2














struct interested_point_xy {

	int x;
	int y;

};




struct RGB_color {

	unsigned char R;
	unsigned char G;
	unsigned char B;
};

struct yolo_bounding_box {

	int x;
	int y;
	int width;
	int height;
	char name[YOLO_LABEL_STRING_MAX_LENGTH];
	double likelihood;

};


struct yolo_bounding_box_cluster_point {

	double x;
	double y;
	double z;


	int id;

	int count;
	double sum_x;
	double sum_y;
	double sum_z;

};


struct yolo_bounding_box_point {

	double x;
	double y;
	double z;


	//int id;


};




struct yolo_bounding_box_object_center {

	double x;
	double y;
	double z;

	int n_object_pixel_num;
	int n_bounding_box_pixel_num;
	double d_boject_pixel_ratio_to_box;

};

struct yolo_object_detected_position {



	double cx;
	double cy;
	double cz;

	double sum_x;
	double sum_y;
	double sum_z;

	int counter;

	//char name[YOLO_LABEL_STRING_MAX_LENGTH];


};


struct yolo_objects_of_interest_detected_position {

	int point_num;

	int box_height;
	int box_width;

	double cx;
	double cy;
	double cz;

	double distance_from_camera;



	//char name[YOLO_LABEL_STRING_MAX_LENGTH];


};

struct slam_map_route_xyz {

	double x;
	double y;
	double z;

	int map_block_no_x;
	int map_block_no_z;
	unsigned int map_node_no;

};

struct slam_xyz {

	double x;
	double y;
	double z;

	double time_stamp;
};



enum yolo_bounding_box_mode {

	ONLY_MAKE_REMAP_MOVIE,
	ONLY_CALC_TRAVELING_DISTANCE,
	NOT_USE_YOLO,
	YOLO_UME,
	YOLO_OBJECTS


};




using namespace std;
using namespace cv;
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
string sFolder_save_stereo_3D;
string sFolder_save_stereo_3D_matched;
string sFolder_save_keypoints_depth;
string sFolder_save_keyimages;
string sFolder_save_triangulatePoints;
string sFolder_save_landmark_training;
string sFolder_landmark_matching;


string sFilePath1, sFilePath2, sFilePath3;
string sMovieFname;
string sFilePath_movie1, sFilePath_movie2;

string sFilePath_stereo_calibration_parameters;
string sSaveFilePath_stereo_calibration_parameters_yaml;
string sSaveFilePath_cam1_parameters;
string sSaveFilePath_cam2_parameters;
string sSaveFilePath_stereo_calibration;

string sFolder_save_stereo_results;
string sSaveFilePath_stereo_3D;
string sSaveFilePath_moive_stereo;
string sSaveFilePath_moive_mask_sub_image;
string sSaveFilePath_moive_cam1_remap;
string sSaveFilePath_Landmark_image_left;
string sSaveFilePath_Landmark_image_right;
string sSaveFilePath_Landmark_keypoints;
string sSaveFilePath_SLAM_mapping_objects;
string sSaveFilePath_yolo_bounding_box_objects;
string sSaveFilePath_yolo_detected_objects_use_NN_clustering;
string sSaveFilePath_yolo_bounding_box_objects_of_interest;
string sSaveFilePath_yolo_bounding_box_road_edge_line;
string sSaveFilePath_grids_between_left_edge_and_right_edge;
string sSaveFilePath_Daijkstra_blocks;
string sSaveFilePath_Daijkstra_VehiclePosition;

string sFname_field_map_image;
string sFilePath_Daijkstra_field_map_image;
string sSaveFilePath_Daijkstra_field_map_image;

string sFilePath_ground_plane_estimation_parameters;

string sFilePath_ORB_SLAM_trajectory;
string sFilePath_YOLO_BoundingBox;


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
string save_file_path_Triangulate_points = "";
string save_file_path_vehicle_position_use_landmark = "";
string save_file_path_traveling_distance = "";
string save_file_path_traveling_velocity = "";

string sFilePath_landmark_keypoints_previous = "";
string sFilePath_landmark_keypoints_current = "";
string sFilePath_landmark_keypoints_next = "";


string sFrameNo;
string s_yolo_object_name_road_edge;

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







Mat camera_axis_1(3, 1, CV_64FC1);
Mat camera_axis_2(3, 1, CV_64FC1);
Mat camera_axis_3(3, 1, CV_64FC1);
Mat camera_axis_1n(3, 1, CV_64FC1);
Mat camera_axis_2n(3, 1, CV_64FC1);
Mat camera_axis_3n(3, 1, CV_64FC1);


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
int n_flag_done_calc_PnP_solve1;
int n_flag_done_calc_PnP_solve2;
int n_flag_detected_point_num_enough_to_estimate_3D_plane;

int n_flag_done_estimation_default_ground_normal_vector;




struct interested_point_xy targeted_3D_Point[IMG_YSIZE*IMG_XSIZE];
struct interested_point_xy all_3D_Point[IMG_YSIZE*IMG_XSIZE];
struct RGB_color color_table_RGB[360];



//kalman filter
struct kalman_filter_parameters kalman_param_distance;











// yolo bb parameters //

char yolo_bb_file_BUFFER[YOLO_FILE_LINE_MAX_BUFFER_SIZE];
//char yolo_bb_label_name[YOLO_LABEL_STRING_MAX_LENGTH];

int histogram_TANK[HISTOGRAM_LEVEL_MAX];

std::vector<struct yolo_bounding_box_object_center> yolo_bbox_object_center;
struct yolo_bounding_box_object_center buf_yolo_bbox_object_center;

std::vector<struct yolo_object_detected_position> yolo_detected_Object;
struct yolo_object_detected_position buf_yolo_detected_object;


std::vector<struct slam_map_route_xyz> slam_xyz;
struct slam_map_route_xyz buf_slam_xyz;






enum yolo_bounding_box_mode YOLO_mode = NOT_USE_YOLO;
char yolo_bounding_box_targeted_object_name_list[YOLO_BOUNDING_BOX_OBJECT_NUM_MAX][YOLO_LABEL_STRING_MAX_LENGTH];
char yolo_bounding_box_label_name_of_road_edge[YOLO_LABEL_STRING_MAX_LENGTH];




// Daijkstra

unsigned int daijkstra_field_MAP[DAIJKSTRA_MAP_Z_BLOCK_NUM][DAIJKSTRA_MAP_X_BLOCK_NUM];












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
unsigned _stdcall thread_gnuplot(void *p); //_stdcall...呼び出し規約(https://konuma.org/blog/2006/01/02/post_1fd3/)



int determine_ootsu_auto_thresh_in_bounding_box_gray_image(const Mat &img_gray, int nLTx, int nLTy, int nRBx, int nRBy, int histogram_tank[HISTOGRAM_LEVEL_MAX]);
int determine_primary_object_center_in_bounding_box_using_k_means(std::vector<struct yolo_bounding_box_cluster_point> &bb_point, int n_cluster_candidates_num, int n_clustering_loop_times, struct yolo_bounding_box_cluster_point *object_center);

void detect_road_edge_line_using_RANSAC(std::vector<struct yolo_bounding_box_point> &bb_point, int ransac_loop_times, double ransac_d_threshold_distance_to_vote, double *distance_from_camera_to_edge, int *ransac_voted_count);
void detect_road_edge_line_using_VotingMethod(std::vector<struct yolo_bounding_box_point> &bb_point, double road_edge_distance_range_min, double road_edge_distance_range_max, double road_edge_distance_resolution, int thresh_voted_count, int * flag_could_detect_edge, double *distance_from_camera_to_edge, int *voted_count);
void detect_road_center_line_in_image_use_RANSAC(vector<struct interested_point_xy> &center_line_image_points, int ransac_loop_times, double thresh_distance_between_point_and_line, struct interested_point_xy *center_line_org, double center_line_vector[2], double center_line_perpendicular_vector[2]);





























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
	Mat img_mask_no_image_region(IMG_YSIZE, IMG_XSIZE, CV_8UC1);
	Mat img_mask_no_image_region_remap(IMG_YSIZE, IMG_XSIZE, CV_8UC1);

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







	Mat img_field_MAP(DAIJKSTRA_MAP_Z_BLOCK_NUM, DAIJKSTRA_MAP_X_BLOCK_NUM, CV_8UC1, cv::Scalar(0));





	vector<vector<Point2f>>imagePoints1, imagePoints2;
	vector<Point2f> imageCorners1, imageCorners2;
	vector<vector < Point3f >> objectPoints;
	vector<Point3f>objectCorners;

	Mat K1, K2; //Mat cameraMatrix1, cameraMatrix2;
	Mat D1, D2; //Mat distCoeffs1, distCoeffs2;
	vector<Mat> rvecs1, rvecs2;
	vector<Mat> tvecs1, tvecs2;

	Mat undistortion_coeff = Mat::zeros(1, 5, CV_64FC1);



	Mat R, F, E;
	Vec3d T;

	Mat R1, R2, P1, P2, Q;















	camera_axis_1.at<double>(0, 0) = 1.0;
	camera_axis_1.at<double>(1, 0) = 0;
	camera_axis_1.at<double>(2, 0) = 0;


	camera_axis_2.at<double>(0, 0) = 0;
	camera_axis_2.at<double>(1, 0) = 1.0;
	camera_axis_2.at<double>(2, 0) = 0;

	camera_axis_3.at<double>(0, 0) = 0;
	camera_axis_3.at<double>(1, 0) = 0;
	camera_axis_3.at<double>(2, 0) = 1.0;


	camera_axis_1n = camera_axis_1.clone();
	camera_axis_2n = camera_axis_2.clone();
	camera_axis_3n = camera_axis_3.clone();













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












	int n_From_movie_frame;
	//int n_To_movie_frame;
	//	int n_SLAM_frame_step;
	//	int n_3D_data_saving_step;






	int n_frame_count = 0;
	int n_current_frame_No;
	int n_frame_step_saving_traveling_distance = 30; //30 frames is equal to 1 second.

													 //int n_do_SLAM_timing;
													 //	int n_save_3D_data_timing;









	double d_object_distance_from_camera_thresh_min;
	double d_object_distance_from_camera_thresh_max;

	double d_object_FOV_limit_height_min = -1000;
	double d_object_FOV_limit_distance = 3000;







	int n_yolo_bb_object_cluster_candidates_num;
	int n_yolo_bb_object_clustering_k_means_loop_times;
	int n_yolo_bb_point_num_magnification_to_do_clustering;
	int n_yolo_bb_object_pixel_num_thresh;
	double d_distance_thresh_in_nearest_neighborhood_method;
	//	double d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox_thresh;
	int n_yolo_bb_window_half_size_to_acquire_xyz_coordinates;
	int n_road_edge_line_distance_estimation_moving_average_method_frame_number;
	int n_road_edge_line_distance_estimation_bbox_window_center_position_div_num;
	double d_road_edge_line_estimation_distance_resolution;

	double d_road_edge_traveling_grid_pitch_x;
	double d_road_edge_traveling_grid_pitch_z;
	int		n_road_edge_traveling_grid_num_x;
	int		n_road_edge_traveling_grid_num_z;


	int n_yolo_bb_targeted_object_num = 0;














	int n_cloud_HSV_S_upper_thresh;// = int(255.0 * CLOUD_REGION_HSV_S_UPPER_THRESH / 100.0); //0-100 => 0-255
	int n_cloud_HSV_V_lower_thresh;// = int(255.0 * CLOUD_REGION_HSV_V_LOWER_THRESH / 100.0); //0-100 => 0-255


	int n_sky_detction_canny_edge_thresh_low = 50;
	int n_sky_detction_canny_edge_thresh_high = 200;


	int n_non_texure_region_detction_sobel_edge_thresh;// = 10;


													   //estimate 3D plane
													   //	int n_all_3D_points_num;
													   //	int n_targeted_3D_point_num;

	Vec3f dominant_plane_origin;
	Vec3f dominant_plane_vector1, dominant_plane_vector2;
	Vec3f dominant_plane_normal;



	Vec3f vec_camera_x_axis = Vec3f(1, 0, 0);
	Vec3f vec_camera_y_axis = Vec3f(0, 1, 0);
	Vec3f vec_camera_z_axis = Vec3f(0, 0, 1);

	Mat backup_pca_eigen_vectors, backup_pca_eigen_values;
	Mat backup_pca_eigen_org;















	double d_angle_between_ground_normal_and_y_axis;

	//	int n_flag_landmark_mode_select;
	int n_flag_landmark_training_mode = 0;
	int n_flag_landmark_matching_mode = 0;


	int user_mode_using_yolo_bounding_box;

















	cout << "PROGRAM DOCUMENT" << endl;
	cout << endl;
	cout << "Movie file name: ****_cam1.mov, ****_cam2.mov" << endl;
	cout << "set the file name in the parameter file" << endl;
	cout << endl;
	cout << "SGBM stereo mode: #define STEREO_SGBM" << endl;
	cout << "BM stereo mode: #define STEREO_BM" << endl;
	cout << "if use yolo bounding box to detect tree, #define USE_YOLO_BOUNDING_BOX_INFO" << endl;
	cout << "if not define USE_YOLO_BOUNDING_BOX_INFO, all of pixels will be saved at the interval of specific frames" << endl;
	cout << endl;
	cout << endl;


	cout << "SELECT YOLO MODE" << endl;
	cout << to_string(ONLY_MAKE_REMAP_MOVIE) + " | ONLY_MAKE_REMAP_MOVIE : Only making cam1 remap movie file" << endl;
	cout << to_string(ONLY_CALC_TRAVELING_DISTANCE) + " | ONLY_CALC_TRAVELING_DISTANCE : Calc traveling distance " << endl;
	cout << to_string(NOT_USE_YOLO) + " | NOT_USE_YOLO : Not using yolo bounding box" << endl;
	cout << to_string(YOLO_UME) + " | YOLO_UME     : Ume tree detection using yolo bounding box" << endl;
	cout << to_string(YOLO_OBJECTS) + " | YOLO_OBJECTS : Objects detection using yolo bounding box" << endl;

	cout << "Mode = ";
	cin >> user_mode_using_yolo_bounding_box;






	switch (user_mode_using_yolo_bounding_box) {



	case 0:

		YOLO_mode = ONLY_MAKE_REMAP_MOVIE;
		cout << "ONLY_MAKE_REMAP_MOVIE" << endl;
		break;
	case 1:
		YOLO_mode = ONLY_CALC_TRAVELING_DISTANCE;
		cout << "ONLY_CALC_TRAVELING_DISTANCE" << endl;
		break;
	case 2:
		YOLO_mode = NOT_USE_YOLO;
		cout << "NOT_USE_YOLO" << endl;
		break;
	case 3:
		YOLO_mode = YOLO_UME;
		cout << "YOLO UME" << endl;
		break;
	case 4:
		YOLO_mode = YOLO_OBJECTS;
		cout << "YOLO_OBJECTS" << endl;
		break;



	default:
		cout << "Command No. error!" << endl;
		return -1;
		break;
	}




	cout << endl;
	cout << endl;





















	//generate a color table
	for (int h = 0; h < 360; h++) {

		HSV_to_RGB((double)h, 255.0, 255.0, &color_table_RGB[h].R, &color_table_RGB[h].G, &color_table_RGB[h].B);
		//printf("%d %d %d\n", color_table_RGB[h].R, color_table_RGB[h].G, color_table_RGB[h].B);



	}































	std::cout << "Main folder : ";
	std::cin >> sMainFolder;


	//std::cout << "movie file (***_cam1.mov, ***_cam2.mov) ex. movie_stereo: ";
	//std::cin >> sMovieFname;






	//std::cout << "Targeted frames: Start-frame No. = ";
	//std::cin >> n_From_movie_frame;
	n_From_movie_frame = 0;


	//std::cout << "Save stereo data: Frame step (60) = ";
	//std::cin >> n_3D_data_saving_step;














	cv::Mat mat_slam_R(3, 3, CV_64FC1);

	double buffer_load_slam_trajectory;


	double slam_trajectory_d_time;
	double slam_trajectory_d_Twc_X, slam_trajectory_d_Twc_Y, slam_trajectory_d_Twc_Z;
	double slam_trajectory_d_prev_Twc_X, slam_trajectory_d_prev_Twc_Y, slam_trajectory_d_prev_Twc_Z;
	double slam_trajectory_d_moving_distance;
	double slam_trajectory_d_total_traveling_distance;
	double slam_trajectory_d_moving_velocity;
	//double slam_trajectory_d_Quaternion[4];
	//double slam_trajectory_d_axis_x[3], slam_trajectory_d_axis_y[3], slam_trajectory_d_axis_z[3];







	FILE *fp_orb_slam_trajectory = NULL;
	FILE *fp_w_traveling_distance = NULL;
	FILE *fp_w_traveling_velocity = NULL;

	errno_t err_slam_trajectory;


	int n_calc_data_step;























	if (YOLO_mode != ONLY_MAKE_REMAP_MOVIE) {



		

		cout << "Calc data step (default: 30) = ";
		cin >> n_calc_data_step;


		///////////////// SAVE FILE INFO //////////////////////


		save_file_path_traveling_distance = sMainFolder + "/" + "output_traveling_distance_step" + to_string(n_calc_data_step) + ".txt";



		fopen_s(&fp_w_traveling_distance, save_file_path_traveling_distance.c_str(), "wt");

		fprintf(fp_w_traveling_distance, "#Time\tdistance\n");




		save_file_path_traveling_velocity = sMainFolder + "/" + "output_traveling_moving_velocity_step" + to_string(n_calc_data_step) + ".txt";



		fopen_s(&fp_w_traveling_velocity, save_file_path_traveling_velocity.c_str(), "wt");

		fprintf(fp_w_traveling_velocity, "#Time\tvelocity\n");
		



















		///////////////// SLAM TRAJECTORY INFO AND PROCESSING /////////////////









		std::cout << "File path: ORB SLAM trajectory = ";
		std::cin >> sFilePath_ORB_SLAM_trajectory;

		err_slam_trajectory = fopen_s(&fp_orb_slam_trajectory, sFilePath_ORB_SLAM_trajectory.c_str(), "rt");


		if (err_slam_trajectory != 0) {
			std::cout << "Cannot load the orb-slam trajectory data:" << endl;
			std::cout << sFilePath_ORB_SLAM_trajectory << endl;
			return -1;
		}




		fprintf(fp_w_traveling_distance, "#Default sampling frame interval = %d\n", n_frame_step_saving_traveling_distance);










	}





















	if (YOLO_mode == ONLY_CALC_TRAVELING_DISTANCE) {





		
		int n_count;

		int n_ring_buffer_pos_current;
		int n_ring_buffer_pos_oldest;

		vector<struct slam_xyz> vehicle_trajectory(n_calc_data_step);


		for (int i = 0; i < n_calc_data_step; i++) {

			vehicle_trajectory[i].x = 0;
			vehicle_trajectory[i].y = 0;
			vehicle_trajectory[i].z = 0;


		}




		




		fprintf(fp_w_traveling_distance, "#The sampling frame interval was set to %d\n", n_calc_data_step);






		slam_trajectory_d_prev_Twc_X = slam_trajectory_d_prev_Twc_Y = slam_trajectory_d_prev_Twc_Z = 0;

		slam_trajectory_d_total_traveling_distance = 0;


		n_count = 0;

		while ((fscanf_s(fp_orb_slam_trajectory, "%lf", &slam_trajectory_d_time)) != EOF) {




			n_ring_buffer_pos_current = n_count % n_calc_data_step;

			n_ring_buffer_pos_oldest = n_ring_buffer_pos_current - (n_calc_data_step - 1);
			if (n_ring_buffer_pos_oldest < 0) n_ring_buffer_pos_oldest = n_calc_data_step + n_ring_buffer_pos_oldest;

			



			// load slam trajectory data



			fscanf_s(fp_orb_slam_trajectory, "%lf", &slam_trajectory_d_Twc_X);
			fscanf_s(fp_orb_slam_trajectory, "%lf", &slam_trajectory_d_Twc_Y);
			fscanf_s(fp_orb_slam_trajectory, "%lf", &slam_trajectory_d_Twc_Z);



			for (int j = 0; j < 3; j++) {
				for (int i = 0; i < 3; i++) {

					fscanf_s(fp_orb_slam_trajectory, "%lf", &buffer_load_slam_trajectory);

					mat_slam_R.at<double>(j, i) = buffer_load_slam_trajectory;
				}
			}


			printf("TIME:%.0lf\n", slam_trajectory_d_time);



			//store slam data into vector parameters
			buf_slam_xyz.x = slam_trajectory_d_Twc_X;
			buf_slam_xyz.y = slam_trajectory_d_Twc_Y;
			buf_slam_xyz.z = slam_trajectory_d_Twc_Z;

			slam_xyz.push_back(buf_slam_xyz);





			//calc camera axis

			camera_axis_1n = mat_slam_R * camera_axis_1;
			camera_axis_2n = mat_slam_R * camera_axis_2;
			camera_axis_3n = mat_slam_R * camera_axis_3;









			vehicle_trajectory[n_ring_buffer_pos_current].x = slam_trajectory_d_Twc_X;
			vehicle_trajectory[n_ring_buffer_pos_current].y = slam_trajectory_d_Twc_Y;
			vehicle_trajectory[n_ring_buffer_pos_current].z = slam_trajectory_d_Twc_Z;
			vehicle_trajectory[n_ring_buffer_pos_current].time_stamp = slam_trajectory_d_time;





			//calc moving velocity

			if (n_count < n_calc_data_step) {

				slam_trajectory_d_moving_velocity = 0;
			
			}
			else {
				slam_trajectory_d_moving_velocity = (slam_trajectory_d_Twc_X - vehicle_trajectory[n_ring_buffer_pos_oldest].x)*(slam_trajectory_d_Twc_X - vehicle_trajectory[n_ring_buffer_pos_oldest].x) + (slam_trajectory_d_Twc_Y - vehicle_trajectory[n_ring_buffer_pos_oldest].y)*(slam_trajectory_d_Twc_Y - vehicle_trajectory[n_ring_buffer_pos_oldest].y) + (slam_trajectory_d_Twc_Z - vehicle_trajectory[n_ring_buffer_pos_oldest].z)*(slam_trajectory_d_Twc_Z - vehicle_trajectory[n_ring_buffer_pos_oldest].z);

				slam_trajectory_d_moving_velocity = sqrt(slam_trajectory_d_moving_velocity) / (slam_trajectory_d_time - vehicle_trajectory[n_ring_buffer_pos_oldest].time_stamp) * 1000;


			}

			

			fprintf(fp_w_traveling_velocity, "%lf\t%lf\n", slam_trajectory_d_time, slam_trajectory_d_moving_velocity);


			




			///// ONLY_CALC_TRAVELING_DISTANCE /////


			if ((n_count%n_calc_data_step) == 0) {




				slam_trajectory_d_moving_distance = sqrt((slam_trajectory_d_Twc_X - slam_trajectory_d_prev_Twc_X)*(slam_trajectory_d_Twc_X - slam_trajectory_d_prev_Twc_X) + (slam_trajectory_d_Twc_Y - slam_trajectory_d_prev_Twc_Y)*(slam_trajectory_d_Twc_Y - slam_trajectory_d_prev_Twc_Y) + (slam_trajectory_d_Twc_Z - slam_trajectory_d_prev_Twc_Z)*(slam_trajectory_d_Twc_Z - slam_trajectory_d_prev_Twc_Z));

				slam_trajectory_d_total_traveling_distance += slam_trajectory_d_moving_distance;




				fprintf(fp_w_traveling_distance, "%lf\t%lf\n", slam_trajectory_d_time, slam_trajectory_d_total_traveling_distance);


				slam_trajectory_d_prev_Twc_X = slam_trajectory_d_Twc_X;
				slam_trajectory_d_prev_Twc_Y = slam_trajectory_d_Twc_Y;
				slam_trajectory_d_prev_Twc_Z = slam_trajectory_d_Twc_Z;



			}









			n_count++;

		}



		fclose(fp_w_traveling_distance);
		fclose(fp_w_traveling_velocity);



		cout << "OUTPUT => " << save_file_path_traveling_distance << endl;
		cout << "OUTPUT => " << save_file_path_traveling_velocity << endl;


		return 0;


	}








	////////////////////////////////////////////////////////////////////////////////////////




























	std::cout << "File path: Stereo camera paramters = ";
	cin >> sFilePath_stereo_calibration_parameters;



	sSaveFilePath_stereo_calibration_parameters_yaml = sFilePath_stereo_calibration_parameters + ".yaml";




	FileStorage fs_stereo_param(sFilePath_stereo_calibration_parameters, FileStorage::READ);


	fs_stereo_param["K1"] >> K1;
	fs_stereo_param["K2"] >> K2;
	fs_stereo_param["D1"] >> D1;
	fs_stereo_param["D2"] >> D2;
	fs_stereo_param["R"] >> R;
	fs_stereo_param["T"] >> T;
	fs_stereo_param["E"] >> E;
	fs_stereo_param["F"] >> F;

	fs_stereo_param["R1"] >> R1;
	fs_stereo_param["R2"] >> R2;
	fs_stereo_param["P1"] >> P1;
	fs_stereo_param["P2"] >> P2;
	fs_stereo_param["Q"] >> Q;

	fs_stereo_param.release();

	cout << "The stereo camera parameters have been loaded!\n";




	//convert xml file to yaml file
	FileStorage fs_stereo_param_yaml(sSaveFilePath_stereo_calibration_parameters_yaml, FileStorage::WRITE);


	fs_stereo_param_yaml << "K1" << K1;
	fs_stereo_param_yaml << "K2" << K2;
	fs_stereo_param_yaml << "D1" << D1;
	fs_stereo_param_yaml << "D2" << D2;
	fs_stereo_param_yaml << "R" << R;
	//fs_stereo_param_yaml << "T" << T;
	//fs_stereo_param_yaml << "E" << E;
	//fs_stereo_param_yaml << "F" << F;

	fs_stereo_param_yaml << "R1" << R1;
	fs_stereo_param_yaml << "R2" << R2;
	fs_stereo_param_yaml << "P1" << P1;
	fs_stereo_param_yaml << "P2" << P2;
	fs_stereo_param_yaml << "Q" << Q;


	fs_stereo_param_yaml.release();












	//歪み補正および平行化変換のマップを求めます．
	initUndistortRectifyMap(K1, D1, R1, P1, IM_SIZE, CV_16SC2, map11, map12);
	initUndistortRectifyMap(K2, D2, R2, P2, IM_SIZE, CV_16SC2, map21, map22);



















	if (YOLO_mode != ONLY_MAKE_REMAP_MOVIE) {




		cout << "File path to load parameters, movie file( ****_cam1.mov, ****_cam2.mov ) and threshold for distance, = ";

		cin >> sFilePath_ground_plane_estimation_parameters;









		FileStorage fs_ground_plane_param(sFilePath_ground_plane_estimation_parameters, FileStorage::READ);


		fs_ground_plane_param["sMovieFname"] >> sMovieFname;																			cout << "Movie file name : " << sMovieFname << "_cam1.mov, _cam2.mov" << endl;
		fs_ground_plane_param["n_cloud_HSV_S_upper_thresh"] >> n_cloud_HSV_S_upper_thresh;												cout << "Cloud HSV S thresh : " << n_cloud_HSV_S_upper_thresh << endl;
		fs_ground_plane_param["n_cloud_HSV_V_lower_thresh"] >> n_cloud_HSV_V_lower_thresh;												cout << "Cloud HSV V thresh : " << n_cloud_HSV_V_lower_thresh << endl;
		fs_ground_plane_param["n_non_texure_region_detction_sobel_edge_thresh"] >> n_non_texure_region_detction_sobel_edge_thresh;		cout << "NonTextureRegion Edge thresh : " << n_non_texure_region_detction_sobel_edge_thresh << endl;
		fs_ground_plane_param["d_object_distance_from_camera_thresh_min"] >> d_object_distance_from_camera_thresh_min;					cout << "Object distance min : " << to_string(int(d_object_distance_from_camera_thresh_min)) << endl;
		fs_ground_plane_param["d_object_distance_from_camera_thresh_max"] >> d_object_distance_from_camera_thresh_max;					cout << "Object distance max : " << to_string(int(d_object_distance_from_camera_thresh_max)) << endl;



		if (YOLO_mode == YOLO_UME) {


			fs_ground_plane_param["n_yolo_bb_object_cluster_candidates_num"] >> n_yolo_bb_object_cluster_candidates_num;							cout << "Cluster candidates number in k-means clustering : " << n_yolo_bb_object_cluster_candidates_num << endl;
			fs_ground_plane_param["n_yolo_bb_point_num_magnification_to_do_clustering"] >> n_yolo_bb_point_num_magnification_to_do_clustering;		cout << "Cluster num magnification to do k-means: " << n_yolo_bb_point_num_magnification_to_do_clustering << endl;
			fs_ground_plane_param["n_yolo_bb_object_clustering_k_means_loop_times"] >> n_yolo_bb_object_clustering_k_means_loop_times;				cout << "k-means loop times : " << n_yolo_bb_object_clustering_k_means_loop_times << endl;
			fs_ground_plane_param["n_yolo_bb_object_pixel_num_thresh"] >> n_yolo_bb_object_pixel_num_thresh;										cout << "Thresh for Object pixel number : " << n_yolo_bb_object_pixel_num_thresh << endl;
			fs_ground_plane_param["d_distance_thresh_in_nearest_neighborhood_method"] >> d_distance_thresh_in_nearest_neighborhood_method;			cout << "Thresh to distance in NN (Nearest Neighborhood) clustering: " << d_distance_thresh_in_nearest_neighborhood_method << endl;


			//	fs_ground_plane_param["d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox_thresh"] >> d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox_thresh;		cout << "Cluster pixel ratio threshold : " << std::fixed << std::setprecision(2) << d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox_thresh << endl;	

		}

		if (YOLO_mode == YOLO_OBJECTS) {

			fs_ground_plane_param["n_road_edge_line_distance_estimation_moving_average_method_frame_number"] >> n_road_edge_line_distance_estimation_moving_average_method_frame_number; cout << "Frame number for moving average method to estimate the distance to road edge line :" << n_road_edge_line_distance_estimation_moving_average_method_frame_number << endl;
			fs_ground_plane_param["n_yolo_bb_window_half_size_to_acquire_xyz_coordinates"] >> n_yolo_bb_window_half_size_to_acquire_xyz_coordinates; cout << "Window half size to acquire xyz coordinates in Yolo Bounding Box :" << n_yolo_bb_window_half_size_to_acquire_xyz_coordinates << endl;
			fs_ground_plane_param["n_road_edge_line_distance_estimation_bbox_window_center_position_div_num"] >> n_road_edge_line_distance_estimation_bbox_window_center_position_div_num; cout << "Window position in Yolo Bounding Box 1/? (1/2, 1/3, 1/4) :" << n_road_edge_line_distance_estimation_bbox_window_center_position_div_num << endl;
			fs_ground_plane_param["d_road_edge_line_estimation_distance_resolution"] >> d_road_edge_line_estimation_distance_resolution; cout << "The distance resolution to estimate edge line (0.2, 0.8 etc) :" << d_road_edge_line_estimation_distance_resolution << endl;

			fs_ground_plane_param["d_road_edge_traveling_grid_pitch_x"] >> d_road_edge_traveling_grid_pitch_x; cout << "Traveling GRID x-pitch [m] (ex. 0.1) :" << d_road_edge_traveling_grid_pitch_x << endl;
			fs_ground_plane_param["d_road_edge_traveling_grid_pitch_z"] >> d_road_edge_traveling_grid_pitch_z; cout << "Traveling GRID z-pitch [m] (ex. 0.2) :" << d_road_edge_traveling_grid_pitch_z << endl;
			fs_ground_plane_param["n_road_edge_traveling_grid_num_z"] >> n_road_edge_traveling_grid_num_z; cout << "Traveling GRID num z-pitch [m] (ex. 0.2) :" << n_road_edge_traveling_grid_num_z << endl;

		}





		//	fs_ground_plane_param["n_SLAM_frame_step"] >> n_SLAM_frame_step;			//"SLAM: Frame step to calc SLAM (15) = "
		//	fs_ground_plane_param["n_Keypoints_Num_Max"] >> n_Keypoints_Num_Max;		//"ORB: Max num of keypoints to detect (3000) = "
		//	fs_ground_plane_param["n_Landmark_Keypoints_Num_Max"] >> n_Landmark_Keypoints_Num_Max;
		//	fs_ground_plane_param["d_RANSAC_outlier_thresh"] >> d_RANSAC_outlier_thresh; // "RANSAC Estimate plane  outlier threshold for distance from a plane (50) [mm] = "
		//	fs_ground_plane_param["n_estimate_plane_image_x_range_min"] >> n_estimate_plane_image_x_range_min; //"Estimate plane: Image x range min (0) [pixel] = "
		//	fs_ground_plane_param["n_estimate_plane_image_x_range_max"] >> n_estimate_plane_image_x_range_max; //"Estimate plane: Image x range max (640) [pixel] = "
		//	fs_ground_plane_param["n_estimate_plane_image_y_range_min"] >> n_estimate_plane_image_y_range_min; //"Estimate plane: Image y range min (240) [pixel] = "
		//	fs_ground_plane_param["n_estimate_plane_image_y_range_max"] >> n_estimate_plane_image_y_range_max; //"Estimate plane: Image y range max (480) [pixel] = "
		//	fs_ground_plane_param["n_mesh_width_to_detect_wall"] >> n_mesh_width_to_detect_wall;	//"Estimate plane: Mesh width to detect wall region [pixel] (50) = "
		//	fs_ground_plane_param["n_mesh_height_to_detect_wall"] >> n_mesh_height_to_detect_wall;	//"Estimate plane: Mesh height to detect wall region [pixel] (50) = "
		//	fs_ground_plane_param["d_angle_tolerance_between_y_axis_and_surface_patch_normal"] >> d_angle_tolerance_between_y_axis_and_surface_patch_normal; //"Estimate plane: Tolerance for the angle between y-axis and surface patch normal [degree](30) = "
		//	fs_ground_plane_param["d_object_height_minus_thresh"] >> d_object_height_minus_thresh;	//"Object height: Minus height thresh to detect object (-3000) [mm] = "
		//	fs_ground_plane_param["d_object_height_plus_thresh"] >> d_object_height_plus_thresh;	//"Object height: Plus thresh to detect object (-200) [mm] = "
		//	fs_ground_plane_param["d_ditch_height_low_thresh"] >> d_ditch_height_low_thresh;		//"Ditch height: low thresh to detect object (150) [mm] = "
		//	fs_ground_plane_param["d_ditch_height_high_thresh"] >> d_ditch_height_high_thresh;		//"Ditch height: high thresh to detect object (500) [mm] = "

		//	fs_ground_plane_param["n_obstacle_monitoring_zone_x_min"] >> n_obstacle_monitoring_zone_x_min;		//"Obstacle: Monitoring zone local x min (-5000) [mm] = "
		//	fs_ground_plane_param["n_obstacle_monitoring_zone_x_max"] >> n_obstacle_monitoring_zone_x_max;		//"Obstacle: Monitoring zone local x max (5000) [mm] = "
		//	fs_ground_plane_param["n_obstacle_monitoring_zone_z_min"] >> n_obstacle_monitoring_zone_z_min;		//"Obstacle: Monitoring zone local z min (0) [mm] = "
		//	fs_ground_plane_param["n_obstacle_monitoring_zone_z_max"] >> n_obstacle_monitoring_zone_z_max;		//"Obstacle: Monitoring zone local z max (15000) [mm] = "
		//	fs_ground_plane_param["n_obstacle_monitoring_zone_y_min"] >> n_obstacle_monitoring_zone_y_min;		//"Obstacle: Monitoring zone local y min (-1800) [mm] = "
		//	fs_ground_plane_param["n_obstacle_monitoring_zone_y_max"] >> n_obstacle_monitoring_zone_y_max;		//"Obstacle: Monitoring zone local y max (0) [mm] = "
		//	fs_ground_plane_param["n_obstacle_monitoring_zone_block_x_num"] >> n_obstacle_monitoring_zone_block_x_num; //"Obstacle: monitoring zone x-block num (100) = "
		//	fs_ground_plane_param["n_obstacle_monitoring_zone_block_z_num"] >> n_obstacle_monitoring_zone_block_z_num; //"Obstacle: monitoring zone z-block num (150) = "
		//	fs_ground_plane_param["n_object_mesh_width_to_detect_step"] >> n_object_mesh_width_to_detect_step;		//"Obstacle: Mesh width to detect step region (20) = "
		//	fs_ground_plane_param["n_object_mesh_height_to_detect_step"] >> n_object_mesh_height_to_detect_step;	//"Obstacle: Mesh height to detect step region (20) = "
		//	fs_ground_plane_param["d_object_angle_tolerance_between_mesh_normal_and_ground_normal"] >> d_object_angle_tolerance_between_mesh_normal_and_ground_normal; //"Obstacle: Angle tolerance between mesh normal and ground normal (around 45) = "
		//	fs_ground_plane_param["d_object_angle_tolerance_between_mesh_normal_and_x_axis"] >> d_object_angle_tolerance_between_mesh_normal_and_x_axis; //"Obstacle: Angle tolerance between mesh normal and x-axis (around 80 deg) = "
		//	fs_ground_plane_param["d_danger_zone_x_side_length"] >> d_danger_zone_x_side_length;		// "Danger zone: x side length (500) [mm] = "
		//	fs_ground_plane_param["d_danger_zone_z_forward_length"] >> d_danger_zone_z_forward_length;	//"Danger zone: z forward length (2000) [mm] = "

		//	fs_ground_plane_param["n_lower_thresh_RANSAC_data_number_to_check_credibility_estimated_ground_normal"] >> n_lower_thresh_RANSAC_data_number_to_check_credibility_estimated_ground_normal;
		//	fs_ground_plane_param["n_upper_thresh_wall_region_pixel_number_to_check_credibility_estimated_ground_normal"] >> n_upper_thresh_wall_region_pixel_number_to_check_credibility_estimated_ground_normal;


		//	fs_ground_plane_param["n_window_width_to_extract_non_texture_region"] >> n_window_width_to_extract_non_texture_region; //100
		//	fs_ground_plane_param["n_window_height_to_extract_non_texture_region"] >> n_window_height_to_extract_non_texture_region; // ; 100
		//	fs_ground_plane_param["d_thresh_for_z_coordinate_difference_to_detect_depth_outlier"] >> d_thresh_for_z_coordinate_difference_to_detect_depth_outlier; //1500

		//	fs_ground_plane_param["n_Landmark_matching_frame_step"] >> n_Landmark_matching_frame_step;






		fs_ground_plane_param.release();



		std::cout << endl;

		std::cout << "Parameters have been loaded!\n";













	}
	else {


		std::cout << "movie file (***_cam1.mov, ***_cam2.mov) ex. movie_stereo= ";
		std::cin >> sMovieFname;


	}










	//	n_obstacle_monitoring_zone_range_x = n_obstacle_monitoring_zone_x_max - n_obstacle_monitoring_zone_x_min;
	//	n_obstacle_monitoring_zone_range_z = n_obstacle_monitoring_zone_z_max - n_obstacle_monitoring_zone_z_min;

	//	d_obstacle_monitoring_zone_block_x_width = n_obstacle_monitoring_zone_range_x / (double)n_obstacle_monitoring_zone_block_x_num;
	//	d_obstacle_monitoring_zone_block_z_width = n_obstacle_monitoring_zone_range_z / (double)n_obstacle_monitoring_zone_block_z_num;


















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












	//////////////// stereo ///////////////////////////// 


	// make subfolder for saving stereo results //

	//sFolder_save_stereo_results = sMainFolder;
	sFolder_save_stereo_3D = sMainFolder + "/3D";






	//_mkdir(sFolder_save_stereo_results.c_str());
	_mkdir(sFolder_save_stereo_3D.c_str());

























	/////////////////////////////////////////////////////////////////////////////////////////////////////////







































	/////////////////////////////////////////////////////////////////////////////////////





















	// ONLY MAKING REMAP MOVIE FILE




	if (YOLO_mode == ONLY_MAKE_REMAP_MOVIE) {



		//genarate window
		cv::namedWindow(win_src_cam1, CV_WINDOW_AUTOSIZE);
		cv::namedWindow(win_dst1, CV_WINDOW_AUTOSIZE);





		sSaveFilePath_moive_cam1_remap = sMainFolder + "/out_cam1_remap.mov";

		VideoWriter writer_remp_cam1(sSaveFilePath_moive_cam1_remap, VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, Size(IMG_XSIZE, IMG_YSIZE), true);
		if (!writer_remp_cam1.isOpened()) {
			cout << "error: movie writer remap cam1" << endl;
			return -1;
		}








		while (1) {



			capture1 >> img_cam1;


			key = waitKey(1);

			if (img_cam1.empty() || capture1.get(CV_CAP_PROP_POS_AVI_RATIO) == 1 || key == 'q') { //If EOF, CV_CAP_PROP_POS_AVI_RATIO = 1 


																								  //n_processed_file_number_3D_data = result_thread.get();


				break;

			}





			remap(img_cam1, img_cam1_remap, map11, map12, INTER_LINEAR);
			//remap(img_cam2, img_cam2_remap, map21, map22, INTER_LINEAR);


			cv::imshow(win_src_cam1, img_cam1);
			cv::imshow(win_dst1, img_cam1_remap);






			writer_remp_cam1 << img_cam1_remap;




		}





		writer_remp_cam1.release();



		return 0;


	}





























	/////////////////////////////////////////////////////////////////////////////////////////////////////////







































	FILE *fp_yolo_bb = NULL;
	errno_t err_yolo_bb;


	if (YOLO_mode == YOLO_UME || YOLO_mode == YOLO_OBJECTS) {






		// Yolo bounding box info //






		std::cout << "File path : Yolo Bounding Box Info = ";
		std::cin >> sFilePath_YOLO_BoundingBox;




		err_yolo_bb = fopen_s(&fp_yolo_bb, sFilePath_YOLO_BoundingBox.c_str(), "rt");

		if (err_yolo_bb != 0) {

			cout << "cannot open " << endl;
			cout << sFilePath_YOLO_BoundingBox << endl;
			return -1;

		}










		if (YOLO_mode == YOLO_OBJECTS) {






			string file_path_targeted_yolo_object_label;
			string buf_object_label;
			ifstream read_yolo_obj_label;




			cout << "File path to load the YOLO object labels in estimating the road center line (Road_Local, Road_Main, Road_Intersection, Road_LocalOut, Road_LocalIn) = ";
			cin >> file_path_targeted_yolo_object_label;





			read_yolo_obj_label.open(file_path_targeted_yolo_object_label, std::ios::in);

			if (!read_yolo_obj_label) {

				std::cerr << "cannot open yolo object label file " << endl;
				return -1;
			}











			n_yolo_bb_targeted_object_num = 0;


			//while (!read_yolo_obj_label.eof()) {
			while (std::getline(read_yolo_obj_label, buf_object_label)) {

				//std::getline(read_yolo_obj_label, buf_object_label);


				sprintf_s(&yolo_bounding_box_targeted_object_name_list[n_yolo_bb_targeted_object_num][0], YOLO_LABEL_STRING_MAX_LENGTH, "%s", buf_object_label.c_str());


				cout << &yolo_bounding_box_targeted_object_name_list[n_yolo_bb_targeted_object_num][0] << endl;


				n_yolo_bb_targeted_object_num++;


			}


			std::cout << "The number of the loadedlabels : " << to_string(n_yolo_bb_targeted_object_num) << endl;

















			//yolo_bounding_box_label_name_of_intersection
			std::cout << "The label name for single targeted object = ";
			scanf_s("%s", yolo_bounding_box_label_name_of_road_edge, YOLO_LABEL_STRING_MAX_LENGTH);


			s_yolo_object_name_road_edge = yolo_bounding_box_label_name_of_road_edge;








		}







	}






































	//set save file path










	//sSaveFilePath_moive_stereo = sMainFolder + "/ObstacleDitch" + to_string(n_From_movie_frame) + "stp" + to_string(n_SLAM_frame_step) + "ORB" + to_string(n_Keypoints_Num_Max) + "WallW" + to_string(n_mesh_width_to_detect_wall) + "H" + to_string(n_mesh_height_to_detect_wall) + "RansacTh" + to_string((int)d_RANSAC_outlier_thresh) + "X" + to_string(n_estimate_plane_image_x_range_min) + "_" + to_string(n_estimate_plane_image_x_range_max) + "Y" + to_string(n_estimate_plane_image_y_range_min) + "_" + to_string(n_estimate_plane_image_y_range_max) + "ObjTh" + to_string(int(d_object_height_minus_thresh)) + "_" + to_string(int(d_object_height_plus_thresh)) + "ditch" + to_string(int(d_ditch_height_low_thresh)) + "_" + to_string(int(d_ditch_height_high_thresh)) + "DistTh" + to_string(int(d_object_distance_from_camera_thresh_min)) + "to" + to_string(int(d_object_distance_from_camera_thresh_max)) + "CloudHSV_S" + to_string(CLOUD_REGION_HSV_S_UPPER_THRESH) + "V" + to_string(CLOUD_REGION_HSV_V_LOWER_THRESH) + "MeshW" + to_string(n_object_mesh_width_to_detect_step) + "H" + to_string(n_object_mesh_height_to_detect_step) + "Toler" + to_string((int)d_object_angle_tolerance_between_mesh_normal_and_ground_normal) + "_" + to_string((int)d_object_angle_tolerance_between_mesh_normal_and_x_axis) + ".mov";
	sSaveFilePath_moive_stereo = sMainFolder + "/obstacle.mov";



	VideoWriter writer(sSaveFilePath_moive_stereo, VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, Size(IMG_XSIZE * 2, IMG_YSIZE * 2), true);
	if (!writer.isOpened()) {
		cout << "error: movie writer" << sSaveFilePath_moive_stereo << endl;
		return -1;
	}






	//set movie 2

	//sSaveFilePath_moive_mask_sub_image = sMainFolder + "/MaskImages_start" + to_string(n_From_movie_frame) + "WallW" + to_string(n_mesh_width_to_detect_wall) + "H" + to_string(n_mesh_height_to_detect_wall) + "CloudHSV_S" + to_string(CLOUD_REGION_HSV_S_UPPER_THRESH) + "V" + to_string(CLOUD_REGION_HSV_V_LOWER_THRESH) + ".mov";
	sSaveFilePath_moive_mask_sub_image = sMainFolder + "/Mask" + "CloudHSV_S" + to_string(n_cloud_HSV_S_upper_thresh) + "V" + to_string(n_cloud_HSV_V_lower_thresh) + ".mov";

	VideoWriter writer2(sSaveFilePath_moive_mask_sub_image, VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, Size(IMG_XSIZE * 2, IMG_YSIZE * 2), true);
	if (!writer2.isOpened()) {
		cout << "error: movie writer2" << endl;
		return -1;
	}














	FILE *fp_w_orb_slam_object = NULL;
	FILE *fp_w_yolo_bounding_box_object_k_means = NULL;
	FILE *fp_w_yolo_detected_object_NN = NULL;
	FILE *fp_w_yolo_bounding_box_object_of_interest = NULL;
	FILE *fp_w_yolo_bounding_box_object_road_edge_line = NULL;
	FILE *fp_w_grids_between_left_edge_and_right_edge = NULL;
	FILE *fp_w_daijkstra_blocks = NULL;
	FILE *fp_w_daijkstra_vehicle_pos = NULL;






	if (YOLO_mode == YOLO_UME || YOLO_mode == YOLO_OBJECTS) {


		//	stringstream  s_BBox_pixel_ratio_thresh;
		//	s_BBox_pixel_ratio_thresh << std::fixed << std::setprecision(2) << d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox_thresh;


		sSaveFilePath_SLAM_mapping_objects = sMainFolder + "/" + "output_ORB-SLAM_3DMaping_DistTh" + to_string((int)d_object_distance_from_camera_thresh_min) + "_" + to_string((int)d_object_distance_from_camera_thresh_max) + "Edge" + to_string(n_non_texure_region_detction_sobel_edge_thresh) + "HSV_S" + to_string(n_cloud_HSV_S_upper_thresh) + "V" + to_string(n_cloud_HSV_V_lower_thresh) + ".txt";
		//sSaveFilePath_yolo_bounding_box_objects = sMainFolder + "/" + "output_UseYoloBBoxObjectCenterDistTh" + to_string((int)d_object_distance_from_camera_thresh_min) + "_" + to_string((int)d_object_distance_from_camera_thresh_max) + +"Edge" + to_string(n_non_texure_region_detction_sobel_edge_thresh) + "S" + to_string(n_cloud_HSV_S_upper_thresh) + "V" + to_string(n_cloud_HSV_V_lower_thresh) + "Clstr" + "K" + to_string(n_yolo_bb_object_cluster_candidates_num) + "T" + to_string(n_yolo_bb_object_clustering_k_means_loop_times) + "Ratio" + s_BBox_pixel_ratio_thresh.str() + ".txt";

		sSaveFilePath_yolo_bounding_box_objects = sMainFolder + "/" + "output_k_means_based_YOLO_ObjectCenter_DistTh" + to_string((int)d_object_distance_from_camera_thresh_min) + "_" + to_string((int)d_object_distance_from_camera_thresh_max) + "Edge" + to_string(n_non_texure_region_detction_sobel_edge_thresh) + "S" + to_string(n_cloud_HSV_S_upper_thresh) + "V" + to_string(n_cloud_HSV_V_lower_thresh) + "Clstr" + "K" + to_string(n_yolo_bb_object_cluster_candidates_num) + "T" + to_string(n_yolo_bb_object_clustering_k_means_loop_times) + "PixelThresh" + to_string(n_yolo_bb_object_pixel_num_thresh) + ".txt";

		fopen_s(&fp_w_yolo_bounding_box_object_k_means, sSaveFilePath_yolo_bounding_box_objects.c_str(), "wt");










		if (YOLO_mode == YOLO_UME) {

			sSaveFilePath_yolo_detected_objects_use_NN_clustering = sMainFolder + "/" + "output_NN_based_YOLO_Objects_detection_DistTh" + to_string((int)d_object_distance_from_camera_thresh_min) + "_" + to_string((int)d_object_distance_from_camera_thresh_max) + "Edge" + to_string(n_non_texure_region_detction_sobel_edge_thresh) + "S" + to_string(n_cloud_HSV_S_upper_thresh) + "V" + to_string(n_cloud_HSV_V_lower_thresh) + "Clstr" + "K" + to_string(n_yolo_bb_object_cluster_candidates_num) + "T" + to_string(n_yolo_bb_object_clustering_k_means_loop_times) + "PixelThresh" + to_string(n_yolo_bb_object_pixel_num_thresh) + "NN" + _Floating_to_string("%.1lf", d_distance_thresh_in_nearest_neighborhood_method) + ".txt";
			fopen_s(&fp_w_yolo_detected_object_NN, sSaveFilePath_yolo_detected_objects_use_NN_clustering.c_str(), "wt");

		}


		if (YOLO_mode == YOLO_OBJECTS) {


			sSaveFilePath_yolo_bounding_box_objects_of_interest = sMainFolder + "/" + "output_YOLO_Object_of_interest_" + s_yolo_object_name_road_edge + "_DistTh" + to_string((int)d_object_distance_from_camera_thresh_min) + "_" + to_string((int)d_object_distance_from_camera_thresh_max) + "Edge" + to_string(n_non_texure_region_detction_sobel_edge_thresh) + "S" + to_string(n_cloud_HSV_S_upper_thresh) + "V" + to_string(n_cloud_HSV_V_lower_thresh) + ".txt";
			fopen_s(&fp_w_yolo_bounding_box_object_of_interest, sSaveFilePath_yolo_bounding_box_objects_of_interest.c_str(), "wt");

			fprintf_s(fp_w_yolo_bounding_box_object_of_interest, "#n_current_frame_No\t|\tcx\tcy\tcz\tbox_height\tbox_width\tdistance_from_cam\tpoint_num\t|\t->\n");


			sSaveFilePath_yolo_bounding_box_road_edge_line = sMainFolder + "/" + "output_YOLO_Object_road_edge_line_distance_W" + to_string(n_yolo_bb_window_half_size_to_acquire_xyz_coordinates * 2) + "MAframe" + to_string(n_road_edge_line_distance_estimation_moving_average_method_frame_number) + "BBoxWndPos" + to_string(n_road_edge_line_distance_estimation_bbox_window_center_position_div_num) + "th" + "DistDivPitch" + _Floating_to_string("%.1lf", d_road_edge_line_estimation_distance_resolution) + ".txt";
			fopen_s(&fp_w_yolo_bounding_box_object_road_edge_line, sSaveFilePath_yolo_bounding_box_road_edge_line.c_str(), "wt");


			sSaveFilePath_grids_between_left_edge_and_right_edge = sMainFolder + "/" + "output_grids_between_left_edge_and_right_edge_W" + to_string(n_yolo_bb_window_half_size_to_acquire_xyz_coordinates * 2) + "MAframe" + to_string(n_road_edge_line_distance_estimation_moving_average_method_frame_number) + "BBoxWndPos" + to_string(n_road_edge_line_distance_estimation_bbox_window_center_position_div_num) + "th" + "DistDivPitch" + _Floating_to_string("%.1lf", d_road_edge_line_estimation_distance_resolution) + "PitchX" + _Floating_to_string("%.1lf", d_road_edge_traveling_grid_pitch_x) + "Z" + _Floating_to_string("%.1lf", d_road_edge_traveling_grid_pitch_z) + "x" + to_string(n_road_edge_traveling_grid_num_z) + ".txt";
			fopen_s(&fp_w_grids_between_left_edge_and_right_edge, sSaveFilePath_grids_between_left_edge_and_right_edge.c_str(), "wt");



			sSaveFilePath_Daijkstra_blocks = sMainFolder + "/" + "output_Daijkstra_blocks_between_left_edge_and_right_edge_W" + to_string(n_yolo_bb_window_half_size_to_acquire_xyz_coordinates * 2) + "MAframe" + to_string(n_road_edge_line_distance_estimation_moving_average_method_frame_number) + "BBoxWndPos" + to_string(n_road_edge_line_distance_estimation_bbox_window_center_position_div_num) + "th" + "DistDivPitch" + _Floating_to_string("%.1lf", d_road_edge_line_estimation_distance_resolution) + "PitchX" + _Floating_to_string("%.1lf", d_road_edge_traveling_grid_pitch_x) + "Z" + _Floating_to_string("%.1lf", d_road_edge_traveling_grid_pitch_z) + "x" + to_string(n_road_edge_traveling_grid_num_z) + ".txt";
			fopen_s(&fp_w_daijkstra_blocks, sSaveFilePath_Daijkstra_blocks.c_str(), "wt");



			sSaveFilePath_Daijkstra_VehiclePosition = sMainFolder + "/" + "output_Daijkstra_vehicle_position_block_no_W" + to_string(n_yolo_bb_window_half_size_to_acquire_xyz_coordinates * 2) + "MAframe" + to_string(n_road_edge_line_distance_estimation_moving_average_method_frame_number) + "BBoxWndPos" + to_string(n_road_edge_line_distance_estimation_bbox_window_center_position_div_num) + "th" + "DistDivPitch" + _Floating_to_string("%.1lf", d_road_edge_line_estimation_distance_resolution) + "PitchX" + _Floating_to_string("%.1lf", d_road_edge_traveling_grid_pitch_x) + "Z" + _Floating_to_string("%.1lf", d_road_edge_traveling_grid_pitch_z) + "x" + to_string(n_road_edge_traveling_grid_num_z) + ".txt";
			fopen_s(&fp_w_daijkstra_vehicle_pos, sSaveFilePath_Daijkstra_VehiclePosition.c_str(), "wt");


			sSaveFilePath_Daijkstra_field_map_image = sMainFolder + "/" + "output_Daijkstra_field_map.png";


		}



	}
	else {


		sSaveFilePath_SLAM_mapping_objects = sMainFolder + "/" + "output_ORB-SLAM_3DMaping_DistTh" + to_string((int)d_object_distance_from_camera_thresh_min) + "_" + to_string((int)d_object_distance_from_camera_thresh_max) + +"Edge" + to_string(n_non_texure_region_detction_sobel_edge_thresh) + "HSV_S" + to_string(n_cloud_HSV_S_upper_thresh) + "V" + to_string(n_cloud_HSV_V_lower_thresh) + ".txt";


	}



	fopen_s(&fp_w_orb_slam_object, sSaveFilePath_SLAM_mapping_objects.c_str(), "wt"); // X Y Z R G B




















																					  //HSV S_thresh V_thresh




	n_cloud_HSV_S_upper_thresh = int(255.0 * n_cloud_HSV_S_upper_thresh / 100.0); //0-100 => 0-255
	n_cloud_HSV_V_lower_thresh = int(255.0 * n_cloud_HSV_V_lower_thresh / 100.0); //0-100 => 0-255


























																				  /// stereo ///









	int ndisparities;
	int SADWindowSize;
	double minVal, maxVal;






#ifdef STEREO_BM	
	//StereoBM//
	ndisparities = STEREO_BLOCK_MATCHING_DISPARITIES_SIZE;//16 * 5;
	SADWindowSize = STEREO_BLOCK_MATCHING_SADWINDOW_SIZE;
	Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);
#endif







#ifdef STEREO_SGBM
	
	
	//StereoSGBM//
	//ndisparities = STEREO_BLOCK_MATCHING_DISPARITIES_SIZE;// 16 * 10;// 16 * 5;
	//SADWindowSize = STEREO_BLOCK_MATCHING_SADWINDOW_SIZE;//21;//17
	//
	//Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, ndisparities, SADWindowSize);
	//sgbm->setSpeckleWindowSize(STEREO_BLOCK_MATCHING_SPECKLE_WINDOW_SIZE);
	//sgbm->setSpeckleRange(STEREO_BLOCK_MATCHING_SPECKLE_RANGE);







	// StereoSGBM with Weighted Least Squares filter //

	int sgbm_minDisparity = 0;
	int sgbm_numDisparities = 16 * 5;
	int sgbm_window_size = 3;
	int sgbm_P1 = 8 * 3 * sgbm_window_size*sgbm_window_size;
	int sgbm_P2 = 32 * 3 * sgbm_window_size*sgbm_window_size;
	int sgbm_disp12MaxDiff = 16 * 5;
	int sgbm_preFilterCap = 63;
	int sgbm_uniquenessRatio = 15;
	int sgbm_speckleWindowSize = 200;
	int sgbm_speckleRange = 1;
	double wls_lmbda = 80000.0;
	double wls_sigma = 1.5;




	Mat left_for_matcher, right_for_matcher;
	Mat left_disp, right_disp;
	Mat filtered_disp;
	Mat raw_disp_vis;
	Mat filtered_disp_vis;
	Mat image_filterd_disp = Mat(IMG_YSIZE, IMG_XSIZE, CV_8UC3);
	double vis_mult = 1.0;





	Ptr<DisparityWLSFilter> wls_filter;

	Ptr<StereoSGBM> left_matcher = StereoSGBM::create(sgbm_minDisparity, sgbm_numDisparities, sgbm_window_size);




	left_matcher->setP1(sgbm_P1);
	left_matcher->setP2(sgbm_P2);
	left_matcher->setPreFilterCap(sgbm_preFilterCap);
	left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);

	wls_filter = createDisparityWLSFilter(left_matcher);




	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);



#endif

















	// ORB detector //







	vector<cv::KeyPoint > keypoints0, keypoints1;
	vector<cv::KeyPoint>::iterator itk0, itk1;
	Point point0, point1;
	Mat	descriptors0, descriptors1;




	vector<cv::KeyPoint > keypoints_t0, keypoints_t_1, keypoints_t_2, keypoints_t_3;
	Mat descriptors_t0, descriptors_t_1, descriptors_t_2, descriptors_t_3; //minus 1, minus 2

	std::vector<cv::Point2f> match_point_t0;
	std::vector<cv::Point2f> match_point_t_1;
	std::vector<cv::Point2f> match_point_t_2;
	std::vector<cv::Point2f> match_point_t_3;
	std::vector<int> match_keypointNoLink_t0_0;
	std::vector<int> match_keypointNoLink_t0_1;
	std::vector<int> match_keypointNoLink_t_1_1;
	std::vector<int> match_keypointNoLink_t_1_2;
	std::vector<int> match_keypointNoLink_t_2_2;
	std::vector<int> match_keypointNoLink_t_2_3;







	vector<cv::KeyPoint > keypoints1_landmark;
	Mat descriptors1_landmark;







	//Ptr<ORB> detector_ORB = ORB::create(n_Keypoints_Num_Max, 2.25f, 4, 7, 0, 2, 0, 31);
	//cv::BFMatcher matcher(NORM_HAMMING); //NORM_HAMMING


	//Ptr<ORB> detector_ORB = ORB::create(n_Keypoints_Num_Max, 2.25f, 4, 7, 0, 4, 0, 31);
	//	Ptr<ORB> detector_ORB = ORB::create(n_Keypoints_Num_Max, 1.2f, 8, 7, 0, 4, 0, 31);
	//Ptr<ORB> detector_ORB_Landmark = ORB::create(n_Landmark_Keypoints_Num_Max, 2.25f, 4, 7, 0, 4, 0, 31);
	//	Ptr<ORB> detector_ORB_Landmark = ORB::create(n_Landmark_Keypoints_Num_Max, 1.2f, 8, 7, 0, 4, 0, 31);

	//	cv::BFMatcher matcher(NORM_HAMMING2); //NORM_HAMMING2 : when WTA-K = 4









	//stitching
	//Stitcher stt = Stitcher::createDefault();

















	dwTime0 = timeGetTime();








	// define window //



	cv::namedWindow(win_src_cam1, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(win_src_cam2, CV_WINDOW_AUTOSIZE);


	cv::namedWindow(win_dst1, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(win_dst2, CV_WINDOW_AUTOSIZE);

	cv::namedWindow(win_dst0, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(win_dst_mask, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(win_dst_wall_mask, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(win_dst_non_texture_region_mask, CV_WINDOW_AUTOSIZE);



















	//////////////////////////////////////////////
	////////////// Main processing ///////////////
	//////////////////////////////////////////////









	int n_flag_first_count_to_estimate_distance_to_left_road_edge_line;
	int n_flag_first_count_to_estimate_distance_to_right_road_edge_line;




	//yolo bounding box parameters

	vector<struct yolo_bounding_box> yolo_BBox;
	struct yolo_bounding_box yolo_buf_bbox;








	int yolo_BBox_LTx;
	int yolo_BBox_LTy;
	int yolo_BBox_RBx;
	int yolo_BBox_RBy;

	int n_ootsu_threshold_in_boudning_box;

	int n_yolo_bb_file_stream_size;

	int yolo_n_flag_loop_exit;


	int n_yolo_bb_pixel_num_in_Box;
	double d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox;

	double d_point_distance;


	double d_distance_in_nearest_neighborhood_method;
	double d_distance_min_in_nearest_neighborhood_method;

	int n_distance_min_cluster_no_in_nearest_neighborhood_method;






	Mat matrix_R_cam = Mat::eye(3, 3, CV_64FC1);
	Mat matrix_R_cam_accumulated = Mat::eye(3, 3, CV_64FC1);
	Mat matrix_RT_cam_accumulated = Mat::eye(4, 4, CV_64FC1);


	Mat PnP_rvec;
	Mat PnP_tvec;
	Mat PnP_rvec2;
	Mat PnP_tvec2;



	axis_dim4_1n.at<double>(0, 0) = 1;
	axis_dim4_1n.at<double>(1, 0) = 0;
	axis_dim4_1n.at<double>(2, 0) = 0;
	axis_dim4_1n.at<double>(3, 0) = 1;

	axis_dim4_2n.at<double>(0, 0) = 0;
	axis_dim4_2n.at<double>(1, 0) = 1;
	axis_dim4_2n.at<double>(2, 0) = 0;
	axis_dim4_2n.at<double>(3, 0) = 1;

	axis_dim4_3n.at<double>(0, 0) = 0;
	axis_dim4_3n.at<double>(1, 0) = 0;
	axis_dim4_3n.at<double>(2, 0) = 1;
	axis_dim4_3n.at<double>(3, 0) = 1;

	axis_dim4_origin_n.at<double>(0, 0) = 0;
	axis_dim4_origin_n.at<double>(1, 0) = 0;
	axis_dim4_origin_n.at<double>(2, 0) = 0;
	axis_dim4_origin_n.at<double>(3, 0) = 1;










	//yolo
	yolo_n_flag_loop_exit = -1;


	//set the frame counter
	n_frame_count = 0;
	n_current_frame_No = 0;



	n_frame_count_3D_data = 0;
	n_frame_count_keypoints_XYZ = 0;



	n_flag_first_count_to_estimate_distance_to_left_road_edge_line = -1;
	n_flag_first_count_to_estimate_distance_to_right_road_edge_line = -1;







	// begin gnuplot thread //


	n_flag_thread_gnuplot_exit = 0;


	hThread_gnuplot = (HANDLE)_beginthreadex(NULL, 0, thread_gnuplot, "gnuplot", 0, NULL); //(https://docs.microsoft.com/ja-jp/cpp/c-runtime-library/reference/beginthread-beginthreadex?view=msvc-160)















	int n_buffer_size_for_stereo_vision_based_moving_distance = 7;
	int n_ring_buffer_position_moving_distance = 0;

	std::vector<double> RING_BUFFER_moving_distance(n_buffer_size_for_stereo_vision_based_moving_distance);
	for (size_t i = 0; i < n_buffer_size_for_stereo_vision_based_moving_distance; i++) {

		RING_BUFFER_moving_distance[i] = 0;

	}





	d_angle_between_ground_normal_and_y_axis = 90;


	n_flag_done_estimation_default_ground_normal_vector = 0;











	//kalman filter initialization
	Kalman_initialize(&kalman_param_distance);







	//slam traveling parameters

	slam_trajectory_d_prev_Twc_X = slam_trajectory_d_prev_Twc_Y = slam_trajectory_d_prev_Twc_Z = 0;
	slam_trajectory_d_total_traveling_distance = 0;






	// init daijkstra array

	unsigned int n_daijkstra_field_map_node_no = 1;

	int n_daijkstra_block_no_x;
	int n_daijkstra_block_no_z;
	double d_daijkstra_block_pos_x;
	double d_daijkstra_block_pos_z;
	double d_daijkstra_block_pos_y;

	double d_daijkstra_vehicle_self_block_pos_x;
	double d_daijkstra_vehicle_self_block_pos_y;
	double d_daijkstra_vehicle_self_block_pos_z;
	int n_daijkstra_vehicle_self_block_no_x;
	int n_daijkstra_vehicle_self_block_no_z;
	unsigned int n_daijkstra_vehicle_self_pos_block_no;
	unsigned int n_daijkstra_vehicle_self_pos_block_no_previous = 0;


	for (int j = 0; j < DAIJKSTRA_MAP_Z_BLOCK_NUM; j++) {
		for (int i = 0; i < DAIJKSTRA_MAP_X_BLOCK_NUM; i++) {

			daijkstra_field_MAP[j][i] = 0;

		}
	}




	n_daijkstra_vehicle_self_pos_block_no = 1;





	/// Main loop ///









	while (1) {














		//descriptors1.release();
		//Descriptor1.release();
		//keypoints1.clear();  keypoints1.shrink_to_fit();
		//Keypoints_1.clear(); Keypoints_1.shrink_to_fit();





		//n_flag_done_good_matching_keypoints = 0;
		//n_flag_done_calc_homography_matrix = 0;
		//n_flag_done_calc_essential_matrix = 0;














		sFrameNo = to_string(n_current_frame_No);













		//initialized the vote number to detect obstacles
		//		n_voted_num_to_detect_obstacles = 0;
		//		n_voted_num_to_detect_ditch = 0;
















		//		int	px, py;

		Mat matrix_T(3, 1, CV_64F);
		Mat matrix_THETA(3, 1, CV_64F);

		Mat	img_3D_plane(IMG_YSIZE, IMG_XSIZE, CV_8UC3);

















		std::cout << "[" << n_current_frame_No << "]" << endl;


		capture1 >> img_cam1;
		capture2 >> img_cam2;











		key = waitKey(1);

		if (img_cam1.empty() || capture1.get(CV_CAP_PROP_POS_AVI_RATIO) == 1 || key == 'q') { //If EOF, CV_CAP_PROP_POS_AVI_RATIO = 1 


																							  //n_processed_file_number_3D_data = result_thread.get();
			n_flag_thread_gnuplot_exit = 1;

			break;

		}










		///////////// SLAM INFO ///////////////////

		// load slam trajectory data

		fscanf_s(fp_orb_slam_trajectory, "%lf", &slam_trajectory_d_time);

		fscanf_s(fp_orb_slam_trajectory, "%lf", &slam_trajectory_d_Twc_X);
		fscanf_s(fp_orb_slam_trajectory, "%lf", &slam_trajectory_d_Twc_Y);
		fscanf_s(fp_orb_slam_trajectory, "%lf", &slam_trajectory_d_Twc_Z);



		for (int j = 0; j < 3; j++) {
			for (int i = 0; i < 3; i++) {

				fscanf_s(fp_orb_slam_trajectory, "%lf", &buffer_load_slam_trajectory);

				mat_slam_R.at<double>(j, i) = buffer_load_slam_trajectory;
			}
		}


		printf("TIME:%.0lf\n", slam_trajectory_d_time);



		//store slam data into vector parameters
		buf_slam_xyz.x = slam_trajectory_d_Twc_X;
		buf_slam_xyz.y = slam_trajectory_d_Twc_Y;
		buf_slam_xyz.z = slam_trajectory_d_Twc_Z;
		buf_slam_xyz.map_block_no_x = 0;
		buf_slam_xyz.map_block_no_z = 0;
		buf_slam_xyz.map_node_no = 0;


		slam_xyz.push_back(buf_slam_xyz);





		//calc camera axis

		camera_axis_1n = mat_slam_R * camera_axis_1;
		camera_axis_2n = mat_slam_R * camera_axis_2;
		camera_axis_3n = mat_slam_R * camera_axis_3;












		///// CALC_TRAVELING_DISTANCE /////


		if ((n_frame_count%n_frame_step_saving_traveling_distance) == 0) {




			slam_trajectory_d_moving_distance = sqrt((slam_trajectory_d_Twc_X - slam_trajectory_d_prev_Twc_X)*(slam_trajectory_d_Twc_X - slam_trajectory_d_prev_Twc_X) + (slam_trajectory_d_Twc_Y - slam_trajectory_d_prev_Twc_Y)*(slam_trajectory_d_Twc_Y - slam_trajectory_d_prev_Twc_Y) + (slam_trajectory_d_Twc_Z - slam_trajectory_d_prev_Twc_Z)*(slam_trajectory_d_Twc_Z - slam_trajectory_d_prev_Twc_Z));

			slam_trajectory_d_total_traveling_distance += slam_trajectory_d_moving_distance;




			fprintf(fp_w_traveling_distance, "%lf\t%lf\n", slam_trajectory_d_time, slam_trajectory_d_total_traveling_distance);




			slam_trajectory_d_prev_Twc_X = slam_trajectory_d_Twc_X;
			slam_trajectory_d_prev_Twc_Y = slam_trajectory_d_Twc_Y;
			slam_trajectory_d_prev_Twc_Z = slam_trajectory_d_Twc_Z;





		}





















		//////////////// LOAD YOLO info of current frame ///////////////////



		if (YOLO_mode == YOLO_UME || YOLO_mode == YOLO_OBJECTS) {







			while (1) { //for each frame





				n_yolo_bb_file_stream_size = 0;


				if (fgets(yolo_bb_file_BUFFER, YOLO_FILE_LINE_MAX_BUFFER_SIZE, fp_yolo_bb) != NULL) {


					n_yolo_bb_file_stream_size = (int)strlen(yolo_bb_file_BUFFER);


					if (n_yolo_bb_file_stream_size == 2) {

						break;
					}
					else {

						sscanf_s(yolo_bb_file_BUFFER, "%s\t%lf\t%d\t%d\t%d\t%d", &yolo_buf_bbox.name, YOLO_LABEL_STRING_MAX_LENGTH, &yolo_buf_bbox.likelihood, &yolo_buf_bbox.x, &yolo_buf_bbox.y, &yolo_buf_bbox.width, &yolo_buf_bbox.height);

						yolo_BBox.push_back(yolo_buf_bbox);


						printf_s("%s\t%d\t%d\t%d\t%d\n", yolo_buf_bbox.name, yolo_buf_bbox.x, yolo_buf_bbox.y, yolo_buf_bbox.width, yolo_buf_bbox.height);



					}


				}
				else {

					printf_s("Detected the end of file!!\n");


					yolo_n_flag_loop_exit = 1;
					break;

				}








			}//while for each frame









		}//if mode














		 //make mask image for original image region //

		img_mask_no_image_region = Scalar(255);

		remap(img_mask_no_image_region, img_mask_no_image_region_remap, map11, map12, INTER_LINEAR);













		// detect cloud region using canny edge detector //


		cvtColor(img_cam1, img_wrk1, CV_BGR2GRAY);
		//cvtColor(img_cam2, img_wrk2, CV_BGR2GRAY);
		Canny(img_wrk1, img_edge1, n_sky_detction_canny_edge_thresh_low, n_sky_detction_canny_edge_thresh_high);
		//Canny(img_wrk2, img_edge2, 50, 200);





		img_mask_cloud1 = Scalar(0);
		//img_mask_cloud2 = Scalar(0);


		for (auto i = 0; i < IMG_XSIZE; i++) {
			for (auto j = 0; j < IMG_YSIZE; j++) {

				img_mask_cloud1.at<uchar>(j, i) = 255;

				if (img_edge1.at<uchar>(j, i) == 255) break;


			}
		}



		remap(img_mask_cloud1, img_mask_cloud1_remap, map11, map12, INTER_LINEAR);


		cvtColor(img_mask_cloud1_remap, img_cloud_remap, CV_GRAY2BGR);


		cv::imshow(win_dst_mask, img_cloud_remap);




















		//detect the non-texture region using edge strength
		Sobel(img_wrk1, edge1_sobel_dx, CV_32F, 1, 0);
		Sobel(img_wrk1, edge1_sobel_dy, CV_32F, 0, 1);

		magnitude(edge1_sobel_dx, edge1_sobel_dy, edge1_sobel);
		threshold(edge1_sobel, img_temp, n_non_texure_region_detction_sobel_edge_thresh, 255, THRESH_BINARY_INV); //after thresholding, inverse the image

		convertScaleAbs(img_temp, img_mask_non_texture_region); // => 8bit image




		remap(img_mask_non_texture_region, img_mask_non_texture_region_remap, map11, map12, INTER_LINEAR); //mask image


																										   //cvtColor(img_mask_non_texture_region_remap, img_edge1_sobel_remap, CV_GRAY2BGR);// => 24bit image
																										   //cv::imshow(win_dst_non_texture_region_mask, img_edge1_sobel_remap);
















		remap(img_cam1, img_cam1_remap, map11, map12, INTER_LINEAR);
		remap(img_cam2, img_cam2_remap, map21, map22, INTER_LINEAR);




		cv::imshow(win_dst1, img_cam1_remap);
		cv::imshow(win_dst2, img_cam2_remap);




		cvtColor(img_cam1_remap, img_gray1, CV_BGR2GRAY);
		cvtColor(img_cam2_remap, img_gray2, CV_BGR2GRAY);








		//extract HSV
		cvtColor(img_cam1_remap, img_hsv, CV_BGR2HSV);











		//extract only the cloud region ,not extracting the shadow region

		for (int y = 0; y < xyz.rows; y++) {
			for (int x = 0; x < xyz.cols; x++) {





				//shadow region

				//if (img_mask_non_texture_region_remap.at<uchar>(y, x) == 255 && img_hsv.at<Vec3b>(y, x)[1] >= n_cloud_HSV_S_upper_thresh && img_hsv.at<Vec3b>(y, x)[2] <= n_cloud_HSV_V_lower_thresh){
				if (img_mask_non_texture_region_remap.at<uchar>(y, x) == 255 && img_hsv.at<Vec3b>(y, x)[2] <= n_cloud_HSV_V_lower_thresh) {

					img_mask_non_texture_region_remap.at<uchar>(y, x) = 0; //shadow region

				}





			}
		}




		cvtColor(img_mask_non_texture_region_remap, img_edge1_sobel_remap, CV_GRAY2BGR);// => 24bit image

		cv::imshow(win_dst_non_texture_region_mask, img_edge1_sobel_remap);





























		/////////////////////////////////////////////////////////////////////////////////
		//// STEREO VISION 
		/////////////////////////////////////////////////////////////////////////////////

























		//xyz.release();



		//calc disparity
#ifdef STEREO_BM
		sbm->compute(img_gray1, img_gray2, img_disparity16S);


		minMaxLoc(img_disparity16S, &minVal, &maxVal);
		//cout << "Disp: Min" << minVal << "Max" << maxVal << endl;


		img_disparity16S.convertTo(img_disparity8U, CV_8UC1, 255 / (maxVal - minVal));




		//calc 3D coordinates
		reprojectImageTo3D(img_disparity16S, xyz, Q, true);


		//gray to BGR
		cvtColor(img_disparity8U, img_merge, CV_GRAY2BGR);



		cv::putText(img_merge, sFrameNo, cv::Point(10, 450), cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 255, 0), 1, CV_AA);




		namedWindow(win_Disparity, CV_WINDOW_AUTOSIZE);
		//imshow(win_Disparity, img_disparity8U);
		imshow(win_Disparity, img_merge);


#endif

#ifdef STEREO_SGBM
		//sgbm->compute(img_gray1, img_gray2, img_disparity16S);




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



		// gray to BGR //
		cvtColor(filtered_disp_vis, img_merge, CV_GRAY2BGR);
		
		cv::putText(img_merge, sFrameNo, cv::Point(10, 450), cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 255, 0), 1, CV_AA);




		namedWindow(win_Disparity, CV_WINDOW_AUTOSIZE);
		
		imshow(win_Disparity, img_merge);

		



		








		// calc 3D corrdinates //


		reprojectImageTo3D(filtered_disp, xyz, Q, true);






#endif


		














		//Check depth value




		for (int y = 0; y < xyz.rows; y++) {
			for (int x = 0; x < xyz.cols; x++) {




				xyz.at<Vec3f>(y, x) *= 16.0;





				Vec3f point_xyz = xyz.at<Vec3f>(y, x);





				if (fabs(point_xyz[2] - STEREO_Z_DEPTH_MAX) < FLT_EPSILON || fabs(point_xyz[2]) > STEREO_Z_DEPTH_MAX || img_mask_no_image_region_remap.at<uchar>(y, x) == 0) {


					mask_with_xyz.at<uchar>(y, x) = 0;


				}
				else {










					d_point_distance = sqrt(point_xyz[0] * point_xyz[0] + point_xyz[1] * point_xyz[1] + point_xyz[2] * point_xyz[2]);





					if ((d_point_distance < d_object_distance_from_camera_thresh_min) || (d_point_distance > d_object_distance_from_camera_thresh_max)) {


						mask_with_xyz.at<uchar>(y, x) = 0;



					}
					else {

						mask_with_xyz.at<uchar>(y, x) = 1;

					}










					if (YOLO_mode == NOT_USE_YOLO) {


						// check whether the pixel of interest is on a cloud region or not. //
						//if (img_mask_cloud1_remap.at<uchar>(y, x) == 255 && img_hsv.at<Vec3b>(y, x)[1] <= n_cloud_HSV_S_upper_thresh && img_hsv.at<Vec3b>(y, x)[2] >= n_cloud_HSV_V_lower_thresh){
						if (img_mask_non_texture_region_remap.at<uchar>(y, x) == 255) {

							mask_with_xyz.at<uchar>(y, x) = 0;

						}



					}










					//add 2018/02/23
					//remove the non-texture region using sobel edge strength
					//if (img_mask_non_texture_region_remap.at<uchar>(y, x) == 255 ){ 
					//	mask_with_xyz.at<uchar>(y, x) = 0;
					//}











				}




			}
		}







































		////////////////////////////////////////////////////////////////////
		///// Processing using depth with stereo camera and SLAM data  /////
		////////////////////////////////////////////////////////////////////

















		cv::Mat stereo_XYZ_input(3, 1, CV_64FC1);
		cv::Mat slam_map_XYZ(3, 1, CV_64FC1);


















		if (YOLO_mode == NOT_USE_YOLO) {


























			for (int y = 0; y < xyz.rows; y += SLAM_STEREO_3D_DATA_OUTPUT_STEP) {
				for (int x = 0; x < xyz.cols; x += SLAM_STEREO_3D_DATA_OUTPUT_STEP) {



					Vec3f point_xyz = xyz.at<Vec3f>(y, x);






					if (mask_with_xyz.at<uchar>(y, x) == 1) {





						stereo_XYZ_input.at<double>(0, 0) = (point_xyz[0] / 1000.0);
						stereo_XYZ_input.at<double>(1, 0) = (point_xyz[1] / 1000.0);
						stereo_XYZ_input.at<double>(2, 0) = (point_xyz[2] / 1000.0);


						slam_map_XYZ = mat_slam_R*stereo_XYZ_input;

						slam_map_XYZ.at<double>(0, 0) += slam_trajectory_d_Twc_X;
						slam_map_XYZ.at<double>(1, 0) += slam_trajectory_d_Twc_Y;
						slam_map_XYZ.at<double>(2, 0) += slam_trajectory_d_Twc_Z;





						fprintf_s(fp_w_orb_slam_object, "%lf\t%lf\t%lf\t%d\t%d\t%d\n", slam_map_XYZ.at<double>(0, 0), slam_map_XYZ.at<double>(1, 0), slam_map_XYZ.at<double>(2, 0), img_cam1_remap.at<cv::Vec3b>(y, x)[2], img_cam1_remap.at<cv::Vec3b>(y, x)[1], img_cam1_remap.at<cv::Vec3b>(y, x)[0]);







					}


				}//loop x
			}//loop y



































		}
		else if (YOLO_mode == YOLO_UME) {




















			//for displaying the bouding box//

			img_3D_plane = cv::Mat::zeros(IMG_YSIZE, IMG_XSIZE, CV_8UC3);

			img_feature_points = img_cam1_remap.clone();








			if ((int)yolo_BBox.size() == 0) {



				printf_s("[no bouding boxes]\n");




			}
			else {




				for (int i = 0; i < (int)yolo_BBox.size(); i++) {




					//printf_s("%d\t%d\t%d\t%d\n", yolo_BBox[i].x, yolo_BBox[i].y, yolo_BBox[i].width, yolo_BBox[i].height);

					int n_bb_width, n_bb_height;




					yolo_BBox_LTx = yolo_BBox[i].x;
					yolo_BBox_LTy = yolo_BBox[i].y;


					yolo_BBox_RBx = yolo_BBox[i].x + (yolo_BBox[i].width - 1);
					yolo_BBox_RBy = yolo_BBox[i].y + (yolo_BBox[i].height - 1);



					if (yolo_BBox_LTx < 0) yolo_BBox_LTx = 0;
					if (yolo_BBox_LTy < 0) yolo_BBox_LTy = 0;

					if (yolo_BBox_RBx >= IMG_XSIZE) yolo_BBox_RBx = IMG_XSIZE - 1;
					if (yolo_BBox_RBy >= IMG_YSIZE) yolo_BBox_RBy = IMG_YSIZE - 1;





					n_bb_width = yolo_BBox_RBx - yolo_BBox_LTx;
					n_bb_height = yolo_BBox_RBy - yolo_BBox_LTy;


					n_yolo_bb_pixel_num_in_Box = (n_bb_width + 1)*(n_bb_height + 1);







					//img_feature_points

					cv::rectangle(img_feature_points, cv::Point(yolo_BBox_LTx, yolo_BBox_LTy), cv::Point(yolo_BBox_RBx, yolo_BBox_RBy), cv::Scalar(0, 255, 0), 1, 4);










					//determine ootsu threshold

					n_ootsu_threshold_in_boudning_box = determine_ootsu_auto_thresh_in_bounding_box_gray_image(img_gray1, yolo_BBox[i].x, yolo_BBox[i].y, yolo_BBox_RBx, yolo_BBox_RBy, histogram_TANK);



					std::cout << "OOTSU Thresh:" << to_string(n_ootsu_threshold_in_boudning_box) << endl;












					vector<yolo_bounding_box_cluster_point> yolo_BBox_point;
					struct yolo_bounding_box_cluster_point yolo_buf_BBox_point;








					//for (int y = yolo_BBox_LTy; y <= yolo_BBox_RBy; y++){
					//	for (int x = yolo_BBox_LTx; x <= yolo_BBox_RBx; x++){
					//for (int y = yolo_BBox_LTy + n_bb_height / 3; y <= yolo_BBox_RBy; y++) {
					//	for (int x = yolo_BBox_LTx + n_bb_width / 6; x <= yolo_BBox_RBx - n_bb_width / 6; x++) {
					for (int y = yolo_BBox_LTy + n_bb_height / 6; y <= yolo_BBox_RBy - n_bb_height / 6; y++) {
						for (int x = yolo_BBox_LTx + n_bb_width / 6; x <= yolo_BBox_RBx - n_bb_width / 6; x++) {








							//check the gray level
							if (img_gray1.at<uchar>(y, x) > n_ootsu_threshold_in_boudning_box) continue;




							Vec3f point_xyz = xyz.at<Vec3f>(y, x);


							if (mask_with_xyz.at<uchar>(y, x) == 1) {


								//d_point_distance = sqrt(point_xyz[0] * point_xyz[0] + point_xyz[1] * point_xyz[1] + point_xyz[2] * point_xyz[2]);









								//unit: mm -> m

								stereo_XYZ_input.at<double>(0, 0) = (point_xyz[0] / 1000.0);
								stereo_XYZ_input.at<double>(1, 0) = (point_xyz[1] / 1000.0);
								stereo_XYZ_input.at<double>(2, 0) = (point_xyz[2] / 1000.0);




								slam_map_XYZ = mat_slam_R*  stereo_XYZ_input;





								slam_map_XYZ.at<double>(0, 0) += slam_trajectory_d_Twc_X;
								slam_map_XYZ.at<double>(1, 0) += slam_trajectory_d_Twc_Y;
								slam_map_XYZ.at<double>(2, 0) += slam_trajectory_d_Twc_Z;






								//set 3D points for clustering

								yolo_buf_BBox_point.x = slam_map_XYZ.at<double>(0, 0);
								yolo_buf_BBox_point.y = slam_map_XYZ.at<double>(1, 0);
								yolo_buf_BBox_point.z = slam_map_XYZ.at<double>(2, 0);

								yolo_BBox_point.push_back(yolo_buf_BBox_point);






								if ((x % YOLO_BOUNDING_BOX_3DPOINT_OUTPUT_DATA_STEP == 0) && (y % YOLO_BOUNDING_BOX_3DPOINT_OUTPUT_DATA_STEP == 0)) {
									fprintf_s(fp_w_orb_slam_object, "%lf\t%lf\t%lf\t%d\t%d\t%d\n", slam_map_XYZ.at<double>(0, 0), slam_map_XYZ.at<double>(1, 0), slam_map_XYZ.at<double>(2, 0), img_cam1_remap.at<cv::Vec3b>(y, x)[2], img_cam1_remap.at<cv::Vec3b>(y, x)[1], img_cam1_remap.at<cv::Vec3b>(y, x)[0]);
								}







								img_3D_plane.at<Vec3b>(y, x) = Vec3b(255, 255, 255);//<=====










							}





						}//loop bounding box x
					}//loop bounding box y





					cout << "Point Number in boudning box: " << to_string((int)yolo_BBox_point.size()) << endl;





					//k-means

					struct yolo_bounding_box_cluster_point BBox_object;
					int n_ret;



					if ((int)yolo_BBox_point.size() < n_yolo_bb_object_cluster_candidates_num * n_yolo_bb_point_num_magnification_to_do_clustering) {

						std::cout << "Error: too little points in bouding box" << endl;
					}
					else {



						n_ret = determine_primary_object_center_in_bounding_box_using_k_means(yolo_BBox_point, n_yolo_bb_object_cluster_candidates_num, n_yolo_bb_object_clustering_k_means_loop_times, &BBox_object);



						d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox = BBox_object.count / (double)n_yolo_bb_pixel_num_in_Box;





						//if (d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox > d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox_thresh){
						if (BBox_object.count > n_yolo_bb_object_pixel_num_thresh) {


							fprintf_s(fp_w_yolo_bounding_box_object_k_means, "%lf\t%lf\t%lf\t%d\t%d\t%lf\n", BBox_object.x, BBox_object.y, BBox_object.z, BBox_object.count, n_yolo_bb_pixel_num_in_Box, d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox);

							buf_yolo_bbox_object_center.x = BBox_object.x;
							buf_yolo_bbox_object_center.y = BBox_object.y;
							buf_yolo_bbox_object_center.z = BBox_object.z;
							buf_yolo_bbox_object_center.n_object_pixel_num = BBox_object.count;
							buf_yolo_bbox_object_center.n_bounding_box_pixel_num = n_yolo_bb_pixel_num_in_Box;
							buf_yolo_bbox_object_center.d_boject_pixel_ratio_to_box = d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox;


							yolo_bbox_object_center.push_back(buf_yolo_bbox_object_center);









							// Clustering by using Nearest Neighborhood method //

							if ((int)yolo_detected_Object.size() == 0) {


								buf_yolo_detected_object.cx = BBox_object.x;
								buf_yolo_detected_object.cy = BBox_object.y;
								buf_yolo_detected_object.cz = BBox_object.z;

								buf_yolo_detected_object.sum_x = BBox_object.x;
								buf_yolo_detected_object.sum_y = BBox_object.y;
								buf_yolo_detected_object.sum_z = BBox_object.z;
								buf_yolo_detected_object.counter = 1;

								yolo_detected_Object.push_back(buf_yolo_detected_object);


								//fprintf_s(fp_w_yolo_detected_object_NN, "%d\t%lf\n", (int)yolo_detected_Object.size(), d_distance_min_in_nearest_neighborhood_method);

							}
							else {


								// search the nearest cluster

								//d_distance_thresh_in_nearest_neighborhood_method = 3.0;// 1 m 




								d_distance_min_in_nearest_neighborhood_method = 1000000;

								for (int k = 0; k < (int)yolo_detected_Object.size(); k++) {



									//calc the distance between the current detected object position and the previously detected object positions

									d_distance_in_nearest_neighborhood_method = (BBox_object.x - yolo_detected_Object[k].cx)*(BBox_object.x - yolo_detected_Object[k].cx) + (BBox_object.y - yolo_detected_Object[k].cy)*(BBox_object.y - yolo_detected_Object[k].cy) + (BBox_object.z - yolo_detected_Object[k].cz)*(BBox_object.z - yolo_detected_Object[k].cz);

									d_distance_in_nearest_neighborhood_method = sqrt(d_distance_in_nearest_neighborhood_method);


									//search the nearest object
									if (d_distance_min_in_nearest_neighborhood_method > d_distance_in_nearest_neighborhood_method) {


										d_distance_min_in_nearest_neighborhood_method = d_distance_in_nearest_neighborhood_method;

										n_distance_min_cluster_no_in_nearest_neighborhood_method = k;


									}


								}





								//add a new object when the min distance is larger than thresholding value

								if (d_distance_min_in_nearest_neighborhood_method > d_distance_thresh_in_nearest_neighborhood_method) {


									buf_yolo_detected_object.cx = BBox_object.x;
									buf_yolo_detected_object.cy = BBox_object.y;
									buf_yolo_detected_object.cz = BBox_object.z;

									buf_yolo_detected_object.sum_x = BBox_object.x;
									buf_yolo_detected_object.sum_y = BBox_object.y;
									buf_yolo_detected_object.sum_z = BBox_object.z;
									buf_yolo_detected_object.counter = 1;


									yolo_detected_Object.push_back(buf_yolo_detected_object);


									//fprintf_s(fp_w_yolo_detected_object_NN, "%d\t%lf\n", (int)yolo_detected_Object.size(), d_distance_min_in_nearest_neighborhood_method);



								}
								else { //update the cluster


									yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].sum_x += BBox_object.x;
									yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].sum_y += BBox_object.y;
									yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].sum_z += BBox_object.z;

									yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].counter++;


									yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].cx = ((yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].counter - 1) * yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].cx + BBox_object.x) / (double)yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].counter;
									yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].cy = ((yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].counter - 1) * yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].cy + BBox_object.y) / (double)yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].counter;
									yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].cz = ((yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].counter - 1) * yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].cz + BBox_object.z) / (double)yolo_detected_Object[n_distance_min_cluster_no_in_nearest_neighborhood_method].counter;







								}







							}//if:NN method













						}//if thresh






					}//if BBox point number








					 //clear buffer
					yolo_BBox_point.clear();
					yolo_BBox_point.shrink_to_fit();






				}//loop for each bouding box




			}//if bouding box




			cv::imshow(win_dst0, img_feature_points);
			cv::imshow(win_dst_wall_mask, img_3D_plane);






































		}
		else if (YOLO_mode == YOLO_OBJECTS) {































			//for displaying the bouding box//

			img_3D_plane = cv::Mat::zeros(IMG_YSIZE, IMG_XSIZE, CV_8UC3);

			img_feature_points = img_cam1_remap.clone();








			/*
			for (int y = 0; y < xyz.rows; y++) {
			for (int x = 0; x < xyz.cols; x++) {



			Vec3f point_xyz = xyz.at<Vec3f>(y, x);



			if (mask_with_xyz.at<uchar>(y, x) == 1 && point_xyz[0] < 0) {
			img_3D_plane.at<Vec3b>(y, x) = Vec3b(0, 255, 0);
			}



			}
			}

			*/

























			if ((int)yolo_BBox.size() == 0) {








				printf_s("[no bouding boxes]\n");









			}
			else {













				vector<struct interested_point_xy> img_points_on_road_center_line;
				struct interested_point_xy buf_img_point_on_road_center_line;






				vector<struct yolo_bounding_box_point> road_edge_left_point_candidate;
				vector<struct yolo_bounding_box_point> road_edge_right_point_candidate;
				struct yolo_bounding_box_point buf_road_edge_left_point_candidate;
				struct yolo_bounding_box_point buf_road_edge_right_point_candidate;

				vector<struct interested_point_xy> road_edge_left_image_point_candidate;
				vector<struct interested_point_xy> road_edge_right_image_point_candidate;
				struct interested_point_xy buf_road_edge_left_image_point_candidate;
				struct interested_point_xy buf_road_edge_right_image_point_candidate;






				int n_yolo_object_id;
				int n_yolo_object_pixel_color_pallete_position;
				int n_yolo_object_pixel_color_pallete_band_width = 360 / n_yolo_bb_targeted_object_num; //0-360. 0 color is near 360 color.
				int n_yolo_object_pixel_color_R, n_yolo_object_pixel_color_G, n_yolo_object_pixel_color_B;




				int n_flag_road_edge_line_bounding_box;

				int n_bounding_box_size_max;

				struct yolo_object_detected_position road_edge_nearest_bounding_box_point;









				//for BBox loop




				n_bounding_box_size_max = 0;


				for (int i = 0; i < (int)yolo_BBox.size(); i++) {








					//check road edge bounding box

					if ((strcmp(yolo_bounding_box_label_name_of_road_edge, yolo_BBox[i].name)) == 0) n_flag_road_edge_line_bounding_box = 1;
					else n_flag_road_edge_line_bounding_box = -1;














					// targeted object //





					n_yolo_object_id = -1;



					for (int j = 0; j < n_yolo_bb_targeted_object_num; j++) {

						if ((strcmp(&yolo_bounding_box_targeted_object_name_list[j][0], yolo_BBox[i].name)) == 0) {
							n_yolo_object_id = j;
							break;
						}

					}















					if (n_yolo_object_id >= 0) {









						//color pallete position
						n_yolo_object_pixel_color_pallete_position = n_yolo_object_pixel_color_pallete_band_width * n_yolo_object_id;

						n_yolo_object_pixel_color_R = color_table_RGB[n_yolo_object_pixel_color_pallete_position].R;
						n_yolo_object_pixel_color_G = color_table_RGB[n_yolo_object_pixel_color_pallete_position].G;
						n_yolo_object_pixel_color_B = color_table_RGB[n_yolo_object_pixel_color_pallete_position].B;














						//printf_s("%d\t%d\t%d\t%d\n", yolo_BBox[i].x, yolo_BBox[i].y, yolo_BBox[i].width, yolo_BBox[i].height);





						int n_bb_width, n_bb_height;
						int n_bb_center_x, n_bb_center_y;



						yolo_BBox_LTx = yolo_BBox[i].x;
						yolo_BBox_LTy = yolo_BBox[i].y;


						yolo_BBox_RBx = yolo_BBox[i].x + (yolo_BBox[i].width - 1);
						yolo_BBox_RBy = yolo_BBox[i].y + (yolo_BBox[i].height - 1);







						if (yolo_BBox_LTx < 0) yolo_BBox_LTx = 0;
						if (yolo_BBox_LTy < 0) yolo_BBox_LTy = 0;

						if (yolo_BBox_RBx >= IMG_XSIZE) yolo_BBox_RBx = IMG_XSIZE - 1;
						if (yolo_BBox_RBy >= IMG_YSIZE) yolo_BBox_RBy = IMG_YSIZE - 1;





						n_bb_width = yolo_BBox_RBx - yolo_BBox_LTx;
						n_bb_height = yolo_BBox_RBy - yolo_BBox_LTy;

						n_bb_center_x = (yolo_BBox_LTx + yolo_BBox_RBx) / 2;
						n_bb_center_y = (yolo_BBox_LTy + yolo_BBox_RBy) / 2;





						n_yolo_bb_pixel_num_in_Box = (n_bb_width + 1)*(n_bb_height + 1);















						/// store the image points on road center line ///

						if (n_flag_road_edge_line_bounding_box != 1) {






							buf_img_point_on_road_center_line.x = n_bb_center_x;
							buf_img_point_on_road_center_line.y = n_bb_center_y;

							img_points_on_road_center_line.push_back(buf_img_point_on_road_center_line);








						}













						//img_feature_points

						//cv::rectangle(img_feature_points, cv::Point(yolo_BBox_LTx, yolo_BBox_LTy), cv::Point(yolo_BBox_RBx, yolo_BBox_RBy), cv::Scalar(0, 255, 0), 1, 4);
						cv::rectangle(img_feature_points, cv::Point(yolo_BBox_LTx, yolo_BBox_LTy), cv::Point(yolo_BBox_RBx, yolo_BBox_RBy), cv::Scalar(n_yolo_object_pixel_color_B, n_yolo_object_pixel_color_G, n_yolo_object_pixel_color_R), 1, 4);
















						vector<yolo_bounding_box_point> yolo_BBox_point;
						struct yolo_bounding_box_point yolo_buf_BBox_point;


						//vector<yolo_bounding_box_cluster_point> road_edge_yolo_BBox_point;
						//struct yolo_bounding_box_cluster_point road_edge_yolo_buf_BBox_point;


						vector<yolo_bounding_box_point> yolo_BBox_point_of_interest_in_camera_coordinates;
						struct yolo_bounding_box_point yolo_buf_BBox_point_of_interest_in_camera_coordinates;








						//set the window size to detect road edge line in the yolo bbox

						int n_window_size;
						double d_window_base_point_y[2];
						double d_window_base_point_w[2];

						int n_left_edge_line_region_x_min;
						int n_left_edge_line_region_x_max;
						int n_right_edge_line_region_x_min;
						int n_right_edge_line_region_x_max;

						int n_upper_window_size_scale_to_estimate_edge_line = 20;



						d_window_base_point_y[0] = 160;
						d_window_base_point_w[0] = n_yolo_bb_window_half_size_to_acquire_xyz_coordinates / n_upper_window_size_scale_to_estimate_edge_line;
						d_window_base_point_y[1] = 320;
						d_window_base_point_w[1] = n_yolo_bb_window_half_size_to_acquire_xyz_coordinates;

						n_window_size = int((d_window_base_point_w[1] - d_window_base_point_w[0]) / (d_window_base_point_y[1] - d_window_base_point_y[0])*(n_bb_center_y - d_window_base_point_y[0]) + d_window_base_point_w[0]);





						n_left_edge_line_region_x_min = ((n_road_edge_line_distance_estimation_bbox_window_center_position_div_num - 1)*yolo_BBox_LTx + yolo_BBox_RBx) / n_road_edge_line_distance_estimation_bbox_window_center_position_div_num - n_window_size;
						n_left_edge_line_region_x_max = ((n_road_edge_line_distance_estimation_bbox_window_center_position_div_num - 1)*yolo_BBox_LTx + yolo_BBox_RBx) / n_road_edge_line_distance_estimation_bbox_window_center_position_div_num + n_window_size;

						n_right_edge_line_region_x_min = (yolo_BBox_LTx + (n_road_edge_line_distance_estimation_bbox_window_center_position_div_num - 1)*yolo_BBox_RBx) / n_road_edge_line_distance_estimation_bbox_window_center_position_div_num - n_window_size;
						n_right_edge_line_region_x_max = (yolo_BBox_LTx + (n_road_edge_line_distance_estimation_bbox_window_center_position_div_num - 1)*yolo_BBox_RBx) / n_road_edge_line_distance_estimation_bbox_window_center_position_div_num + n_window_size;
						//n_right_edge_line_region_x_min = yolo_BBox_RBx - 2*n_window_size;
						//n_right_edge_line_region_x_max = yolo_BBox_RBx;













						//for (int y = yolo_BBox_LTy; y <= yolo_BBox_RBy; y++) {
						//	for (int x = yolo_BBox_LTx; x <= yolo_BBox_RBx; x++) {
						//for (int y = yolo_BBox_RBy - 2*n_yolo_bb_window_half_size_to_acquire_xyz_coordinates; y <= yolo_BBox_RBy; y++) {
						//	for (int x = n_bb_center_x - n_yolo_bb_window_half_size_to_acquire_xyz_coordinates; x <= n_bb_center_x + n_yolo_bb_window_half_size_to_acquire_xyz_coordinates; x++) {
						//for (int x = yolo_BBox_LTx; x <= yolo_BBox_RBx; x++) {

						for (int y = yolo_BBox_RBy - n_window_size; y <= yolo_BBox_RBy; y++) {
							//for (int x = n_bb_center_x - n_window_size; x <= n_bb_center_x + n_window_size; x++) {
							for (int x = yolo_BBox_LTx; x <= yolo_BBox_RBx; x++) {





								//check the gray level
								//if (img_gray1.at<uchar>(y, x) > n_ootsu_threshold_in_boudning_box) continue;




								Vec3f point_xyz = xyz.at<Vec3f>(y, x);


								if (mask_with_xyz.at<uchar>(y, x) == 1) {


									//d_point_distance = sqrt(point_xyz[0] * point_xyz[0] + point_xyz[1] * point_xyz[1] + point_xyz[2] * point_xyz[2]);









									//unit: mm -> m

									stereo_XYZ_input.at<double>(0, 0) = (point_xyz[0] / 1000.0);
									stereo_XYZ_input.at<double>(1, 0) = (point_xyz[1] / 1000.0);
									stereo_XYZ_input.at<double>(2, 0) = (point_xyz[2] / 1000.0);




									slam_map_XYZ = mat_slam_R*  stereo_XYZ_input;





									slam_map_XYZ.at<double>(0, 0) += slam_trajectory_d_Twc_X;
									slam_map_XYZ.at<double>(1, 0) += slam_trajectory_d_Twc_Y;
									slam_map_XYZ.at<double>(2, 0) += slam_trajectory_d_Twc_Z;
















									yolo_buf_BBox_point.x = slam_map_XYZ.at<double>(0, 0);
									yolo_buf_BBox_point.y = slam_map_XYZ.at<double>(1, 0);
									yolo_buf_BBox_point.z = slam_map_XYZ.at<double>(2, 0);




									// yolo_BBox_point

									yolo_BBox_point.push_back(yolo_buf_BBox_point);









									// object of interest using 2D array in CAMERA COORDINATES SYSTEM

									yolo_buf_BBox_point_of_interest_in_camera_coordinates.x = stereo_XYZ_input.at<double>(0, 0);
									yolo_buf_BBox_point_of_interest_in_camera_coordinates.y = stereo_XYZ_input.at<double>(1, 0);
									yolo_buf_BBox_point_of_interest_in_camera_coordinates.z = stereo_XYZ_input.at<double>(2, 0);




									yolo_BBox_point_of_interest_in_camera_coordinates.push_back(yolo_buf_BBox_point_of_interest_in_camera_coordinates);











									// road edge //


									//check the object label and set the points for processings

									if (n_flag_road_edge_line_bounding_box == 1) {







										//if (stereo_XYZ_input.at<double>(0, 0) < 0 && (x >= n_left_edge_line_region_x_min && x <= n_left_edge_line_region_x_max)) {
										if (x >= n_left_edge_line_region_x_min && x <= n_left_edge_line_region_x_max) {




											buf_road_edge_left_image_point_candidate.x = x;
											buf_road_edge_left_image_point_candidate.y = y;
											road_edge_left_image_point_candidate.push_back(buf_road_edge_left_image_point_candidate);






											buf_road_edge_left_point_candidate.x = stereo_XYZ_input.at<double>(0, 0);
											buf_road_edge_left_point_candidate.y = stereo_XYZ_input.at<double>(1, 0);
											buf_road_edge_left_point_candidate.z = stereo_XYZ_input.at<double>(2, 0);

											road_edge_left_point_candidate.push_back(buf_road_edge_left_point_candidate);


											//img_3D_plane.at<Vec3b>(y, x) = Vec3b(0, 255, 0);//<=====





										}
										//else if( stereo_XYZ_input.at<double>(0, 0) >=0 && (x >= n_right_edge_line_region_x_min && x <= n_right_edge_line_region_x_max) ){
										else if (x >= n_right_edge_line_region_x_min && x <= n_right_edge_line_region_x_max) {



											buf_road_edge_right_image_point_candidate.x = x;
											buf_road_edge_right_image_point_candidate.y = y;
											road_edge_right_image_point_candidate.push_back(buf_road_edge_right_image_point_candidate);




											buf_road_edge_right_point_candidate.x = stereo_XYZ_input.at<double>(0, 0);
											buf_road_edge_right_point_candidate.y = stereo_XYZ_input.at<double>(1, 0);
											buf_road_edge_right_point_candidate.z = stereo_XYZ_input.at<double>(2, 0);

											road_edge_right_point_candidate.push_back(buf_road_edge_right_point_candidate);



											//img_3D_plane.at<Vec3b>(y, x) = Vec3b(0, 0, 255);//<=====

										}











									}//if compare yolo object label string: road_edge











									 //setting for saving slam map data


									if ((x % YOLO_BOUNDING_BOX_3DPOINT_OUTPUT_DATA_STEP == 0) && (y % YOLO_BOUNDING_BOX_3DPOINT_OUTPUT_DATA_STEP == 0)) {

										fprintf_s(fp_w_orb_slam_object, "%lf\t%lf\t%lf\t%d\t%d\t%d\t%d\t%d\t%d\n", slam_map_XYZ.at<double>(0, 0), slam_map_XYZ.at<double>(1, 0), slam_map_XYZ.at<double>(2, 0), img_cam1_remap.at<cv::Vec3b>(y, x)[2], img_cam1_remap.at<cv::Vec3b>(y, x)[1], img_cam1_remap.at<cv::Vec3b>(y, x)[0], n_yolo_object_pixel_color_R, n_yolo_object_pixel_color_G, n_yolo_object_pixel_color_B);

									}







									//img_3D_plane.at<Vec3b>(y, x) = Vec3b(255, 255, 255);//<=====










								}//if mask





							}//loop bounding box x
						}//loop bounding box y



















						cout << "Point Number in boudning box: " << to_string((int)yolo_BBox_point.size()) << endl;























						// calc center using all of the pixels in BBox //







						struct yolo_object_detected_position BBox_object;





						//if ((int)yolo_BBox_point.size() < n_yolo_bb_object_cluster_candidates_num * n_yolo_bb_point_num_magnification_to_do_clustering) {
						if ((int)yolo_BBox_point.size() < 1) {










							std::cout << "Error: too little points in bouding box" << endl;













						}
						else {











							std::cout << "OK: can use the points enough to estimate" << endl;






							// Not calc the camera coordinates but the absolute coordinates //





							BBox_object.sum_x = BBox_object.sum_y = BBox_object.sum_z = 0;




							for (int k = 0; k < (int)yolo_BBox_point.size(); k++) {





								BBox_object.sum_x += yolo_BBox_point[k].x;
								BBox_object.sum_y += yolo_BBox_point[k].y;
								BBox_object.sum_z += yolo_BBox_point[k].z;





							}


							BBox_object.cx = BBox_object.sum_x / (double)yolo_BBox_point.size();
							BBox_object.cy = BBox_object.sum_y / (double)yolo_BBox_point.size();
							BBox_object.cz = BBox_object.sum_z / (double)yolo_BBox_point.size();

							BBox_object.counter = (int)yolo_BBox_point.size();











							d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox = (int)yolo_BBox_point.size() / (double)n_yolo_bb_pixel_num_in_Box;

							fprintf_s(fp_w_yolo_bounding_box_object_k_means, "%d\t%lf\t%lf\t%lf\t%d\t%d\t%lf\n", n_current_frame_No, BBox_object.cx, BBox_object.cy, BBox_object.cz, (int)yolo_BBox_point.size(), n_yolo_bb_pixel_num_in_Box, d_yolo_bb_object_cluster_pixel_num_ratio_in_bbox);







							if (n_flag_road_edge_line_bounding_box == 1 && BBox_object.cx < 0 && n_bounding_box_size_max < n_yolo_bb_pixel_num_in_Box) {

								n_bounding_box_size_max = n_yolo_bb_pixel_num_in_Box;

								road_edge_nearest_bounding_box_point.cx = BBox_object.cx;
								road_edge_nearest_bounding_box_point.cy = BBox_object.cy;
								road_edge_nearest_bounding_box_point.cz = BBox_object.cz;



							}






						}//if





















						 // Objects of interest //


						if (yolo_BBox_point_of_interest_in_camera_coordinates.size() < 1) {










							std::cout << "[Object of interest] Error: too little points in bouding box" << endl;











						}
						else {





							vector<struct yolo_objects_of_interest_detected_position> yolo_detected_OBJECTS_of_interest(n_yolo_bb_targeted_object_num);
							//yolo_detected_OBJECTS_of_interest.resize(n_yolo_bb_targeted_object_num);






							int n_distance_VOTE[YOLO_BOUNDING_BOX_INTERSECTION_VOTE_DISTANCE_MAX];//
							double d_distance_SUM[YOLO_BOUNDING_BOX_INTERSECTION_VOTE_DISTANCE_MAX];

							int n_distance_vote_box_position;
							int n_distance_vote_max_position;
							int n_distance_vote_max_num;
							double d_distance;
							//double d_estimated_distance;


							double dsum_x, dsum_y, dsum_z;
















							std::cout << "OK: can use the points enough to estimate" << endl;




							for (int i = 0; i < n_yolo_bb_targeted_object_num; i++) {


								yolo_detected_OBJECTS_of_interest[i].point_num = 0;
								yolo_detected_OBJECTS_of_interest[i].box_height = 0;
								yolo_detected_OBJECTS_of_interest[i].box_width = 0;
								yolo_detected_OBJECTS_of_interest[i].cx = 0;
								yolo_detected_OBJECTS_of_interest[i].cy = 0;
								yolo_detected_OBJECTS_of_interest[i].distance_from_camera = 0;


							}










							for (int k = 0; k < YOLO_BOUNDING_BOX_INTERSECTION_VOTE_DISTANCE_MAX; k++) {


								n_distance_VOTE[k] = 0;
								d_distance_SUM[k] = 0;


							}






							dsum_x = dsum_y = dsum_z = 0;







							for (int k = 0; k < (int)yolo_BBox_point_of_interest_in_camera_coordinates.size(); k++) {





								dsum_x += yolo_BBox_point_of_interest_in_camera_coordinates[k].x;
								dsum_y += yolo_BBox_point_of_interest_in_camera_coordinates[k].y;
								dsum_z += yolo_BBox_point_of_interest_in_camera_coordinates[k].z;







								d_distance = yolo_BBox_point_of_interest_in_camera_coordinates[k].z;


								if (d_distance < d_object_distance_from_camera_thresh_max) {



									n_distance_vote_box_position = (int)d_distance;



									n_distance_VOTE[n_distance_vote_box_position]++;

									d_distance_SUM[n_distance_vote_box_position] += yolo_BBox_point_of_interest_in_camera_coordinates[k].z;




								}








							}








							yolo_detected_OBJECTS_of_interest[n_yolo_object_id].box_height = n_bb_height;
							yolo_detected_OBJECTS_of_interest[n_yolo_object_id].box_width = n_bb_width;



							yolo_detected_OBJECTS_of_interest[n_yolo_object_id].point_num = (int)yolo_BBox_point_of_interest_in_camera_coordinates.size();

							yolo_detected_OBJECTS_of_interest[n_yolo_object_id].cx = dsum_x / (double)yolo_BBox_point_of_interest_in_camera_coordinates.size();
							yolo_detected_OBJECTS_of_interest[n_yolo_object_id].cy = dsum_y / (double)yolo_BBox_point_of_interest_in_camera_coordinates.size();
							yolo_detected_OBJECTS_of_interest[n_yolo_object_id].cz = dsum_z / (double)yolo_BBox_point_of_interest_in_camera_coordinates.size();







							n_distance_vote_max_num = n_distance_VOTE[0];
							n_distance_vote_max_position = 0;

							for (int k = 0; k < YOLO_BOUNDING_BOX_INTERSECTION_VOTE_DISTANCE_MAX; k++) {

								if (n_distance_vote_max_num < n_distance_VOTE[k]) {

									n_distance_vote_max_num = n_distance_VOTE[k];
									n_distance_vote_max_position = k;
								}

							}




							if (n_distance_vote_max_num > 0) {





								yolo_detected_OBJECTS_of_interest[n_yolo_object_id].distance_from_camera = d_distance_SUM[n_distance_vote_max_position] / (double)n_distance_vote_max_num;







							}
							else {

								yolo_detected_OBJECTS_of_interest[n_yolo_object_id].distance_from_camera = YOLO_BOUNDING_BOX_INTERSECTION_VOTE_DISTANCE_MAX;

							}











							//save object position of interest

							fprintf_s(fp_w_yolo_bounding_box_object_of_interest, "%d\t|", n_current_frame_No);


							for (int i = 0; i < n_yolo_bb_targeted_object_num; i++) {


								if (yolo_detected_OBJECTS_of_interest[i].point_num > 0) {

									fprintf_s(fp_w_yolo_bounding_box_object_of_interest, "\t%lf\t%lf\t%lf\t%d\t%d\t%lf\t%d\t|", yolo_detected_OBJECTS_of_interest[i].cx, yolo_detected_OBJECTS_of_interest[i].cy, yolo_detected_OBJECTS_of_interest[i].cz, yolo_detected_OBJECTS_of_interest[i].box_height, yolo_detected_OBJECTS_of_interest[i].box_width, yolo_detected_OBJECTS_of_interest[i].distance_from_camera, yolo_detected_OBJECTS_of_interest[i].point_num);


								}
								else {

									fprintf_s(fp_w_yolo_bounding_box_object_of_interest, "\t?\t?\t?\t?\t?\t?\t?\t|");

								}





							}


							fprintf_s(fp_w_yolo_bounding_box_object_of_interest, "\n");






						}







































						//clear buffer

						if (yolo_BBox_point.size() > 0) {

							yolo_BBox_point.clear();
							yolo_BBox_point.shrink_to_fit();

						}


						if (yolo_BBox_point_of_interest_in_camera_coordinates.size() > 0) {

							yolo_BBox_point_of_interest_in_camera_coordinates.clear();
							yolo_BBox_point_of_interest_in_camera_coordinates.shrink_to_fit();
						}











					}//if targeted object, check counter




















				}//loop for each bouding box, yolo_BBox.size()





















				 /////////////////////////////////////////

















				 /// DETECT ROAD CENTER LINE AND EDGE ///















				 /// Detect road center line ///


				struct interested_point_xy road_center_line_org;
				double road_center_line_vector[2];
				double road_center_line_perpendicular_vector[2];








				detect_road_center_line_in_image_use_RANSAC(img_points_on_road_center_line, 100, 30, &road_center_line_org, road_center_line_vector, road_center_line_perpendicular_vector);








				int n_draw_line_length = 50;
				int n_draw_rectangle_width = 10;
				cv::line(img_feature_points, cv::Point(road_center_line_org.x, road_center_line_org.y), cv::Point(int(road_center_line_org.x + road_center_line_vector[0] * n_draw_line_length), int(road_center_line_org.y + road_center_line_vector[1] * n_draw_line_length)), cv::Scalar(255, 0, 0), 2, 4);
				cv::rectangle(img_feature_points, cv::Point(road_center_line_org.x - n_draw_rectangle_width, road_center_line_org.y - n_draw_rectangle_width), cv::Point(road_center_line_org.x + n_draw_rectangle_width, road_center_line_org.y + n_draw_rectangle_width), cv::Scalar(255, 0, 0), -1, CV_AA);















				/// Extract LEFT ADN RIGHT Road Edge Points ///


				vector<struct yolo_bounding_box_point> road_edge_left_point;
				vector<struct yolo_bounding_box_point> road_edge_right_point;
				struct yolo_bounding_box_point buf_road_edge_left_point;
				struct yolo_bounding_box_point buf_road_edge_right_point;



				double d_x_projection_on_the_perpendicular_vector;





				// LEFT edge points

				for (int k = 0; k < (int)road_edge_left_image_point_candidate.size(); k++) {




					//calc projection

					d_x_projection_on_the_perpendicular_vector = (road_edge_left_image_point_candidate[k].x - road_center_line_org.x)*road_center_line_perpendicular_vector[0] + (road_edge_left_image_point_candidate[k].y - road_center_line_org.y)*road_center_line_perpendicular_vector[1];






					if (d_x_projection_on_the_perpendicular_vector < 0) {


						buf_road_edge_left_point.x = road_edge_left_point_candidate[k].x;
						buf_road_edge_left_point.y = road_edge_left_point_candidate[k].y;
						buf_road_edge_left_point.z = road_edge_left_point_candidate[k].z;

						road_edge_left_point.push_back(buf_road_edge_left_point);


						img_3D_plane.at<Vec3b>(road_edge_left_image_point_candidate[k].y, road_edge_left_image_point_candidate[k].x) = Vec3b(0, 255, 0);


					}










				}






				// RIGHT edge points


				for (int k = 0; k < (int)road_edge_right_image_point_candidate.size(); k++) {




					//calc projection

					d_x_projection_on_the_perpendicular_vector = (road_edge_right_image_point_candidate[k].x - road_center_line_org.x)*road_center_line_perpendicular_vector[0] + (road_edge_right_image_point_candidate[k].y - road_center_line_org.y)*road_center_line_perpendicular_vector[1];






					if (d_x_projection_on_the_perpendicular_vector > 0) {


						buf_road_edge_right_point.x = road_edge_right_point_candidate[k].x;
						buf_road_edge_right_point.y = road_edge_right_point_candidate[k].y;
						buf_road_edge_right_point.z = road_edge_right_point_candidate[k].z;

						road_edge_right_point.push_back(buf_road_edge_right_point);


						img_3D_plane.at<Vec3b>(road_edge_right_image_point_candidate[k].y, road_edge_right_image_point_candidate[k].x) = Vec3b(204, 0, 196);


					}










				}














				/// Detect road edge line ///









				int road_edge_left_line_n_voted_count;
				double road_edge_left_line_d_estimated_distance;
				double road_edge_left_line_d_estimated_distance_using_moving_average;


				int road_edge_right_line_n_voted_count;
				double road_edge_right_line_d_estimated_distance;
				double road_edge_right_line_d_estimated_distance_using_moving_average;


				int n_flag_could_detect_left_road_edge_line;
				int n_flag_could_detect_right_road_edge_line;



				cv::Mat buf_left_edge_stereo_xyz(3, 1, CV_64FC1);
				cv::Mat buf_left_edge__slam_map_xyz(3, 1, CV_64FC1);
				cv::Mat buf_right_edge_stereo_xyz(3, 1, CV_64FC1);
				cv::Mat buf_right_edge__slam_map_xyz(3, 1, CV_64FC1);

				int n_lower_limit_edge_candidates_points_to_estimate_edge_line;


				// detect left edge





				//d_road_edge_estimation_distance_resolution = 0.8;
				n_lower_limit_edge_candidates_points_to_estimate_edge_line = 100;





				n_flag_could_detect_left_road_edge_line = -1;




				if (road_edge_left_point.size() > n_lower_limit_edge_candidates_points_to_estimate_edge_line) {




					detect_road_edge_line_using_VotingMethod(road_edge_left_point, -10, 10, d_road_edge_line_estimation_distance_resolution, n_yolo_bb_window_half_size_to_acquire_xyz_coordinates*n_yolo_bb_window_half_size_to_acquire_xyz_coordinates, &n_flag_could_detect_left_road_edge_line, &road_edge_left_line_d_estimated_distance, &road_edge_left_line_n_voted_count);




					if (n_flag_could_detect_left_road_edge_line > 0) {


						if (n_flag_first_count_to_estimate_distance_to_left_road_edge_line < 0) {

							road_edge_left_line_d_estimated_distance_using_moving_average = road_edge_left_line_d_estimated_distance;
							n_flag_first_count_to_estimate_distance_to_left_road_edge_line = 1;

						}
						else {

							road_edge_left_line_d_estimated_distance_using_moving_average = (road_edge_left_line_d_estimated_distance + (n_road_edge_line_distance_estimation_moving_average_method_frame_number - 1)*road_edge_left_line_d_estimated_distance_using_moving_average) / (double)n_road_edge_line_distance_estimation_moving_average_method_frame_number;

						}







						//buf_left_edge_stereo_xyz.at<double>(0, 0) = road_edge_left_line_d_estimated_distance_using_moving_average;
						//buf_left_edge_stereo_xyz.at<double>(1, 0) = 0;
						//buf_left_edge_stereo_xyz.at<double>(2, 0) = 0;
						//
						//buf_left_edge__slam_map_xyz = mat_slam_R*buf_left_edge_stereo_xyz;
						//
						//buf_left_edge__slam_map_xyz.at<double>(0, 0) += slam_trajectory_d_Twc_X;
						//buf_left_edge__slam_map_xyz.at<double>(1, 0) += slam_trajectory_d_Twc_Y;
						//buf_left_edge__slam_map_xyz.at<double>(2, 0) += slam_trajectory_d_Twc_Z;










					}//if (n_flag_could_detect_left_road_edge_line > 0)

				}//if (road_edge_left_point.size() > 100)









				 // detect right edge


				n_flag_could_detect_right_road_edge_line = -1;



				if (road_edge_right_point.size() > n_lower_limit_edge_candidates_points_to_estimate_edge_line) {




					detect_road_edge_line_using_VotingMethod(road_edge_right_point, -10, 10, d_road_edge_line_estimation_distance_resolution, n_yolo_bb_window_half_size_to_acquire_xyz_coordinates*n_yolo_bb_window_half_size_to_acquire_xyz_coordinates, &n_flag_could_detect_right_road_edge_line, &road_edge_right_line_d_estimated_distance, &road_edge_right_line_n_voted_count);




					if (n_flag_could_detect_right_road_edge_line > 0) {


						if (n_flag_first_count_to_estimate_distance_to_right_road_edge_line < 0) {

							road_edge_right_line_d_estimated_distance_using_moving_average = road_edge_right_line_d_estimated_distance;
							n_flag_first_count_to_estimate_distance_to_right_road_edge_line = 1;

						}
						else {

							road_edge_right_line_d_estimated_distance_using_moving_average = (road_edge_right_line_d_estimated_distance + (n_road_edge_line_distance_estimation_moving_average_method_frame_number - 1)*road_edge_right_line_d_estimated_distance_using_moving_average) / (double)n_road_edge_line_distance_estimation_moving_average_method_frame_number;

						}







						//buf_right_edge_stereo_xyz.at<double>(0, 0) = road_edge_right_line_d_estimated_distance_using_moving_average;
						//buf_right_edge_stereo_xyz.at<double>(1, 0) = 0;
						//buf_right_edge_stereo_xyz.at<double>(2, 0) = 0;
						//
						//buf_right_edge__slam_map_xyz = mat_slam_R*buf_right_edge_stereo_xyz;
						//
						//buf_right_edge__slam_map_xyz.at<double>(0, 0) += slam_trajectory_d_Twc_X;
						//buf_right_edge__slam_map_xyz.at<double>(1, 0) += slam_trajectory_d_Twc_Y;
						//buf_right_edge__slam_map_xyz.at<double>(2, 0) += slam_trajectory_d_Twc_Z;











					}//if (n_flag_could_detect_right_road_edge_line > 0)

				}//if (road_edge_right_point.size() > 100)














				fprintf_s(fp_w_yolo_bounding_box_object_road_edge_line, "%d\t", n_frame_count);




				/// Left side ///




				//save road edge line data

				if (n_flag_could_detect_left_road_edge_line > 0) {



					buf_left_edge_stereo_xyz.at<double>(0, 0) = road_edge_left_line_d_estimated_distance_using_moving_average;
					buf_left_edge_stereo_xyz.at<double>(1, 0) = 0;
					buf_left_edge_stereo_xyz.at<double>(2, 0) = 0;

					buf_left_edge__slam_map_xyz = mat_slam_R*buf_left_edge_stereo_xyz;

					buf_left_edge__slam_map_xyz.at<double>(0, 0) += slam_trajectory_d_Twc_X;
					buf_left_edge__slam_map_xyz.at<double>(1, 0) += slam_trajectory_d_Twc_Y;
					buf_left_edge__slam_map_xyz.at<double>(2, 0) += slam_trajectory_d_Twc_Z;

					fprintf_s(fp_w_yolo_bounding_box_object_road_edge_line, "%lf\t%lf\t%d\t%lf\t%lf\t%lf\t", road_edge_left_line_d_estimated_distance, road_edge_left_line_d_estimated_distance_using_moving_average, road_edge_left_line_n_voted_count, buf_left_edge__slam_map_xyz.at<double>(0, 0), buf_left_edge__slam_map_xyz.at<double>(1, 0), buf_left_edge__slam_map_xyz.at<double>(2, 0));










					//save grids between left edge and z-axis





					n_road_edge_traveling_grid_num_x = int(fabs(road_edge_left_line_d_estimated_distance_using_moving_average) / d_road_edge_traveling_grid_pitch_x);


					for (int zz = 0; zz <= n_road_edge_traveling_grid_num_z; zz++) {
						for (int xx = 0; xx <= n_road_edge_traveling_grid_num_x; xx++) {




							buf_left_edge_stereo_xyz.at<double>(0, 0) = -xx*d_road_edge_traveling_grid_pitch_x;
							buf_left_edge_stereo_xyz.at<double>(1, 0) = 0;
							buf_left_edge_stereo_xyz.at<double>(2, 0) = zz*d_road_edge_traveling_grid_pitch_z;


							buf_left_edge__slam_map_xyz = mat_slam_R*buf_left_edge_stereo_xyz;

							buf_left_edge__slam_map_xyz.at<double>(0, 0) += slam_trajectory_d_Twc_X;
							buf_left_edge__slam_map_xyz.at<double>(1, 0) += slam_trajectory_d_Twc_Y;
							buf_left_edge__slam_map_xyz.at<double>(2, 0) += slam_trajectory_d_Twc_Z;






							fprintf_s(fp_w_grids_between_left_edge_and_right_edge, "%lf\t%lf\t%lf\n", buf_left_edge__slam_map_xyz.at<double>(0, 0), buf_left_edge__slam_map_xyz.at<double>(1, 0), buf_left_edge__slam_map_xyz.at<double>(2, 0));




							// daijkstra

							n_daijkstra_block_no_x = int((buf_left_edge__slam_map_xyz.at<double>(0, 0) - DAIJKSTRA_MAP_X_RANGE_MIN) / DAIJKSTRA_MAP_BLOCK_PITCH);
							n_daijkstra_block_no_z = int((buf_left_edge__slam_map_xyz.at<double>(2, 0) - DAIJKSTRA_MAP_Z_RANGE_MIN) / DAIJKSTRA_MAP_BLOCK_PITCH);





							if (n_daijkstra_block_no_x < DAIJKSTRA_MAP_X_BLOCK_NUM && n_daijkstra_block_no_x >= 0 && n_daijkstra_block_no_z < DAIJKSTRA_MAP_Z_BLOCK_NUM && n_daijkstra_block_no_z >= 0) {


								if (daijkstra_field_MAP[n_daijkstra_block_no_z][n_daijkstra_block_no_x] == 0) {


									daijkstra_field_MAP[n_daijkstra_block_no_z][n_daijkstra_block_no_x] = 1; /////<<<<<<<<<<<<<<<<<<<<<<<


									img_field_MAP.at<uchar>(n_daijkstra_block_no_z, n_daijkstra_block_no_x) = 255;




									//save daijkstra block info

									d_daijkstra_block_pos_x = (n_daijkstra_block_no_x - DAIJKSTRA_MAP_ORIGIN_X_BLOCK_NO) * DAIJKSTRA_MAP_BLOCK_PITCH;
									d_daijkstra_block_pos_z = (n_daijkstra_block_no_z - DAIJKSTRA_MAP_ORIGIN_Z_BLOCK_NO) * DAIJKSTRA_MAP_BLOCK_PITCH;
									d_daijkstra_block_pos_y = buf_left_edge__slam_map_xyz.at<double>(1, 0);




									fprintf_s(fp_w_daijkstra_blocks, "%d\t%d\t%lf\t%lf\t%lf\n", n_daijkstra_block_no_x, n_daijkstra_block_no_z, d_daijkstra_block_pos_x, d_daijkstra_block_pos_y, d_daijkstra_block_pos_z);
									//fprintf_s(fp_w_daijkstra_blocks, "%lf\t%lf\t%lf", d_daijkstra_block_pos_x, d_daijkstra_block_pos_y, d_daijkstra_block_pos_z);





									/*
									for (int k = 0; k < 9; k++) { //save dummy Rotation matrix data for daijkstra program

									fprintf_s(fp_w_daijkstra_blocks, "\t0");
									}

									fprintf_s(fp_w_daijkstra_blocks, "\n");
									*/




									//n_daijkstra_field_map_node_no++;  /////<<<<<<<<<<<<<<<<<<<<<<<



								}

							}//if daijkstra






















						}
					}








				}
				else {

					fprintf_s(fp_w_yolo_bounding_box_object_road_edge_line, "?\t?\t?\t?\t?\t?\t");
					//fprintf_s(fp_w_yolo_bounding_box_object_road_edge_line, "%d\t?\t?\t?\t?\t?\t?\t", n_frame_count);
				}






				/// Right side ///

				if (n_flag_could_detect_right_road_edge_line > 0) {





					buf_right_edge_stereo_xyz.at<double>(0, 0) = road_edge_right_line_d_estimated_distance_using_moving_average;
					buf_right_edge_stereo_xyz.at<double>(1, 0) = 0;
					buf_right_edge_stereo_xyz.at<double>(2, 0) = 0;

					buf_right_edge__slam_map_xyz = mat_slam_R*buf_right_edge_stereo_xyz;

					buf_right_edge__slam_map_xyz.at<double>(0, 0) += slam_trajectory_d_Twc_X;
					buf_right_edge__slam_map_xyz.at<double>(1, 0) += slam_trajectory_d_Twc_Y;
					buf_right_edge__slam_map_xyz.at<double>(2, 0) += slam_trajectory_d_Twc_Z;





					fprintf_s(fp_w_yolo_bounding_box_object_road_edge_line, "%lf\t%lf\t%d\t%lf\t%lf\t%lf\n", road_edge_right_line_d_estimated_distance, road_edge_right_line_d_estimated_distance_using_moving_average, road_edge_right_line_n_voted_count, buf_right_edge__slam_map_xyz.at<double>(0, 0), buf_right_edge__slam_map_xyz.at<double>(1, 0), buf_right_edge__slam_map_xyz.at<double>(2, 0));









					//save grids between z axis and right edge



					n_road_edge_traveling_grid_num_x = int(fabs(road_edge_right_line_d_estimated_distance_using_moving_average) / d_road_edge_traveling_grid_pitch_x);


					for (int zz = 0; zz <= n_road_edge_traveling_grid_num_z; zz++) {  ///////////<<<<<<<< 
						for (int xx = 0; xx <= n_road_edge_traveling_grid_num_x; xx++) {



							buf_right_edge_stereo_xyz.at<double>(0, 0) = xx*d_road_edge_traveling_grid_pitch_x;
							buf_right_edge_stereo_xyz.at<double>(1, 0) = 0;
							buf_right_edge_stereo_xyz.at<double>(2, 0) = zz*d_road_edge_traveling_grid_pitch_z;

							buf_right_edge__slam_map_xyz = mat_slam_R*buf_right_edge_stereo_xyz;

							buf_right_edge__slam_map_xyz.at<double>(0, 0) += slam_trajectory_d_Twc_X;
							buf_right_edge__slam_map_xyz.at<double>(1, 0) += slam_trajectory_d_Twc_Y;
							buf_right_edge__slam_map_xyz.at<double>(2, 0) += slam_trajectory_d_Twc_Z;



							fprintf_s(fp_w_grids_between_left_edge_and_right_edge, "%lf\t%lf\t%lf\n", buf_right_edge__slam_map_xyz.at<double>(0, 0), buf_right_edge__slam_map_xyz.at<double>(1, 0), buf_right_edge__slam_map_xyz.at<double>(2, 0));














							// daijkstra

							n_daijkstra_block_no_x = int((buf_right_edge__slam_map_xyz.at<double>(0, 0) - DAIJKSTRA_MAP_X_RANGE_MIN) / DAIJKSTRA_MAP_BLOCK_PITCH);
							n_daijkstra_block_no_z = int((buf_right_edge__slam_map_xyz.at<double>(2, 0) - DAIJKSTRA_MAP_Z_RANGE_MIN) / DAIJKSTRA_MAP_BLOCK_PITCH);






							if (n_daijkstra_block_no_x < DAIJKSTRA_MAP_X_BLOCK_NUM && n_daijkstra_block_no_x >= 0 && n_daijkstra_block_no_z < DAIJKSTRA_MAP_Z_BLOCK_NUM && n_daijkstra_block_no_z >= 0) {


								if (daijkstra_field_MAP[n_daijkstra_block_no_z][n_daijkstra_block_no_x] == 0) {

									daijkstra_field_MAP[n_daijkstra_block_no_z][n_daijkstra_block_no_x] = 1;


									img_field_MAP.at<uchar>(n_daijkstra_block_no_z, n_daijkstra_block_no_x) = 255;




									//save daijkstra block info

									d_daijkstra_block_pos_x = (n_daijkstra_block_no_x - DAIJKSTRA_MAP_ORIGIN_X_BLOCK_NO) * DAIJKSTRA_MAP_BLOCK_PITCH;
									d_daijkstra_block_pos_z = (n_daijkstra_block_no_z - DAIJKSTRA_MAP_ORIGIN_Z_BLOCK_NO) * DAIJKSTRA_MAP_BLOCK_PITCH;
									d_daijkstra_block_pos_y = buf_right_edge__slam_map_xyz.at<double>(1, 0);




									fprintf_s(fp_w_daijkstra_blocks, "%d\t%d\t%lf\t%lf\t%lf\n", n_daijkstra_block_no_x, n_daijkstra_block_no_z, d_daijkstra_block_pos_x, d_daijkstra_block_pos_y, d_daijkstra_block_pos_z);

									//fprintf_s(fp_w_daijkstra_blocks, "%lf\t%lf\t%lf", d_daijkstra_block_pos_x, d_daijkstra_block_pos_y, d_daijkstra_block_pos_z);





									/*
									for (int k = 0; k < 9; k++) { //save dummy Rotation matrix data for daijkstra program

									fprintf_s(fp_w_daijkstra_blocks, "\t0");
									}


									fprintf_s(fp_w_daijkstra_blocks, "\n");
									*/






									//n_daijkstra_field_map_node_no++;







								}

							}//if daijkstra














						}
					}







				}
				else {

					fprintf_s(fp_w_yolo_bounding_box_object_road_edge_line, "?\t?\t?\t?\t?\t?\n");
				}













				//vehicle self position

				d_daijkstra_vehicle_self_block_pos_x = slam_trajectory_d_Twc_X;
				d_daijkstra_vehicle_self_block_pos_y = slam_trajectory_d_Twc_Y;
				d_daijkstra_vehicle_self_block_pos_z = slam_trajectory_d_Twc_Z;


				n_daijkstra_vehicle_self_block_no_x = int((d_daijkstra_vehicle_self_block_pos_x - DAIJKSTRA_MAP_X_RANGE_MIN) / DAIJKSTRA_MAP_BLOCK_PITCH);
				n_daijkstra_vehicle_self_block_no_z = int((d_daijkstra_vehicle_self_block_pos_z - DAIJKSTRA_MAP_Z_RANGE_MIN) / DAIJKSTRA_MAP_BLOCK_PITCH);





				slam_xyz[n_current_frame_No].map_block_no_x = n_daijkstra_vehicle_self_block_no_x;
				slam_xyz[n_current_frame_No].map_block_no_z = n_daijkstra_vehicle_self_block_no_z;







				//fprintf_s(fp_w_daijkstra_vehicle_pos, "%lf\t%lf\t%lf\t%d\n", d_daijkstra_vehicle_self_block_pos_x, d_daijkstra_vehicle_self_block_pos_y, d_daijkstra_vehicle_self_block_pos_z, n_daijkstra_vehicle_self_pos_block_no);
				fprintf_s(fp_w_daijkstra_vehicle_pos, "%lf\t%lf\t%lf\t%d\t%d\n", slam_xyz[n_current_frame_No].x, slam_xyz[n_current_frame_No].y, slam_xyz[n_current_frame_No].z, slam_xyz[n_current_frame_No].map_block_no_x, slam_xyz[n_current_frame_No].map_block_no_z);


























				//clear memory

				if (road_edge_left_point.size() > 0) {

					road_edge_left_point.clear();
					road_edge_left_point.shrink_to_fit();
				}

				if (road_edge_right_point.size() > 0) {




					road_edge_right_point.clear();
					road_edge_right_point.shrink_to_fit();
				}




				if (road_edge_left_point_candidate.size() > 0) {
					road_edge_left_point_candidate.clear();
					road_edge_left_point_candidate.shrink_to_fit();

				}

				if (road_edge_right_point_candidate.size() > 0) {
					road_edge_right_point_candidate.clear();
					road_edge_right_point_candidate.shrink_to_fit();

				}








				if (img_points_on_road_center_line.size() > 0) {

					img_points_on_road_center_line.clear();
					img_points_on_road_center_line.shrink_to_fit();


				}















			}  //if bouding box






























			cv::imshow(win_dst0, img_feature_points);
			cv::imshow(win_dst_wall_mask, img_3D_plane);












		}//if yolo mode

















		 // writer 1 //



		img_cam1_remap.copyTo(img_view1);
		//img_cam2_remap.copyTo(img_view2);
		img_merge.copyTo(img_view2);
		img_3D_plane.copyTo(img_view3);
		img_feature_points.copyTo(img_view4);






		writer << img_combined;









		// writer 2 //

		cv::putText(img_cam1_remap, sFrameNo, cv::Point(10, 450), cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 255, 0), 1, CV_AA);


		img_cloud_remap.copyTo(img2_view1);
		img_wall_remap.copyTo(img2_view2);
		img_edge1_sobel_remap.copyTo(img2_view3);
		img_cam1_remap.copyTo(img2_view4);



		writer2 << img2_combined;















































		//thread saving 3D XYZ RGB data






		//saveXYZRGB(point_cloud_filename.c_str(), xyz, img_cam1_remap);

		//hThread = (HANDLE)_beginthreadex(NULL, 0, thread_save_XYZRGB, "save", 0, NULL);






		//cv::imshow(win_src_cam1, img_cam1);
		//cv::imshow(win_src_cam2, img_cam2);











		if (YOLO_mode == YOLO_UME || YOLO_mode == YOLO_OBJECTS) {


			// clear yolo buffer

			yolo_BBox.clear();
			yolo_BBox.shrink_to_fit();

















		}









		n_frame_count++;

		n_current_frame_No++;












		BufferPoolController* c = cv::ocl::getOpenCLAllocator()->getBufferPoolController();
		if (c) {
			c->setMaxReservedSize(0);
		}















	}//if (YOLO_mode == YOLO_UME || YOLO_mode == YOLO_OBJECTS) {






















































	n_flag_thread_gnuplot_exit = 1;



	CloseHandle(hThread_gnuplot);











	dwTime1 = timeGetTime();

	dwSpentTime = dwTime1 - dwTime0;

	dProcessingSpeed = n_frame_count / (double)dwSpentTime * 1000;
	dProcessingSpeed_3D_data_saving = n_frame_count_3D_data / (double)dwSpentTime * 1000;


	std::cout << "Processing speed:" << dProcessingSpeed << "Hz[" << n_frame_count << "files]\n" << "Saving speed:" << dProcessingSpeed_3D_data_saving << "Hz [" << n_frame_count_3D_data << "files]\n";







	/*
	//save daijkstra block data

	for (int j = 0; j < DAIJKSTRA_MAP_Z_BLOCK_NUM; j++) {
	for (int i = 0; i < DAIJKSTRA_MAP_X_BLOCK_NUM; i++) {


	if (daijkstra_field_MAP[j][i] > 0) {



	}



	}
	}

	*/
















	writer.release();
	writer2.release();










	//fclose(fp_w_obstacle);
	//fclose(fp_w_ditch);
	//fclose(fp_w_camera_height);
	//fclose(fp_w_surface_patch);
	fclose(fp_w_traveling_distance);



	fclose(fp_w_orb_slam_object);

	fclose(fp_orb_slam_trajectory);



	if (YOLO_mode == YOLO_UME || YOLO_mode == YOLO_OBJECTS) {

		fclose(fp_w_yolo_bounding_box_object_k_means);
		fclose(fp_w_yolo_bounding_box_object_of_interest);

	}





	if (YOLO_mode == YOLO_OBJECTS) {



		fclose(fp_w_yolo_bounding_box_object_road_edge_line);
		fclose(fp_w_grids_between_left_edge_and_right_edge);

		fclose(fp_w_daijkstra_blocks);

		fclose(fp_w_daijkstra_vehicle_pos);


		//img_field_MAP

		imwrite(sSaveFilePath_Daijkstra_field_map_image, img_field_MAP);



	}













	//save ume clustering results

	if (YOLO_mode == YOLO_UME) {

		cout << "DETECTED UME NUMBER : " << to_string((int)yolo_detected_Object.size()) << endl;

		for (int k = 0; k< (int)yolo_detected_Object.size(); k++) {

			//yolo_detected_Object[k].cx = yolo_detected_Object[k].sum_x / (double)yolo_detected_Object[k].counter;
			//yolo_detected_Object[k].cy = yolo_detected_Object[k].sum_y / (double)yolo_detected_Object[k].counter;
			//yolo_detected_Object[k].cz = yolo_detected_Object[k].sum_z / (double)yolo_detected_Object[k].counter;

			fprintf_s(fp_w_yolo_detected_object_NN, "%lf\t%lf\t%lf\n", yolo_detected_Object[k].cx, yolo_detected_Object[k].cy, yolo_detected_Object[k].cz);

		}

		fclose(fp_w_yolo_detected_object_NN);






		//yolo_detected_Object.clear();
		//yolo_detected_Object.shrink_to_fit();

	}



	//waitKey(0);








	return 0;

}







unsigned _stdcall thread_gnuplot(void *p)
{


	FILE *gid;


	gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");




	fprintf_s(gid, "set size ratio -1\n");
	fflush(gid);




	while (1) {







		if (YOLO_mode == YOLO_UME) {






			fprintf_s(gid, "plot '-' w l t 'SLAM MAP','-' w vectors t 'x-axis', '-' w vectors t 'z-axis', '-' w circles t 'Tree candidates', '-' w p pt 7 ps 2 t 'Detected Tree'\n");


			//draw trajectory
			int n_current_point_data_no = (int)slam_xyz.size() - 1;

			for (int k = 0; k < (int)slam_xyz.size(); k++) {

				fprintf(gid, "%lf\t%lf\n", slam_xyz[k].x, slam_xyz[k].z);

			}
			fprintf(gid, "e\n");



			//draw camera axis
			if ((int)slam_xyz.size() > 0) {

				fprintf(gid, "%lf\t%lf\t%lf\t%lf\n", slam_xyz[n_current_point_data_no].x, slam_xyz[n_current_point_data_no].z, camera_axis_1n.at<double>(0, 0), camera_axis_1n.at<double>(2, 0));
				fprintf(gid, "e\n");
				fprintf(gid, "%lf\t%lf\t%lf\t%lf\n", slam_xyz[n_current_point_data_no].x, slam_xyz[n_current_point_data_no].z, camera_axis_3n.at<double>(0, 0), camera_axis_3n.at<double>(2, 0));
				fprintf(gid, "e\n");

			}
			else {

				fprintf(gid, "0\t0\t1\t0\n");
				fprintf(gid, "e\n");
				fprintf(gid, "0\t0\t0\t1\n");
				fprintf(gid, "e\n");

			}




			//draw the circle at the tree candidates position
			for (int k = 0; k < (int)yolo_bbox_object_center.size(); k++) {
				//fprintf(gid, "%lf\t%lf\t%lf\n", yolo_bbox_object_center[k].x, yolo_bbox_object_center[k].z, yolo_bbox_object_center[k].d_boject_pixel_ratio_to_box);
				fprintf(gid, "%lf\t%lf\t0.1\n", yolo_bbox_object_center[k].x, yolo_bbox_object_center[k].z);
			}
			fprintf(gid, "e\n");


			//draw detected trees
			for (int k = 0; k < (int)yolo_detected_Object.size(); k++) {

				fprintf(gid, "%lf\t%lf\n", yolo_detected_Object[k].cx, yolo_detected_Object[k].cz);

			}
			fprintf(gid, "e\n");



		}
		//↓ココかな（動的なグラフの作成）
		else if (YOLO_mode == YOLO_OBJECTS) {



			fprintf_s(gid, "plot '-' w l t 'SLAM MAP','-' w vectors t 'x-axis', '-' w vectors t 'z-axis'\n");


			//draw trajectory
			int n_current_point_data_no = (int)slam_xyz.size() - 1;

			for (int k = 0; k < (int)slam_xyz.size(); k++) {

				fprintf(gid, "%lf\t%lf\n", slam_xyz[k].x, slam_xyz[k].z);

			}
			fprintf(gid, "e\n");



			//draw camera axis
			if ((int)slam_xyz.size() > 0) {

				fprintf(gid, "%lf\t%lf\t%lf\t%lf\n", slam_xyz[n_current_point_data_no].x, slam_xyz[n_current_point_data_no].z, camera_axis_1n.at<double>(0, 0), camera_axis_1n.at<double>(2, 0));
				fprintf(gid, "e\n");
				fprintf(gid, "%lf\t%lf\t%lf\t%lf\n", slam_xyz[n_current_point_data_no].x, slam_xyz[n_current_point_data_no].z, camera_axis_3n.at<double>(0, 0), camera_axis_3n.at<double>(2, 0));
				fprintf(gid, "e\n");

			}
			else {

				fprintf(gid, "0\t0\t1\t0\n");
				fprintf(gid, "e\n");
				fprintf(gid, "0\t0\t0\t1\n");
				fprintf(gid, "e\n");

			}




		}
		else if (YOLO_mode == NOT_USE_YOLO) {






			fprintf_s(gid, "plot '-' w l t 'SLAM MAP','-' w vectors t 'x-axis', '-' w vectors t 'z-axis'\n");


			//draw trajectory
			int n_current_point_data_no = (int)slam_xyz.size() - 1;

			for (int k = 0; k < (int)slam_xyz.size(); k++) {

				fprintf(gid, "%lf\t%lf\n", slam_xyz[k].x, slam_xyz[k].z);

			}
			fprintf(gid, "e\n");



			//draw camera axis
			if ((int)slam_xyz.size() > 0) {

				fprintf(gid, "%lf\t%lf\t%lf\t%lf\n", slam_xyz[n_current_point_data_no].x, slam_xyz[n_current_point_data_no].z, camera_axis_1n.at<double>(0, 0), camera_axis_1n.at<double>(2, 0));
				fprintf(gid, "e\n");
				fprintf(gid, "%lf\t%lf\t%lf\t%lf\n", slam_xyz[n_current_point_data_no].x, slam_xyz[n_current_point_data_no].z, camera_axis_3n.at<double>(0, 0), camera_axis_3n.at<double>(2, 0));
				fprintf(gid, "e\n");

			}
			else {

				fprintf(gid, "0\t0\t1\t0\n");
				fprintf(gid, "e\n");
				fprintf(gid, "0\t0\t0\t1\n");
				fprintf(gid, "e\n");

			}









		}//if yolo mode




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







			fprintf_s(fp, "%f\t%f\t%f\t%d\t%d\t%d\t%lf\t%lf\n", point[0], point[1], point[2], intensity[2], intensity[1], intensity[0], camera_coordinate_origin_based_distance.at<double>(y, x), ground_based_distance.at<double>(y, x));


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
	//	int k_x, k_y, k_z;
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



int calc_and_save_T_matrix_using_3D_points(const Mat& R_mat, const Mat& T_mat, const Mat& xyz_mat0, const Mat& xyz_mat1, const std::vector<cv::Point2d> &points0, const std::vector<cv::Point2d> &points1, const Mat& mask0, const Mat& mask1, int n_ransac_loop_times, double d_ransac_thresh, double *d_Tx_estimated, double *d_Ty_estimated, double *d_Tz_estimated, double *d_alpha_estimated, double *d_T_abs_estimated, int *ransac_n_T_voted_num)

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

int determine_ootsu_auto_thresh_in_bounding_box_gray_image(const Mat &img_gray, int nLTx, int nLTy, int nRBx, int nRBy, int histogram_tank[HISTOGRAM_LEVEL_MAX])
{
	int buf_gray;
	int opt_thresh;

	int n_num1, n_num2;
	double d_sum1, d_sum2;
	double d_avg1, d_avg2;
	double d_separability;
	double d_max_separability;



	//calc histogram

	for (int i = 0; i < HISTOGRAM_LEVEL_MAX; i++) {

		histogram_tank[i] = 0;
	}


	for (int j = nLTy; j <= nRBy; j++) {
		for (int i = nLTx; i <= nRBx; i++) {

			buf_gray = img_gray.at<uchar>(j, i);

			histogram_tank[buf_gray]++;

		}
	}



	//separability
	// n1*n2*(u1-u2)^2

	opt_thresh = 0;
	d_max_separability = 0;


	for (int k = 1; k < HISTOGRAM_LEVEL_MAX - 1; k++) {




		d_sum1 = 0;
		n_num1 = 0;

		for (int i = 0; i < k; i++) {

			d_sum1 += histogram_tank[i] * i;
			n_num1 += histogram_tank[i];

		}

		d_avg1 = d_sum1 / (double)n_num1;


		d_sum2 = 0;
		n_num2 = 0;

		for (int i = k; i < HISTOGRAM_LEVEL_MAX; i++) {

			d_sum2 += histogram_tank[i] * i;
			n_num2 += histogram_tank[i];

		}

		d_avg2 = d_sum2 / (double)n_num2;





		d_separability = n_num1 * n_num2 * (d_avg1 - d_avg2) * (d_avg1 - d_avg2);






		if (d_max_separability < d_separability) {

			d_max_separability = d_separability;
			opt_thresh = k;

		}




	}


	return opt_thresh;



}


int determine_primary_object_center_in_bounding_box_using_k_means(std::vector<struct yolo_bounding_box_cluster_point> &bb_point, int n_cluster_candidates_num, int n_clustering_loop_times, struct yolo_bounding_box_cluster_point *object_center)
{



	int n_point_num = (int)bb_point.size();

	//int n_point_num_lower_limit = 10;


	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<int> sample_generate(0, n_point_num - 1);




	std::vector<struct yolo_bounding_box_cluster_point> cluster_point;

	struct yolo_bounding_box_cluster_point buf_point;

	int n_sample_no;


	double d_distance;
	double d_distance_min;
	int n_cluster_candidates_no;



	int n_cluster_point_num_max;
	int n_points_max_cluster_id;



	int n_trial_times = 1;
	//int n_clustering_loop_times = 100;







	//set initial cluster using random sampling
	for (int i = 0; i < n_cluster_candidates_num; i++) {


		n_sample_no = sample_generate(mt);

		buf_point.x = bb_point[n_sample_no].x;
		buf_point.y = bb_point[n_sample_no].y;
		buf_point.z = bb_point[n_sample_no].z;

		buf_point.id = i;

		buf_point.count = 0;

		cluster_point.push_back(buf_point);


	}










	//do clustering


	for (int k = 0; k < n_clustering_loop_times; k++) {






		//reset counter

		for (int j = 0; j < n_cluster_candidates_num; j++) {

			cluster_point[j].count = 0;

			cluster_point[j].sum_x = 0;
			cluster_point[j].sum_y = 0;
			cluster_point[j].sum_z = 0;

		}





		//clustering for each point




		for (int i = 0; i < n_point_num; i++) {


			n_cluster_candidates_no = 0;

			d_distance_min = (bb_point[i].x - cluster_point[0].x)*(bb_point[i].x - cluster_point[0].x) + (bb_point[i].y - cluster_point[0].y)*(bb_point[i].y - cluster_point[0].y) + (bb_point[i].z - cluster_point[0].z)*(bb_point[i].z - cluster_point[0].z);
			//d_distance_min = (bb_point[i].x - cluster_point[0].x)*(bb_point[i].x - cluster_point[0].x) + (bb_point[i].z - cluster_point[0].z)*(bb_point[i].z - cluster_point[0].z);



			//search
			for (int j = 0; j < n_cluster_candidates_num; j++) {

				d_distance = (bb_point[i].x - cluster_point[j].x)*(bb_point[i].x - cluster_point[j].x) + (bb_point[i].y - cluster_point[j].y)*(bb_point[i].y - cluster_point[j].y) + (bb_point[i].z - cluster_point[j].z)*(bb_point[i].z - cluster_point[j].z);
				//d_distance = (bb_point[i].x - cluster_point[j].x)*(bb_point[i].x - cluster_point[j].x) + (bb_point[i].z - cluster_point[j].z)*(bb_point[i].z - cluster_point[j].z);

				if (d_distance_min > d_distance) {

					d_distance_min = d_distance;



					n_cluster_candidates_no = j;


				}


			}//search






			 //set id
			bb_point[i].id = n_cluster_candidates_no;

			cluster_point[n_cluster_candidates_no].count++;

			cluster_point[n_cluster_candidates_no].sum_x += bb_point[i].x;
			cluster_point[n_cluster_candidates_no].sum_y += bb_point[i].y;
			cluster_point[n_cluster_candidates_no].sum_z += bb_point[i].z;






		}//each point





		 //calc the center of each cluster

		n_cluster_point_num_max = -1;
		n_points_max_cluster_id = 0;


		for (int j = 0; j < n_cluster_candidates_num; j++) {


			//update max

			if (n_cluster_point_num_max < cluster_point[j].count) {


				n_cluster_point_num_max = cluster_point[j].count;

				n_points_max_cluster_id = j;


			}


			//update cluster center

			if (cluster_point[j].count == 0) continue;


			cluster_point[j].x = cluster_point[j].sum_x / (double)cluster_point[j].count;
			cluster_point[j].y = cluster_point[j].sum_y / (double)cluster_point[j].count;
			cluster_point[j].z = cluster_point[j].sum_z / (double)cluster_point[j].count;



		}








		object_center->x = cluster_point[n_points_max_cluster_id].x;
		object_center->y = cluster_point[n_points_max_cluster_id].y;
		object_center->z = cluster_point[n_points_max_cluster_id].z;
		object_center->count = n_cluster_point_num_max;






	}//loop times








	cluster_point.clear();
	cluster_point.shrink_to_fit();





	return 1;


}



void detect_road_edge_line_using_RANSAC(std::vector<struct yolo_bounding_box_point> &bb_point, int ransac_loop_times, double ransac_d_threshold_distance_to_vote, double *distance_from_camera_to_edge, int *ransac_voted_count)
{







	int n_point_num = (int)bb_point.size();




	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<int> sample_generate(0, n_point_num - 1);





	int n_sample_No[2];



	double edge_line_direction_vector[3];
	double point_vector[3];
	double origin_point_vector[3];
	double cross_vector_line_and_point[3];

	double edge_line_direction_vector_candidates[3];
	double edge_line_origin_candidates[3];


	double edge_line_direction_vector_abs;

	double d_distance_between_point_and_line;
	int ransac_n_voted_count;
	int ransac_n_voted_count_max;







	//ransac random sampling loop


	ransac_n_voted_count_max = -1;

	for (int k = 0; k < ransac_loop_times; k++) {





		//picking up two points to calc a straight line

		n_sample_No[0] = sample_generate(mt);




		while (1) {


			n_sample_No[1] = sample_generate(mt);




			if (n_sample_No[0] != n_sample_No[1]) break;



		}





		//calc the direction vector of the line

		edge_line_direction_vector[0] = bb_point[n_sample_No[1]].x - bb_point[n_sample_No[0]].x;
		edge_line_direction_vector[1] = bb_point[n_sample_No[1]].y - bb_point[n_sample_No[0]].y;
		edge_line_direction_vector[2] = bb_point[n_sample_No[1]].z - bb_point[n_sample_No[0]].z;

		edge_line_direction_vector_abs = sqrt(edge_line_direction_vector[0] * edge_line_direction_vector[0] + edge_line_direction_vector[1] * edge_line_direction_vector[1] + edge_line_direction_vector[2] * edge_line_direction_vector[2]);

		edge_line_direction_vector[0] /= edge_line_direction_vector_abs;
		edge_line_direction_vector[1] /= edge_line_direction_vector_abs;
		edge_line_direction_vector[2] /= edge_line_direction_vector_abs;


		origin_point_vector[0] = bb_point[n_sample_No[0]].x;
		origin_point_vector[1] = bb_point[n_sample_No[0]].y;
		origin_point_vector[2] = bb_point[n_sample_No[0]].z;







		//calc distance between each bb point and the line

		ransac_n_voted_count = 0;

		for (int i = 0; i < n_point_num; i++) {

			point_vector[0] = bb_point[i].x - bb_point[n_sample_No[0]].x;
			point_vector[1] = bb_point[i].y - bb_point[n_sample_No[0]].y;
			point_vector[2] = bb_point[i].z - bb_point[n_sample_No[0]].z;

			//calc cross product
			cross_vector_line_and_point[0] = edge_line_direction_vector[1] * point_vector[2] - edge_line_direction_vector[2] * point_vector[1];
			cross_vector_line_and_point[1] = edge_line_direction_vector[2] * point_vector[0] - edge_line_direction_vector[0] * point_vector[2];
			cross_vector_line_and_point[2] = edge_line_direction_vector[0] * point_vector[1] - edge_line_direction_vector[1] * point_vector[0];

			//

			//calc distance
			d_distance_between_point_and_line = fabs(cross_vector_line_and_point[0] * cross_vector_line_and_point[0] + cross_vector_line_and_point[1] * cross_vector_line_and_point[1] + cross_vector_line_and_point[2] * cross_vector_line_and_point[2]);


			if (d_distance_between_point_and_line < ransac_d_threshold_distance_to_vote) ransac_n_voted_count++;



		}


		if (ransac_n_voted_count_max < ransac_n_voted_count) {

			//backup the line parameters
			for (int j = 0; j < 3; j++) {

				edge_line_direction_vector_candidates[j] = edge_line_direction_vector[j];
				edge_line_origin_candidates[j] = origin_point_vector[j];
			}


			ransac_n_voted_count_max = ransac_n_voted_count;


		}


	}//ransac random sampling loop



	 //calc distance between origin point and camera origin
	point_vector[0] = -edge_line_origin_candidates[0];
	point_vector[1] = -edge_line_origin_candidates[1];
	point_vector[2] = -edge_line_origin_candidates[2];

	cross_vector_line_and_point[0] = edge_line_direction_vector_candidates[1] * point_vector[2] - edge_line_direction_vector_candidates[2] * point_vector[1];
	cross_vector_line_and_point[1] = edge_line_direction_vector_candidates[2] * point_vector[0] - edge_line_direction_vector_candidates[0] * point_vector[2];
	cross_vector_line_and_point[2] = edge_line_direction_vector_candidates[0] * point_vector[1] - edge_line_direction_vector_candidates[1] * point_vector[0];


	d_distance_between_point_and_line = fabs(cross_vector_line_and_point[0] * cross_vector_line_and_point[0] + cross_vector_line_and_point[1] * cross_vector_line_and_point[1] + cross_vector_line_and_point[2] * cross_vector_line_and_point[2]);




	*distance_from_camera_to_edge = d_distance_between_point_and_line;

	*ransac_voted_count = ransac_n_voted_count_max;

}



void detect_road_edge_line_using_VotingMethod(std::vector<struct yolo_bounding_box_point> &bb_point, double road_edge_distance_range_min, double road_edge_distance_range_max, double road_edge_distance_resolution, int thresh_voted_count, int * flag_could_detect_edge, double *distance_from_camera_to_edge, int *voted_count)
{





	int n_distance_level_num;
	int n_vote_box_pos;

	int n_vote_max;
	int n_vote_max_pos;



	int n_point_num = (int)bb_point.size();




	n_distance_level_num = (int)((road_edge_distance_range_max - road_edge_distance_range_min) / road_edge_distance_resolution) + 1;




	vector<int> vote_ARRAY(n_distance_level_num, 0);
	vector<double> vote_ARRAY_SUM(n_distance_level_num, 0);



	double d_shift_x_position;



	for (int i = 0; i < n_point_num; i++) {








		d_shift_x_position = bb_point[i].x - road_edge_distance_range_min;

		if (d_shift_x_position < 0) continue;


		n_vote_box_pos = (int)(d_shift_x_position / road_edge_distance_resolution);




		/*
		n_vote_box_pos = (int)(fabs(bb_point[i].x) / road_edge_distance_resolution);

		if (n_vote_box_pos < 0) {
		printf("The parameter of n_vote_box_pos must be positive");
		exit(-1);
		}

		*/





		if (n_vote_box_pos >= n_distance_level_num) {
			continue;
		}





		vote_ARRAY[n_vote_box_pos]++;
		vote_ARRAY_SUM[n_vote_box_pos] += bb_point[i].x;





	}









	n_vote_max = -1;

	for (int k = 0; k < n_distance_level_num; k++) {


		if (n_vote_max < vote_ARRAY[k]) {

			n_vote_max = vote_ARRAY[k];

			n_vote_max_pos = k;

		}

	}



	if (n_vote_max > thresh_voted_count) {

		*flag_could_detect_edge = 1;

		*distance_from_camera_to_edge = vote_ARRAY_SUM[n_vote_max_pos] / (double)n_vote_max;

		*voted_count = n_vote_max;
	}
	else {
		*flag_could_detect_edge = -1;

	}



}


void detect_road_center_line_in_image_use_RANSAC(vector<struct interested_point_xy> &center_line_image_points, int ransac_loop_times, double thresh_distance_between_point_and_line, struct interested_point_xy *center_line_org, double center_line_vector[2], double center_line_perpendicular_vector[2])
{

	int n_points_num = (int)center_line_image_points.size();

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<int> sample_generate(0, n_points_num - 1);






	if (n_points_num == 1) {



		center_line_org->x = center_line_image_points[0].x;
		center_line_org->y = center_line_image_points[0].y;

		center_line_vector[0] = 0;
		center_line_vector[1] = -1;



	}
	else if (n_points_num == 2) {

		center_line_org->x = (center_line_image_points[0].x + center_line_image_points[1].x) / 2;
		center_line_org->y = (center_line_image_points[0].y + center_line_image_points[1].y) / 2;

		center_line_vector[0] = 0;
		center_line_vector[1] = -1;


	}
	else if (n_points_num > 2) {






		//struct interested_point_xy y_max_point;
		//struct interested_point_xy y_min_point;





		int n_sample_No[2];
		int ransac_n_voted_count;
		int ransac_n_voted_count_max;





		double vector_line[2];
		double vector_line2[2];
		double vector_abs;
		double vector_origin[2];
		double vector_point[2];

		double vector_line_candidate[2];
		double vector_line_candidate2[2];
		double vector_origin_candidate[2];

		double d_distance;



		vector<struct interested_point_xy> point_xy_candidates;
		struct interested_point_xy buf_point_xy_candidate;


		int pca_data_num;





		ransac_n_voted_count_max = -1;


		for (int k = 0; k < ransac_loop_times; k++) {


			//randomly sampling

			n_sample_No[0] = sample_generate(mt);

			while (1) {


				n_sample_No[1] = sample_generate(mt);




				if (n_sample_No[0] != n_sample_No[1]) break;



			}




			vector_line[0] = center_line_image_points[n_sample_No[1]].x - center_line_image_points[n_sample_No[0]].x;
			vector_line[1] = center_line_image_points[n_sample_No[1]].y - center_line_image_points[n_sample_No[0]].y;

			vector_abs = sqrt(vector_line[0] * vector_line[0] + vector_line[1] * vector_line[1]);

			vector_line[0] /= vector_abs;
			vector_line[1] /= vector_abs;

			vector_line2[0] = -vector_line[1];
			vector_line2[1] = vector_line[0];


			vector_origin[0] = center_line_image_points[n_sample_No[0]].x;
			vector_origin[1] = center_line_image_points[n_sample_No[0]].y;






			//voting

			ransac_n_voted_count = 0;


			for (int i = 0; i < n_points_num; i++) {


				vector_point[0] = center_line_image_points[i].x - vector_origin[0];
				vector_point[1] = center_line_image_points[i].y - vector_origin[1];



				//d_distance = fabs(vector_line[0] * vector_point[1] - vector_line[1] * vector_point[0]);
				d_distance = fabs(vector_point[0] * vector_line2[0] + vector_point[1] * vector_line2[1]);

				if (d_distance < thresh_distance_between_point_and_line) {

					ransac_n_voted_count++;

				}


			}



			//update the candidate
			if (ransac_n_voted_count_max < ransac_n_voted_count) {


				ransac_n_voted_count_max = ransac_n_voted_count;

				vector_line_candidate[0] = vector_line[0];
				vector_line_candidate[1] = vector_line[1];
				vector_line_candidate2[0] = vector_line2[0];//perpendicular vector
				vector_line_candidate2[1] = vector_line2[1];

				vector_origin_candidate[0] = vector_origin[0];
				vector_origin_candidate[1] = vector_origin[1];



			}




		}



		//extract the point candidates to estimate the center line


		for (int i = 0; i < n_points_num; i++) {


			vector_point[0] = center_line_image_points[i].x - vector_origin_candidate[0];
			vector_point[1] = center_line_image_points[i].y - vector_origin_candidate[1];



			//d_distance = fabs(vector_line_candidate[0] * vector_point[1] - vector_line_candidate[1] * vector_point[0]);
			d_distance = fabs(vector_point[0] * vector_line_candidate2[0] + vector_point[1] * vector_line_candidate2[1]);


			if (d_distance < thresh_distance_between_point_and_line) {



				buf_point_xy_candidate.x = center_line_image_points[i].x;
				buf_point_xy_candidate.y = center_line_image_points[i].y;

				point_xy_candidates.push_back(buf_point_xy_candidate);




			}


		}






		pca_data_num = (int)point_xy_candidates.size();












		Mat pca_2D_SRC(pca_data_num, 2, CV_32FC1);
		Mat pca_projection;
		Mat pca_eigen_vectors, pca_eigen_values;
		Mat pca_eigen_org;





		for (int i = 0; i < pca_data_num; i++) {

			pca_2D_SRC.at<float>(i, 0) = (float)point_xy_candidates[i].x;
			pca_2D_SRC.at<float>(i, 1) = (float)point_xy_candidates[i].y;

		}










		// PCA analysis //

		cv::PCA pca(pca_2D_SRC, cv::Mat(), CV_PCA_DATA_AS_ROW, 2);

		pca_eigen_vectors = pca.eigenvectors.clone();
		pca_eigen_values = pca.eigenvalues.clone();
		pca_eigen_org = pca.mean.clone();











		// eigen vector
		center_line_vector[0] = pca_eigen_vectors.at<float>(0, 0);
		center_line_vector[1] = pca_eigen_vectors.at<float>(0, 1);


		if (center_line_vector[1] > 0) {
			center_line_vector[0] = -center_line_vector[0];
			center_line_vector[1] = -center_line_vector[1];
		}


		center_line_org->x = (int)pca_eigen_org.at<float>(0, 0);
		center_line_org->y = (int)pca_eigen_org.at<float>(0, 1);









		if (point_xy_candidates.size() > 0) {

			point_xy_candidates.clear();
			point_xy_candidates.shrink_to_fit();
		}




	}//if point num








	center_line_perpendicular_vector[0] = -center_line_vector[1];
	center_line_perpendicular_vector[1] = center_line_vector[0];








}
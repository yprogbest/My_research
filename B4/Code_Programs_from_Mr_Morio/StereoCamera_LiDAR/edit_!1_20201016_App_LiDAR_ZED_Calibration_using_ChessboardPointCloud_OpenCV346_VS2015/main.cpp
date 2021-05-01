#include <iostream>
#include <cmath>
#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>
#include <random>
#include "Eigen/Core"
#include "Eigen/Eigen"

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/ximgproc.hpp>





#define PI	3.141592

#define CHESSBOARD_SQUARE_SIZE		0.10 //[m]
#define CHESSBOARD_SQUARE_W			8
#define CHESSBOARD_SQUARE_H			11
#define CHESSBOARD_WIDTH			CHESSBOARD_SQUARE_W*CHESSBOARD_SQUARE_SIZE
#define CHESSBOARD_HEIGHT			CHESSBOARD_SQUARE_H*CHESSBOARD_SQUARE_SIZE



#define IMG_XSIZE 672
#define IMG_YSIZE 376




struct point_cloud
{
	double x;
	double y;
	double z;
	double intensity;
	int label;

};

struct point_cloud_vector_type {

	double x;
	double y;
	double z;
};









using namespace std;
using namespace cv;
using namespace cv::ximgproc;








string make_n_digit_number(int in_No, int n_digit_num);
double calc_min(double a, double b);

void clustering_point_cloud(std::vector<struct point_cloud> &point_cloud, double dist_thresh, int *cluster_class_num);
void eigen_estimate_point_cloud_plain_using_ransac(std::vector<struct point_cloud> &points, double d_ransac_distance_thresh_between_point_and_base_plane, std::vector<struct point_cloud> &points_candidates, Eigen::Vector3d& output_eigen_origin, Eigen::Vector3d& output_eigen_values, Eigen::Matrix3d& output_eigen_vectors);









int main()
{


	FILE *fp;
	FILE *fp_w;
	FILE *fp_w_hist;
	FILE *fp_w_chessboard_image_corners;
	
	errno_t err;

	//std::string sMainFolder;
	//std::string sFname;
	std::string sFilePath;
	std::string sFilePath_StereoImage;
	std::string sSaveFilePath;
	
	std::string sInputImageFilePath;
	std::string sSaveInteisityHistogramFilePath;
	std::string sSaveChessboardImageCornersFilePath;
	








	// camera calibration chessboard size //

	const int BOARD_CORNER_NUM_W = CHESSBOARD_SQUARE_W - 1;
	const int BOARD_CORNER_NUM_H = CHESSBOARD_SQUARE_H - 1;

	const Size BOARD_SIZE = Size(BOARD_CORNER_NUM_W, BOARD_CORNER_NUM_H);
	const int N_CORNERS = BOARD_CORNER_NUM_W * BOARD_CORNER_NUM_H;
	
	const float SCALE = (float)CHESSBOARD_SQUARE_SIZE; // [m]

	const Size IM_SIZE = Size(IMG_XSIZE, IMG_YSIZE);


	
	// the names of display windows //

	string win_src_cam1 = "cam1";
	string win_src_cam2 = "cam2";




	


	bool found_cam1;
	bool found_cam2;



	
	Mat img_stereo = Mat::zeros(IMG_YSIZE, IMG_XSIZE*2, CV_8UC3);
	Mat img_cam1 = Mat::zeros(IMG_YSIZE, IMG_XSIZE, CV_8UC3);
	Mat img_cam2 = Mat::zeros(IMG_YSIZE, IMG_XSIZE, CV_8UC3);
	Mat img_cam1_remap;
	Mat img_cam2_remap;

	Mat img_cam1_gray;
	Mat img_cam1_gray_remap;
	Mat img_cam2_gray_remap;
	Mat img_cam1_wrk;
	Mat img_cam1_dst;
	Mat img_cam2_dst;







	int key;
	


	int n_corner_No;
	int n_corner_i;
	int n_corner_j;
	int n_corner_base_No = BOARD_CORNER_NUM_W * BOARD_CORNER_NUM_H - BOARD_CORNER_NUM_W;



	// [ 2D ] ZED image //
	
	vector<Point2f> imageCorners1, imageCorners2;
	vector<Point2f> imageCorners1_LiDAR,imageCorners2_LiDAR; // change the order of the corner Nos.
	Point2f buf_image_corner1;
	Point2f buf_image_corner2;

	vector<Point2f> imagePoints_LiDAR2ZED;

	

	// [ 3D ] LiDAR //

	std::vector<struct point_cloud> point_cloud;
	struct point_cloud point;

	std::vector<Point3f> point_cloud_LiDAR_yzx;
	cv::Point3f buf_LiDAR_yzx;

	std::vector<Point3f> point_cloud_LiDAR2ZED;
	cv::Point3f buf_LiDAR2ZED_yzx;

	vector<Point3f>objectCorners;
	vector<Point3f>objectCorners_yzx;
	Point3f buf_object_corner;
	

	// [ 3D ] ZED STEREO //
	vector<Point3f> objectCorners_ZED_STEREO;



	


	Mat K1, K2;
	Mat D1, D2;
	vector<Mat> rvecs1, rvecs2;
	vector<Mat> tvecs1, tvecs2;

	Mat R, F, E;
	Vec3d T;

	Mat R1, R2, P1, P2;
	Mat Q;

	Mat map11, map12, map21, map22;



	

	imageCorners1_LiDAR.resize(BOARD_CORNER_NUM_W*BOARD_CORNER_NUM_H);
	imageCorners2_LiDAR.resize(BOARD_CORNER_NUM_W*BOARD_CORNER_NUM_H);





	// load camera parameters //

	string sFilePath_stereo_calibration_parameters;


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

	fs_stereo_param["R1"] >> R1;//
	fs_stereo_param["R2"] >> R2;//
	fs_stereo_param["P1"] >> P1;//
	fs_stereo_param["P2"] >> P2;//
	fs_stereo_param["Q"] >> Q;//

	fs_stereo_param.release();


	std::cout << "The stereo camera parameters have been loaded!\n";

	std::cout << "K1:" << endl;
	std::cout << K1 << endl;
	std::cout << "K2:" << endl;
	std::cout << K2 << endl;
	std::cout << "D1:" << endl;
	std::cout << D1 << endl;
	std::cout << "D2:" << endl;
	std::cout << D2 << endl;
	std::cout << "P1:" << endl;
	std::cout << P1 << endl;
	std::cout << "P2" << endl;
	std::cout << P2 << endl;
	std::cout << "Q" << endl;
	std::cout << Q << endl;







	initUndistortRectifyMap(K1, D1, R1, P1, IM_SIZE, CV_16SC2, map11, map12);
	initUndistortRectifyMap(K2, D2, R2, P2, IM_SIZE, CV_16SC2, map21, map22);








	// input chessboard information //
	
	std::cout << endl;
	std::cout << "[STEREO CAMERA IMAGE] file path = ";
	std::cin >> sFilePath_StereoImage;

	

	img_stereo = imread(sFilePath_StereoImage, 1);
	

	img_cam1 = img_stereo(cv::Rect(0, 0, img_stereo.cols / 2, img_stereo.rows));
	img_cam2 = img_stereo(cv::Rect(img_stereo.cols / 2, 0, img_stereo.cols / 2,img_stereo.rows));
	
	





	cv::namedWindow(win_src_cam1, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(win_src_cam2, CV_WINDOW_AUTOSIZE);



	cv::imshow(win_src_cam1, img_cam1);
	cv::imshow(win_src_cam2, img_cam2);


	std::cout << "Hit any key to continue... " << endl;
	cv::waitKey(0);








	remap(img_cam1, img_cam1_remap, map11, map12, INTER_LINEAR);
	remap(img_cam2, img_cam2_remap, map21, map22, INTER_LINEAR);


	found_cam1 = findChessboardCorners(img_cam1_remap, BOARD_SIZE, imageCorners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	found_cam2 = findChessboardCorners(img_cam2_remap, BOARD_SIZE, imageCorners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);



	if (found_cam1 == true && found_cam2 == true) {



		// open for saving corners //

		sSaveChessboardImageCornersFilePath = sFilePath_StereoImage + "_chessboard_corners_image_coordinates_and_corner_No.txt";

		fopen_s(&fp_w_chessboard_image_corners, sSaveChessboardImageCornersFilePath.c_str(), "wt");





		std::cout << endl;
		std::cout << "The corners could be detected successfully." << endl;



		cv::cvtColor(img_cam1_remap, img_cam1_gray_remap, CV_BGR2GRAY);
		cv::cvtColor(img_cam2_remap, img_cam2_gray_remap, CV_BGR2GRAY);

		cornerSubPix(img_cam1_gray_remap, imageCorners1, Size(9, 9), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.1));
		cornerSubPix(img_cam2_gray_remap, imageCorners2, Size(9, 9), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.1));


		img_cam1_dst = img_cam1_remap.clone();
		img_cam2_dst = img_cam2_remap.clone();


		drawChessboardCorners(img_cam1_dst, BOARD_SIZE, imageCorners1, found_cam1);
		drawChessboardCorners(img_cam2_dst, BOARD_SIZE, imageCorners2, found_cam2);



		cv::imshow(win_src_cam1, img_cam1_dst);
		cv::imshow(win_src_cam2, img_cam2_dst);


		cv::waitKey(1);








		for (int i = 0; i < imageCorners1.size(); i++) {



			n_corner_i = i % BOARD_CORNER_NUM_W;
			n_corner_j = i / BOARD_CORNER_NUM_W;

			n_corner_No = n_corner_base_No - BOARD_CORNER_NUM_W * n_corner_j + n_corner_i;


			buf_image_corner1.x = imageCorners1[i].x;
			buf_image_corner1.y = imageCorners1[i].y;


			imageCorners1_LiDAR[n_corner_No] = buf_image_corner1;


			fprintf_s(fp_w_chessboard_image_corners, "%f\t%f\t%d\n", imageCorners1[i].x, imageCorners1[i].y, n_corner_No);




		}




		fprintf_s(fp_w_chessboard_image_corners, "\n\n");




		for (int i = 0; i < imageCorners2.size(); i++) {



			n_corner_i = i % BOARD_CORNER_NUM_W;
			n_corner_j = i / BOARD_CORNER_NUM_W;

			n_corner_No = n_corner_base_No - BOARD_CORNER_NUM_W * n_corner_j + n_corner_i;


			buf_image_corner2.x = imageCorners2[i].x;
			buf_image_corner2.y = imageCorners2[i].y;


			imageCorners2_LiDAR[n_corner_No] = buf_image_corner2;


			fprintf_s(fp_w_chessboard_image_corners, "%f\t%f\t%d\n", imageCorners2[i].x, imageCorners2[i].y, n_corner_No);




		}




		fclose(fp_w_chessboard_image_corners);


		//imagePoints1.push_back(imageCorners1_LiDAR);
		//imagePoints2.push_back(imageCorners2_LiDAR);







		// save xyz coordinates //

		string sSaveFilePath_stereo_3D;

		sSaveFilePath_stereo_3D = sFilePath_StereoImage + "_output_chessboard_corners_xyz.txt";

		FILE* fp_w_xyz_chessboard;


		fopen_s(&fp_w_xyz_chessboard, sSaveFilePath_stereo_3D.c_str(), "wt");






		// calc 3D position of the corners and save the chessboard grid points estimated by traiangulation //


		Mat result_4D; // 4 x 1
		vector<Point3f> result_3D;
		
		Point3f buf_3D;




		for (size_t i = 0; i < imageCorners1_LiDAR.size(); i++) {


			triangulatePoints(P1, P2, Mat(imageCorners1_LiDAR[i]), Mat(imageCorners2_LiDAR[i]), result_4D);
			

			//result_4D rows:4 cols:4
			//4D (x0, x1, x2, x3) -> 3D (x0/x3, x1/x3, x2/x3)

			convertPointsFromHomogeneous(result_4D.reshape(4, 1), result_3D);


			buf_3D = result_3D[0];

			objectCorners_ZED_STEREO.push_back(buf_3D);


			fprintf_s(fp_w_xyz_chessboard, "%f\t%f\t%f\n", buf_3D.x, buf_3D.y, buf_3D.z);




		}




		fclose(fp_w_xyz_chessboard);

	}
	else {

		std::cout << "couldn't find chessboard corners." << endl;
		exit(1);
	}
	















	/////////////////////////////////////////////
	//                 LIDAR                   //
	/////////////////////////////////////////////

	


	double d_z_range_mini;








	std::cout << "Filepath for LiDAR point cloud data = ";
	std::cin >> sFilePath;


	std::cout << "The z range mini = ";
	std::cin >> d_z_range_mini;



	err = fopen_s(&fp, sFilePath.c_str(), "rt");

	if (err != 0) {

		std::cout << "Cannot open %s " << sFilePath << std::endl;

		exit(1);
	}


	while (fscanf_s(fp, "%lf\t%lf\t%lf\t%lf", &point.x, &point.y, &point.z, &point.intensity) != EOF) {


		if (point.z < d_z_range_mini) continue;


		point_cloud.push_back(point);

		printf("%lf\t%lf\t%lf\t%lf\n", point.x, point.y, point.z, point.intensity);




	}

	fclose(fp);






	// calc intensity histogram //


	int n_intensity_histogram_level_num = 256;
	std::vector<int> histogram_Inteinsity;


	histogram_Inteinsity.resize(n_intensity_histogram_level_num);



	for (int i = 0; i < n_intensity_histogram_level_num; i++) {

		histogram_Inteinsity[i] = 0;
	}


	for (size_t i = 0; i < point_cloud.size(); i++) {


		histogram_Inteinsity[(int)(point_cloud[i].intensity)]++;


	}




	// save intensity histogram



	sSaveInteisityHistogramFilePath = sFilePath + "_inteisity_histogram.txt";

	fopen_s(&fp_w_hist, sSaveInteisityHistogramFilePath.c_str(), "wt");


	for (int i = 0; i < n_intensity_histogram_level_num; i++) {

		fprintf_s(fp_w_hist, "%d\t%d\n", i, histogram_Inteinsity[i]);
	}


	fclose(fp_w_hist);







	// thresholding for intensity of the chessboard black and white regions //

	double d_thresh_intensity;

	std::cout << "Thresholding value for intensity to segment the black region and white regions of the chessboard\n= ";
	std::cin >> d_thresh_intensity;





	std::string sSaveInteisityBlackWhiteSegmentsFilePath;
	FILE *fp_w_black_white_segments;


	sSaveInteisityBlackWhiteSegmentsFilePath = sFilePath + "_intensity_based_balck0_and_white1_regions_thresh" + std::to_string(d_thresh_intensity) + ".txt";

	fopen_s(&fp_w_black_white_segments, sSaveInteisityBlackWhiteSegmentsFilePath.c_str(), "wt");





	for (size_t i = 0; i < point_cloud.size(); i++) {




		if (point_cloud[i].intensity > d_thresh_intensity) {

			fprintf_s(fp_w_black_white_segments, "%lf\t%lf\t%lf\t%d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, 1);
		}
		else {

			fprintf_s(fp_w_black_white_segments, "%lf\t%lf\t%lf\t%d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, 0);
		}




	}

	fclose(fp_w_black_white_segments);

















	// clustering of point cloud //





	double dist_thresh;
	int n_clustering_class_num;



	std::cout << "[CLUSTERING] Thresholding value for distance in labeling points = ";
	std::cin >> dist_thresh;









	clustering_point_cloud(point_cloud, dist_thresh, &n_clustering_class_num);












	// calc histogram of point labels //



	std::vector<int> cluster_points_histogram;

	cluster_points_histogram.resize(n_clustering_class_num);






	for (int i = 0; i < n_clustering_class_num; i++) {

		cluster_points_histogram[i] = 0;

	}



	for (size_t i = 0; i < point_cloud.size(); i++) {

		cluster_points_histogram[point_cloud[i].label]++;

	}










	std::string sSavePointLabelHistogramFilePath;
	FILE *fp_w_point_label_hist;


	sSavePointLabelHistogramFilePath = sFilePath + "_clustering_point_label_histogram_DistTh" + std::to_string(dist_thresh) + ".txt";

	fopen_s(&fp_w_point_label_hist, sSavePointLabelHistogramFilePath.c_str(), "wt");



	for (int i = 0; i < n_clustering_class_num; i++) {

		fprintf(fp_w_point_label_hist, "%d\t%d\n", i, cluster_points_histogram[i]);

	}


	fclose(fp_w_point_label_hist);











	// find max class //


	int n_cluster_max_points;
	int n_max_points_cluster_no;

	n_cluster_max_points = -1;

	for (int i = 0; i < n_clustering_class_num; i++) {

		if (n_cluster_max_points < cluster_points_histogram[i]) {

			n_cluster_max_points = cluster_points_histogram[i];
			n_max_points_cluster_no = i;
		}

	}


	std::cout << std::endl;
	std::cout << "Largest point class No. and the number of points" << std::endl;
	std::cout << n_max_points_cluster_no << "\t" << n_cluster_max_points << std::endl;
	std::cout << std::endl;












	// Pick upping the points of the largest segment and save results
	std::vector<struct point_cloud> chessboard_points;
	struct point_cloud buf_chessboard_point;


	sSaveFilePath = sFilePath + "_clustering_points_and_label_distTh" + std::to_string(dist_thresh) + ".txt";

	fopen_s(&fp_w, sSaveFilePath.c_str(), "wt");


	//for (int i = 0; i < n_cluster_num; i++) {
	//	fprintf_s(fp_w, "%d\t%d\n", i, cluster_points_histogram[i]);
	//}





	for (size_t i = 0; i < point_cloud.size(); i++) {


		fprintf(fp_w, "%lf\t%lf\t%lf\t%d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, point_cloud[i].label);


	}


	fprintf(fp_w, "\n\n");


	for (size_t i = 0; i < point_cloud.size(); i++) {



		if (point_cloud[i].label == n_max_points_cluster_no) {




			buf_chessboard_point = point_cloud[i];
			chessboard_points.push_back(buf_chessboard_point);


			fprintf(fp_w, "%lf\t%lf\t%lf\t%d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, point_cloud[i].label);



		}


	}



	fclose(fp_w);










	// estimate the plain of point cloud using ransac method //







	std::vector<struct point_cloud> chessboard_points_candidates;

	double d_ransac_distance_thresh_between_point_and_base_plain;

	Eigen::Vector3d eigen_origin;
	Eigen::Vector3d eigen_values;
	Eigen::Matrix3d eigen_vectors;



	std::cout << "[RANSAC ESTIMATE PLAIN] Thresholding value for distance between a point and base plain = ";
	std::cin >> d_ransac_distance_thresh_between_point_and_base_plain;










	eigen_estimate_point_cloud_plain_using_ransac(chessboard_points, d_ransac_distance_thresh_between_point_and_base_plain, chessboard_points_candidates, eigen_origin, eigen_values, eigen_vectors);


	// eigen values and vectors
	// eigen_value[0] < eigen_value[1] < eigen_value[2]
	// 
	// check the directions of the 2nd and 3rd eigen vectors
	if (eigen_vectors(1, 1) > 0) eigen_vectors.col(1) = -eigen_vectors.col(1);
	if (eigen_vectors(2, 2) < 0) eigen_vectors.col(2) = -eigen_vectors.col(2);








	FILE *fp_w_ransac_points;
	std::string sSaveRansacPointsFilePath;
	sSaveRansacPointsFilePath = sFilePath + "_ransac_points_distTh" + std::to_string(d_ransac_distance_thresh_between_point_and_base_plain) + ".txt";


	fopen_s(&fp_w_ransac_points, sSaveRansacPointsFilePath.c_str(), "wt");




	for (int j = 0; j < chessboard_points_candidates.size(); j++) {

		fprintf_s(fp_w_ransac_points, "%lf\t%lf\t%lf\n", chessboard_points_candidates[j].x, chessboard_points_candidates[j].y, chessboard_points_candidates[j].z);


	}



	fclose(fp_w_ransac_points);









	FILE *fp_w_ransac_plane;
	std::string sSaveRansacPlaneFilePath;
	sSaveRansacPlaneFilePath = sFilePath + "_ransac_palne_vector_distTh" + std::to_string(d_ransac_distance_thresh_between_point_and_base_plain) + ".txt";

	fopen_s(&fp_w_ransac_plane, sSaveRansacPlaneFilePath.c_str(), "wt");


	fprintf_s(fp_w_ransac_plane, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", eigen_origin(0), eigen_origin(1), eigen_origin(2), eigen_vectors(0, 0), eigen_vectors(1, 0), eigen_vectors(2, 0));
	fprintf_s(fp_w_ransac_plane, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", eigen_origin(0), eigen_origin(1), eigen_origin(2), eigen_vectors(0, 1), eigen_vectors(1, 1), eigen_vectors(2, 1));
	fprintf_s(fp_w_ransac_plane, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", eigen_origin(0), eigen_origin(1), eigen_origin(2), eigen_vectors(0, 2), eigen_vectors(1, 2), eigen_vectors(2, 2));
	//fprintf_s(fp_w_ransac_plane, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", vec_avg(0), vec_avg(1), vec_avg(2), ransac_base_plane_vec_candidate[0].x, ransac_base_plane_vec_candidate[0].y, ransac_base_plane_vec_candidate[0].z);
	//fprintf_s(fp_w_ransac_plane, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", vec_avg(0), vec_avg(1), vec_avg(2), ransac_base_plane_vec_candidate[1].x, ransac_base_plane_vec_candidate[1].y, ransac_base_plane_vec_candidate[1].z);
	//fprintf_s(fp_w_ransac_plane, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", vec_avg(0), vec_avg(1), vec_avg(2), ransac_base_plane_vec_candidate[2].x, ransac_base_plane_vec_candidate[2].y, ransac_base_plane_vec_candidate[2].z);



	fclose(fp_w_ransac_plane);









	// Projection of points onto eigenspace//
	int n_chessboard_point_num;

	n_chessboard_point_num = (int)chessboard_points_candidates.size();



	Eigen::MatrixXd Mat_chessboard = Eigen::MatrixXd::Zero(n_chessboard_point_num, 3);
	Eigen::MatrixXd Mat_chessboard_proj = Eigen::MatrixXd::Zero(n_chessboard_point_num, 3);


	for (int j = 0; j < n_chessboard_point_num; j++) {



		Mat_chessboard(j, 0) = chessboard_points_candidates[j].x - eigen_origin(0);
		Mat_chessboard(j, 1) = chessboard_points_candidates[j].y - eigen_origin(1);
		Mat_chessboard(j, 2) = chessboard_points_candidates[j].z - eigen_origin(2);


	}


	//projection of points

	Mat_chessboard_proj = Mat_chessboard * eigen_vectors;







	FILE *fp_w_eigen_projection;
	std::string sSaveEigenProjectionFilePath;
	sSaveEigenProjectionFilePath = sFilePath + "_eigen_projection_distTh" + std::to_string(d_ransac_distance_thresh_between_point_and_base_plain) + ".txt";

	fopen_s(&fp_w_eigen_projection, sSaveEigenProjectionFilePath.c_str(), "wt");



	for (int j = 0; j < n_chessboard_point_num; j++) {


		fprintf_s(fp_w_eigen_projection, "%lf\t%lf\t%lf\n", Mat_chessboard_proj(j, 0), Mat_chessboard_proj(j, 1), Mat_chessboard_proj(j, 2));


	}



	fclose(fp_w_eigen_projection);








	Eigen::Vector2d eigen_board_LB_corner_pos;

	
	double board_x_min = -CHESSBOARD_WIDTH / 2.0;
	double board_x_max = CHESSBOARD_WIDTH / 2.0;
	double board_y_min = -CHESSBOARD_HEIGHT / 2.0;
	double board_y_max = CHESSBOARD_HEIGHT / 2.0;

	double d_lidar_point_translated_x;
	double d_lidar_point_translated_y;
	double d_lidar_point_x;
	double d_lidar_point_y;
	int n_lidar_point_x_id;
	int n_lidar_point_y_id;
	int n_lidar_point_pos_no;
	int n_chessboard_black_white_color;



	double d_translate_x_y_step = 0.001;
	int n_translate_x_y_step_num = 35;
	double d_translate_x;
	double d_translate_y;

	double d_rotation_theta_step = 0.1;
	int n_rotation_theta_step_num = 15;
	double d_rotation;
	double d_rotation_radian;



	int n_chessboard_black_white_color_matching_score;
	int n_matching_score_max;
	int n_mismatching_score;
	double d_matching_max_score_translate_x;
	double d_matching_max_score_translate_y;
	double d_matching_max_score_rotation;
	double d_matching_max_score_rotation_radian;

	//double d_point_x0, d_point_x1;
	//double d_point_y0, d_point_y1;
	//double d_point_x_dist1, d_point_x_dist2;
	//double d_point_y_dist1, d_point_y_dist2;
	//double d_cost_dist;
	//double d_cost_energy;
	//double d_cost_energy_min;


	FILE *fp_w_matching;
	std::string sSaveMatchingScoreFilePath;
	sSaveMatchingScoreFilePath = sFilePath + "_matching_score_x_y_plus_minus" + std::to_string(d_translate_x_y_step*n_translate_x_y_step_num) + ".txt";

	fopen_s(&fp_w_matching, sSaveMatchingScoreFilePath.c_str(), "wt");



	n_matching_score_max = -n_chessboard_point_num;
	//d_cost_energy_min = 1000000;


	for (int j = -n_translate_x_y_step_num; j <= n_translate_x_y_step_num; j++) {

		for (int i = -n_translate_x_y_step_num; i <= n_translate_x_y_step_num; i++) {


			d_translate_x = i * d_translate_x_y_step;
			d_translate_y = j * d_translate_x_y_step;



			for (int r = -n_rotation_theta_step_num; r <= n_rotation_theta_step_num; r++) {


				d_rotation = r * d_rotation_theta_step;
				d_rotation_radian = d_rotation / 180.0 * PI;






				n_chessboard_black_white_color_matching_score = 0;
				n_mismatching_score = 0;
				//d_cost_energy = 0;


				for (int k = 0; k < n_chessboard_point_num; k++) {


					d_lidar_point_translated_x = Mat_chessboard_proj(k, 1) + d_translate_x;
					d_lidar_point_translated_y = Mat_chessboard_proj(k, 2) + d_translate_y;


					d_lidar_point_x = d_lidar_point_translated_x*cos(d_rotation_radian) + d_lidar_point_translated_y*sin(d_rotation_radian);
					d_lidar_point_y = -d_lidar_point_translated_x*sin(d_rotation_radian) + d_lidar_point_translated_y*cos(d_rotation_radian);




					if (d_lidar_point_x < board_x_min || d_lidar_point_x > board_x_max || d_lidar_point_y < board_y_min || d_lidar_point_y > board_y_max) {
						
						n_mismatching_score++;
						continue;
					
					}
					

					n_lidar_point_x_id = int((d_lidar_point_x - board_x_min) / CHESSBOARD_SQUARE_SIZE);
					n_lidar_point_y_id = int((d_lidar_point_y - board_y_min) / CHESSBOARD_SQUARE_SIZE);

					//black->white->black->white


					n_lidar_point_pos_no = n_lidar_point_x_id + n_lidar_point_y_id;


					if (n_lidar_point_pos_no % 2 == 0) n_chessboard_black_white_color = 0; //0
					else n_chessboard_black_white_color = 1;



					// count the number of points that the color is matched //
					if (chessboard_points_candidates[k].label == n_chessboard_black_white_color) n_chessboard_black_white_color_matching_score++;





					/*
					// calc cost // 

					d_point_x0 = n_lidar_point_x_id * CHESSBOARD_SQUARE_SIZE + board_x_min;
					d_point_y0 = n_lidar_point_y_id * CHESSBOARD_SQUARE_SIZE + board_y_min;
					d_point_x1 = (n_lidar_point_x_id + 1) * CHESSBOARD_SQUARE_SIZE + board_x_min;
					d_point_y1 = (n_lidar_point_y_id + 1) * CHESSBOARD_SQUARE_SIZE + board_y_min;

					d_point_x_dist1 = d_lidar_point_x - d_point_x0;
					d_point_x_dist2 = d_point_x1 - d_lidar_point_x;

					d_point_y_dist1 = d_lidar_point_y - d_point_y0;
					d_point_y_dist2 = d_point_y1 - d_lidar_point_y;

					//if (d_point_x_dist1 < 0 || d_point_x_dist2 < 0 || d_point_y_dist1 < 0 || d_point_y_dist2 < 0) {
					//	std::cout << "ERROR!" << endl;
					//	exit(1);
					//}


					if (chessboard_points_candidates[k].label == n_chessboard_black_white_color) {

						d_cost_dist = 0;
					}
					else {
						d_cost_dist = calc_min(d_point_x_dist1, d_point_x_dist2) + calc_min(d_point_y_dist1, d_point_y_dist2);
					}

					d_cost_energy += d_cost_dist;

					*/








				}//loop k: each point




				
				 
				 // check mismatching score //

				n_chessboard_black_white_color_matching_score -= n_mismatching_score;

				//if (n_chessboard_black_white_color_matching_score < 0) n_chessboard_black_white_color_matching_score = 0;




				// update matching socre //
				
				if (n_matching_score_max < n_chessboard_black_white_color_matching_score) {

					n_matching_score_max = n_chessboard_black_white_color_matching_score;
					
					d_matching_max_score_translate_x = d_translate_x;
					d_matching_max_score_translate_y = d_translate_y;
					d_matching_max_score_rotation = d_rotation;
					d_matching_max_score_rotation_radian = d_rotation_radian;

				}


				fprintf_s(fp_w_matching, "%lf\t%lf\t%lf\t%d\n", d_translate_x, d_translate_y, d_rotation, n_chessboard_black_white_color_matching_score);



				

				/*
				if (d_cost_energy_min > d_cost_energy) {

					d_cost_energy_min = d_cost_energy;


					d_matching_max_score_translate_x = d_translate_x;
					d_matching_max_score_translate_y = d_translate_y;
					d_matching_max_score_rotation = d_rotation;
					d_matching_max_score_rotation_radian = d_rotation_radian;


				}

				fprintf_s(fp_w_matching, "%lf\t%lf\t%lf\t%lf\n", d_translate_x, d_translate_y, d_rotation, d_cost_energy);
				*/


				//std::cout << "energy:" << d_cost_energy_min << std::endl;




			}//loop r


		}//loop i

	}//loop j





	std::cout << std::endl;
	std::cout << "[MATCHING SCORE] score max = " << n_matching_score_max << "  translate (x,y) = (" << d_matching_max_score_translate_x << " , " << d_matching_max_score_translate_y << ") rotation = " << d_matching_max_score_rotation << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;


	fclose(fp_w_matching);









	double d_Px;
	double d_Py;
	double d_Px2;
	double d_Py2;

	Eigen::Vector3d chessboard_grid_point;
	Eigen::Vector3d chessboard_grid_point_out;



	FILE *fp_w_board_grid;
	std::string sSaveBoardGridFilePath;
	sSaveBoardGridFilePath = sFilePath + "_estimated_chessboard_grids.txt";

	fopen_s(&fp_w_board_grid, sSaveBoardGridFilePath.c_str(), "wt");


	//Eigen::Matrix3d eigen_vectors_T = eigen_vectors.transpose();
	//Eigen::Matrix3d eigen_inv = eigen_vectors_T.inverse();






	for (int j = 0; j <= CHESSBOARD_SQUARE_H; j++) {
		for (int i = 0; i <= CHESSBOARD_SQUARE_W; i++) {


			d_Px = i * CHESSBOARD_SQUARE_SIZE - CHESSBOARD_WIDTH / 2.0;
			d_Py = j * CHESSBOARD_SQUARE_SIZE - CHESSBOARD_HEIGHT / 2.0;


			d_Px2 = d_Px * cos(d_matching_max_score_rotation_radian) - d_Py*sin(d_matching_max_score_rotation_radian);
			d_Py2 = d_Px * sin(d_matching_max_score_rotation_radian) + d_Py*cos(d_matching_max_score_rotation_radian);

			d_Px2 -= d_matching_max_score_translate_x;
			d_Py2 -= d_matching_max_score_translate_y;


			chessboard_grid_point(0) = 0;
			chessboard_grid_point(1) = d_Px2;// d_Px2;
			chessboard_grid_point(2) = d_Py2;// d_Py2;


			chessboard_grid_point_out = eigen_vectors * chessboard_grid_point;
			//chessboard_grid_point_out = eigen_inv * chessboard_grid_point;

			chessboard_grid_point_out += eigen_origin;


			fprintf_s(fp_w_board_grid, "%lf\t%lf\t%lf\n", chessboard_grid_point_out(0), chessboard_grid_point_out(1), chessboard_grid_point_out(2));
			//fprintf_s(fp_w_board_grid, "%lf\t%lf\t%lf\n", chessboard_grid_point(0), chessboard_grid_point(1), chessboard_grid_point(2));

		}
	}


	fclose(fp_w_board_grid);








	
	



	FILE *fp_w_calibration_chessboard_grid;
	std::string sSaveCalibrationBoardGridFilePath;
	sSaveCalibrationBoardGridFilePath = sFilePath + "_chessboard_grids_for_LiDAR_Cam_Calibration.txt";

	fopen_s(&fp_w_calibration_chessboard_grid, sSaveCalibrationBoardGridFilePath.c_str(), "wt");


	int n_corner_count;


	n_corner_count = 0;

	for (int j = 1; j < CHESSBOARD_SQUARE_H; j++) {
		for (int i = 1; i < CHESSBOARD_SQUARE_W; i++) {


			d_Px = i * CHESSBOARD_SQUARE_SIZE - CHESSBOARD_WIDTH / 2.0;
			d_Py = j * CHESSBOARD_SQUARE_SIZE - CHESSBOARD_HEIGHT / 2.0;


			d_Px2 = d_Px * cos(d_matching_max_score_rotation_radian) - d_Py*sin(d_matching_max_score_rotation_radian);
			d_Py2 = d_Px * sin(d_matching_max_score_rotation_radian) + d_Py*cos(d_matching_max_score_rotation_radian);

			d_Px2 -= d_matching_max_score_translate_x;
			d_Py2 -= d_matching_max_score_translate_y;


			chessboard_grid_point(0) = 0;
			chessboard_grid_point(1) = d_Px2;// d_Px2;
			chessboard_grid_point(2) = d_Py2;// d_Py2;


			chessboard_grid_point_out = eigen_vectors * chessboard_grid_point;
			//chessboard_grid_point_out = eigen_inv * chessboard_grid_point;

			chessboard_grid_point_out += eigen_origin;


			buf_object_corner.x = (float)chessboard_grid_point_out(0);
			buf_object_corner.y = (float)chessboard_grid_point_out(1);
			buf_object_corner.z = (float)chessboard_grid_point_out(2);

			objectCorners.push_back(buf_object_corner);


			fprintf_s(fp_w_calibration_chessboard_grid, "%d\t%lf\t%lf\t%lf\t%f\t%f\n", n_corner_count, chessboard_grid_point_out(0), chessboard_grid_point_out(1), chessboard_grid_point_out(2), imageCorners1_LiDAR[n_corner_count].x, imageCorners1_LiDAR[n_corner_count].y);
			//fprintf_s(fp_w_board_grid, "%lf\t%lf\t%lf\n", chessboard_grid_point(0), chessboard_grid_point(1), chessboard_grid_point(2));



			n_corner_count++;


		}
	}


	//objectPoints.push_back(objectCorners);







	fclose(fp_w_calibration_chessboard_grid);









	


	// LiDAR <-> ZED Stereo 
	
	// LiDAR: objectCorners
	// ZED: objectCorners_ZED_STEREO
	//
	//

	int projection_matrix_n_data_set_num = BOARD_CORNER_NUM_W * BOARD_CORNER_NUM_H;
	int projection_param_num = 9;

	Eigen::MatrixXd M_LiDar2Zed = Eigen::MatrixXd::Zero(projection_matrix_n_data_set_num * 3, projection_param_num);
	Eigen::MatrixXd M_LiDar2Zed_trans = Eigen::MatrixXd::Zero(projection_param_num, projection_matrix_n_data_set_num * 3);
	Eigen::MatrixXd M_Zed = Eigen::MatrixXd::Zero(projection_matrix_n_data_set_num * 3, 1);
	Eigen::MatrixXd P_LiDar2Zed = Eigen::MatrixXd::Zero(projection_param_num, 1);

	Eigen::MatrixXd MtM_LiDar2Zed = Eigen::MatrixXd::Zero(projection_param_num, projection_param_num);
	Eigen::MatrixXd MtZed = Eigen::MatrixXd::Zero(projection_param_num, 1);

	Eigen::Vector3d objectCorners_center_ZED_STEREO;
	Eigen::Vector3d objectCorners_yzx_center;
	Eigen::Vector3d point_shift_T_LiDAR2ZED;


	// XYZ coordinate system //
	//
	// LiDAR xyz  => (-y)(-z)(x)
	//

	objectCorners_yzx.resize(objectCorners.size());

	for (int i = 0; i < objectCorners.size(); i++) {

		objectCorners_yzx[i].x = -objectCorners[i].y;
		objectCorners_yzx[i].y = -objectCorners[i].z;
		objectCorners_yzx[i].z = objectCorners[i].x;

	}









	

	// calc center of LiDAR points //

	objectCorners_yzx_center = Eigen::Vector3d::Zero();


	for (int i = 0; i < objectCorners.size(); i++) {

		objectCorners_yzx_center(0) += objectCorners_yzx[i].x;
		objectCorners_yzx_center(1) += objectCorners_yzx[i].y;
		objectCorners_yzx_center(2) += objectCorners_yzx[i].z;
	}

	objectCorners_yzx_center(0) /= objectCorners.size();
	objectCorners_yzx_center(1) /= objectCorners.size();
	objectCorners_yzx_center(2) /= objectCorners.size();




	// calc center of ZED points //

	objectCorners_center_ZED_STEREO = Eigen::Vector3d::Zero();

	for (int i = 0; i < objectCorners_ZED_STEREO.size(); i++) {

		objectCorners_center_ZED_STEREO(0) += objectCorners_ZED_STEREO[i].x;
		objectCorners_center_ZED_STEREO(1) += objectCorners_ZED_STEREO[i].y;
		objectCorners_center_ZED_STEREO(2) += objectCorners_ZED_STEREO[i].z;

	}

	objectCorners_center_ZED_STEREO(0) /= objectCorners_ZED_STEREO.size();
	objectCorners_center_ZED_STEREO(1) /= objectCorners_ZED_STEREO.size();
	objectCorners_center_ZED_STEREO(2) /= objectCorners_ZED_STEREO.size();



	point_shift_T_LiDAR2ZED = objectCorners_center_ZED_STEREO - objectCorners_yzx_center;




	

	for (int i = 0; i < objectCorners.size(); i++) {



		M_LiDar2Zed(3 * i, 0) = objectCorners_yzx[i].x + point_shift_T_LiDAR2ZED(0);
		M_LiDar2Zed(3 * i, 1) = objectCorners_yzx[i].y + point_shift_T_LiDAR2ZED(1);
		M_LiDar2Zed(3 * i, 2) = objectCorners_yzx[i].z + point_shift_T_LiDAR2ZED(2);
		
		M_Zed(3 * i, 0) = objectCorners_ZED_STEREO[i].x;


		M_LiDar2Zed(3 * i + 1, 3) = objectCorners_yzx[i].x + point_shift_T_LiDAR2ZED(0);
		M_LiDar2Zed(3 * i + 1, 4) = objectCorners_yzx[i].y + point_shift_T_LiDAR2ZED(1);
		M_LiDar2Zed(3 * i + 1, 5) = objectCorners_yzx[i].z + point_shift_T_LiDAR2ZED(2);
		
		M_Zed(3 * i + 1, 0) = objectCorners_ZED_STEREO[i].y;


		M_LiDar2Zed(3 * i + 2, 6) = objectCorners_yzx[i].x + point_shift_T_LiDAR2ZED(0);
		M_LiDar2Zed(3 * i + 2, 7) = objectCorners_yzx[i].y + point_shift_T_LiDAR2ZED(1);
		M_LiDar2Zed(3 * i + 2, 8) = objectCorners_yzx[i].z + point_shift_T_LiDAR2ZED(2);
		
		M_Zed(3 * i + 2, 0) = objectCorners_ZED_STEREO[i].z;


	}


	M_LiDar2Zed_trans = M_LiDar2Zed.transpose();

	MtM_LiDar2Zed = M_LiDar2Zed_trans * M_LiDar2Zed;
	MtZed = M_LiDar2Zed_trans * M_Zed;

	P_LiDar2Zed = MtM_LiDar2Zed.inverse() * MtZed;

	




	Eigen::MatrixXd lidar_to_zed_P(3, 3);
	Eigen::MatrixXd lidar_point_IN(3, 1);
	Eigen::MatrixXd lidar_point_OUT(3, 1);

	



	for (int j = 0; j < 3; j++) {

		lidar_to_zed_P(0, j) = P_LiDar2Zed(j);
		lidar_to_zed_P(1, j) = P_LiDar2Zed(3 + j);
		lidar_to_zed_P(2, j) = P_LiDar2Zed(6 + j);

	}



	std::cout << endl;
	std::cout << endl;
	std::cout << "---- LiDAR -> ZED projection matrix parameter ----" << endl;
	std::cout << endl;

	std::cout << lidar_to_zed_P << endl;



	




	// projection of the corners of LiDAR into ZED coordinates

	FILE *fp_w_estimated_lidar_points_to_zed_points;
	std::string sSaveFilePath_estimated_lidar_points_to_zed_points;

	sSaveFilePath_estimated_lidar_points_to_zed_points = sFilePath + "_estimated_lidar_points_to_zed_points.txt";

	fopen_s(&fp_w_estimated_lidar_points_to_zed_points, sSaveFilePath_estimated_lidar_points_to_zed_points.c_str(), "wt");


	img_cam1_dst = img_cam1_remap.clone();




	// chessboard corners 3D

	for (int j = 0; j < objectCorners.size(); j++) {

		lidar_point_IN(0, 0) = objectCorners_yzx[j].x + point_shift_T_LiDAR2ZED(0);
		lidar_point_IN(1, 0) = objectCorners_yzx[j].y + point_shift_T_LiDAR2ZED(1);
		lidar_point_IN(2, 0) = objectCorners_yzx[j].z + point_shift_T_LiDAR2ZED(2);
		

		lidar_point_OUT = lidar_to_zed_P * lidar_point_IN;


		
		fprintf_s(fp_w_estimated_lidar_points_to_zed_points, "%f\t%f\t%f\t%lf\t%lf\t%lf\n", objectCorners_ZED_STEREO[j].x, objectCorners_ZED_STEREO[j].y, objectCorners_ZED_STEREO[j].z, lidar_point_OUT(0, 0), lidar_point_OUT(1, 0), lidar_point_OUT(2, 0));

		
		//cv::circle(img_cam1_dst, cv::Point(lidar_point_OUT(0, 0), lidar_point_OUT(1, 0)), 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);



	}


	
	


	fprintf_s(fp_w_estimated_lidar_points_to_zed_points, "\n\n");

	// point cloud 3D

	for (int j = 0; j < point_cloud.size(); j++) {

		buf_LiDAR_yzx.x = (float)(-point_cloud[j].y);
		buf_LiDAR_yzx.y = (float)(-point_cloud[j].z);
		buf_LiDAR_yzx.z = (float)( point_cloud[j].x);

		point_cloud_LiDAR_yzx.push_back(buf_LiDAR_yzx);

		lidar_point_IN(0, 0) = buf_LiDAR_yzx.x + point_shift_T_LiDAR2ZED(0);
		lidar_point_IN(1, 0) = buf_LiDAR_yzx.y + point_shift_T_LiDAR2ZED(1);
		lidar_point_IN(2, 0) = buf_LiDAR_yzx.z + point_shift_T_LiDAR2ZED(2);
		

		lidar_point_OUT = lidar_to_zed_P * lidar_point_IN;


		fprintf_s(fp_w_estimated_lidar_points_to_zed_points, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", lidar_point_IN(0, 0), lidar_point_IN(1, 0), lidar_point_IN(2, 0), lidar_point_OUT(0, 0), lidar_point_OUT(1, 0), lidar_point_OUT(2, 0));





	}
	





	fclose(fp_w_estimated_lidar_points_to_zed_points);






	// lidar points -> zed points

	cv::Mat mat_R_LiDAR2ZED = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::Mat mat_T_LiDAR2ZED = cv::Mat::zeros(3, 1, CV_64FC1);


	for (int j = 0; j < 3; j++) {

		mat_T_LiDAR2ZED.at<double>(j, 0) = point_shift_T_LiDAR2ZED(j);

		for (int i = 0; i < 3; i++) {

			mat_R_LiDAR2ZED.at<double>(j,i) = lidar_to_zed_P(j, i);

		}
	}






	

	Eigen::Matrix3d K1_mat;


	std::cout << endl;
	std::cout << endl;

	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++) {

			K1_mat(j, i) = K1.at<double>(j, i);

			std::cout << K1_mat(j, i) << " ";
		}

		std::cout << endl;
	}

	










	



	//set camera parameters //

	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	
	

	

	



	img_cam1_dst = img_cam1_remap.clone();


	

	for (int j = 0; j < point_cloud.size(); j++) {


		buf_LiDAR2ZED_yzx.x = (float)(-point_cloud[j].y);
		buf_LiDAR2ZED_yzx.y = (float)(-point_cloud[j].z);
		buf_LiDAR2ZED_yzx.z = (float)(point_cloud[j].x);

		lidar_point_IN(0, 0) = buf_LiDAR2ZED_yzx.x + point_shift_T_LiDAR2ZED(0);
		lidar_point_IN(1, 0) = buf_LiDAR2ZED_yzx.y + point_shift_T_LiDAR2ZED(1);
		lidar_point_IN(2, 0) = buf_LiDAR2ZED_yzx.z + point_shift_T_LiDAR2ZED(2);


		

		lidar_point_OUT = lidar_to_zed_P * lidar_point_IN;


		buf_LiDAR2ZED_yzx.x = (float)lidar_point_OUT(0, 0);
		buf_LiDAR2ZED_yzx.y = (float)lidar_point_OUT(1, 0);
		buf_LiDAR2ZED_yzx.z = (float)lidar_point_OUT(2, 0);

	


		point_cloud_LiDAR2ZED.push_back(buf_LiDAR2ZED_yzx);


	
		

	}

	






	// solvePnP //

	
	//point_cloud -> image corrdinates ; directly //

	cv::solvePnP(objectCorners_yzx, imageCorners1_LiDAR, K1, D1, rvec, tvec);
	cv::projectPoints(point_cloud_LiDAR_yzx, rvec, tvec, K1, D1, imagePoints_LiDAR2ZED);



	// point_cloud -> camera coordinates -> image coordinates //

	//cv::solvePnP(objectCorners_ZED_STEREO, imageCorners1_LiDAR, K1, D1, rvec, tvec);
	//cv::projectPoints(point_cloud_LiDAR2ZED, rvec, tvec, K1, D1, imagePoints_LiDAR2ZED);
	








	// save the image coordinates of of the lidar points //


	FILE *fp_w_estimated_lidar_points_on_image;
	std::string sSaveFilePath_estimated_lidar_points_on_image;

	sSaveFilePath_estimated_lidar_points_on_image = sFilePath + "_estimated_lidar_points_on_image.txt";

	fopen_s(&fp_w_estimated_lidar_points_on_image, sSaveFilePath_estimated_lidar_points_on_image.c_str(), "wt");





	
	for (int j = 0; j < imagePoints_LiDAR2ZED.size(); j++) {


		fprintf_s(fp_w_estimated_lidar_points_on_image, "%f\t%f\n", imagePoints_LiDAR2ZED[j].x, imagePoints_LiDAR2ZED[j].y);
		
		cv::circle(img_cam1_dst, cv::Point((int)imagePoints_LiDAR2ZED[j].x, (int)imagePoints_LiDAR2ZED[j].y), 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);


	}
	

	
	
	

	fclose(fp_w_estimated_lidar_points_on_image);






	// save output image //

	std::string sSaveFilePath_image;
	sSaveFilePath_image = sFilePath_StereoImage + "_PnP_projected_points.png";


	imwrite(sSaveFilePath_image, img_cam1_dst);








	// LiDAR, ZED calibration //


	std::string sSaveFilePath_PnP_rvec_tvec;
	sSaveFilePath_PnP_rvec_tvec = sFilePath + "_solve_PnP_K_D_rvec_tvec.yaml";


	cv::FileStorage fs_rvec_tvec(sSaveFilePath_PnP_rvec_tvec, FileStorage::WRITE);


	fs_rvec_tvec << "K1" << K1;
	fs_rvec_tvec << "D1" << D1;
	fs_rvec_tvec << "rvec" << rvec;
	fs_rvec_tvec << "tvec" << tvec;

	fs_rvec_tvec << "mat_T_LiDAR2ZED" << mat_T_LiDAR2ZED;
	fs_rvec_tvec << "mat_R_LiDAR2ZED" << mat_R_LiDAR2ZED;


	fs_rvec_tvec.release();














		
	
	
	
	
	std::cout << "Hit any key to continue ... " << endl;


	cv::imshow(win_src_cam1, img_cam1_dst);
	cv::waitKey(0);





	




	




	//capture1.release();
	cv::destroyAllWindows();

	

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








void clustering_point_cloud(std::vector<struct point_cloud> &point_cloud, double dist_thresh, int *cluster_class_num)
{






	// point cloud clustering //

	double distance;



	int n_cluster_num;



	int n_neighboring_point_num;
	int n_neighboring_point_label_no_min;

	std::vector<int> lookup_table_point_label;
	lookup_table_point_label.resize(int(point_cloud.size()));

	std::vector<int> neighboring_point_label_buffer;
	neighboring_point_label_buffer.resize(int(point_cloud.size()));








	// initialize the label of every point //

	for (size_t i = 0; i < point_cloud.size(); i++) {

		point_cloud[i].label = -1;

	}


	// initialize the lookup table to label a point //

	for (int i = 0; i < point_cloud.size(); i++) {
		lookup_table_point_label[i] = i;
	}








	n_cluster_num = 0;
	point_cloud[0].label = 0;


	n_cluster_num++;


	for (size_t i = 1; i < point_cloud.size(); i++) { //0




													  // serch the neighboring points around a targeted point //

		n_neighboring_point_num = 0;
		n_neighboring_point_label_no_min = int(point_cloud.size());

		for (size_t k = 0; k < i; k++) {//for (size_t k = 0; k < point_cloud.size(); k++) {


										//if (i == k) continue;


			distance = (point_cloud[i].x - point_cloud[k].x)*(point_cloud[i].x - point_cloud[k].x) + (point_cloud[i].y - point_cloud[k].y)*(point_cloud[i].y - point_cloud[k].y) + (point_cloud[i].z - point_cloud[k].z)*(point_cloud[i].z - point_cloud[k].z);
			distance = sqrt(distance);


			if (distance < dist_thresh && point_cloud[k].label >= 0) {


				neighboring_point_label_buffer[n_neighboring_point_num] = point_cloud[k].label;


				if (point_cloud[i].label >= 0) point_cloud[k].label = point_cloud[i].label;



				if (n_neighboring_point_label_no_min > point_cloud[k].label) n_neighboring_point_label_no_min = point_cloud[k].label;



				n_neighboring_point_num++;


			}

		}


		if (n_neighboring_point_num == 0) {


			point_cloud[i].label = n_cluster_num;
			n_cluster_num++;


			continue;

		}
		else {

			point_cloud[i].label = n_neighboring_point_label_no_min;

			// update lookup table
			for (int j = 0; j < n_neighboring_point_num; j++) {

				if (lookup_table_point_label[neighboring_point_label_buffer[j]] > n_neighboring_point_label_no_min) {

					lookup_table_point_label[neighboring_point_label_buffer[j]] = n_neighboring_point_label_no_min;
				}

			}


		}





	}




	// re-numbering lookup table //
	int n_lookup_table_id;


	for (int i = n_cluster_num - 1; i >= 0; i--) {

		if (i > lookup_table_point_label[i]) {

			n_lookup_table_id = lookup_table_point_label[i];

			while (1) {

				if (n_lookup_table_id == lookup_table_point_label[n_lookup_table_id]) break;

				n_lookup_table_id = lookup_table_point_label[n_lookup_table_id];

			}

			lookup_table_point_label[i] = n_lookup_table_id;
		}

	}








	// re-labeling using lookup table //

	for (size_t i = 0; i < point_cloud.size(); i++) {

		point_cloud[i].label = lookup_table_point_label[point_cloud[i].label];

	}





	//n_cluster_num += 1;




	//calc cluster points histogram
	std::vector<int> cluster_points_histogram;

	cluster_points_histogram.resize(n_cluster_num);



	// calc histogram of point label //

	for (int i = 0; i < n_cluster_num; i++) {

		cluster_points_histogram[i] = 0;

	}



	for (size_t i = 0; i < point_cloud.size(); i++) {
		cluster_points_histogram[point_cloud[i].label]++;
	}





	// print lookup table //

	std::cout << "Histogram" << std::endl;

	for (int i = 0; i < n_cluster_num; i++) {


		std::cout << i << "\t" << lookup_table_point_label[i] << std::endl;


	}







	int n_new_label_num;


	n_new_label_num = 0;

	for (int i = 0; i < n_cluster_num; i++) {


		lookup_table_point_label[i] = i;

		if (cluster_points_histogram[i] > 0) {

			lookup_table_point_label[i] = n_new_label_num;
			n_new_label_num++;
		}

	}


	// re-labeling again //

	for (size_t i = 0; i < point_cloud.size(); i++) {

		point_cloud[i].label = lookup_table_point_label[point_cloud[i].label];

	}









	*cluster_class_num = n_new_label_num;






}



void eigen_estimate_point_cloud_plain_using_ransac(std::vector<struct point_cloud> &points, double d_ransac_distance_thresh_between_point_and_base_plane, std::vector<struct point_cloud> &points_candidates, Eigen::Vector3d& output_eigen_origin, Eigen::Vector3d& output_eigen_values, Eigen::Matrix3d& output_eigen_vectors)
{

	// RANSAC //


	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<int> random_no_generate(0, (int)(points.size() - 1));
	int n_random_point_no;

	int n_ransac_loop_times = 10000;


	struct point_cloud ransac_sampled_3points[3];
	//struct point_cloud ransac_base_plane_orgin;
	struct point_cloud_vector_type ransac_base_plane_vec[3];
	struct point_cloud_vector_type ransac_point_vec_on_base_plane;
	struct point_cloud_vector_type ransac_base_plane_vec_candidate[3];
	struct point_cloud ransac_origin_point_candidate;
	double d_abs_base_plane_vec;
	double d_distance_between_point_and_base_plane;
	int n_ransac_points_num_on_base_plane;
	int n_ransac_max_points_num_on_base_plane;













	n_ransac_max_points_num_on_base_plane = -1;

	for (int k = 0; k < n_ransac_loop_times; k++) {



		for (int i = 0; i < 3; i++) {

			n_random_point_no = random_no_generate(mt);
			ransac_sampled_3points[i] = points[n_random_point_no];


		}

		//std::cout << std::endl;




		ransac_base_plane_vec[0].x = ransac_sampled_3points[1].x - ransac_sampled_3points[0].x;
		ransac_base_plane_vec[0].y = ransac_sampled_3points[1].y - ransac_sampled_3points[0].y;
		ransac_base_plane_vec[0].z = ransac_sampled_3points[1].z - ransac_sampled_3points[0].z;

		ransac_base_plane_vec[1].x = ransac_sampled_3points[2].x - ransac_sampled_3points[0].x;
		ransac_base_plane_vec[1].y = ransac_sampled_3points[2].y - ransac_sampled_3points[0].y;
		ransac_base_plane_vec[1].z = ransac_sampled_3points[2].z - ransac_sampled_3points[0].z;


		// calc cross product

		ransac_base_plane_vec[2].x = ransac_base_plane_vec[0].y*ransac_base_plane_vec[1].z - ransac_base_plane_vec[0].z*ransac_base_plane_vec[1].y;
		ransac_base_plane_vec[2].y = ransac_base_plane_vec[0].z*ransac_base_plane_vec[1].x - ransac_base_plane_vec[0].x*ransac_base_plane_vec[1].z;
		ransac_base_plane_vec[2].z = ransac_base_plane_vec[0].x*ransac_base_plane_vec[1].y - ransac_base_plane_vec[0].y*ransac_base_plane_vec[1].x;

		d_abs_base_plane_vec = ransac_base_plane_vec[2].x*ransac_base_plane_vec[2].x + ransac_base_plane_vec[2].y*ransac_base_plane_vec[2].y + ransac_base_plane_vec[2].z*ransac_base_plane_vec[2].z;
		d_abs_base_plane_vec = sqrt(d_abs_base_plane_vec);

		ransac_base_plane_vec[2].x /= d_abs_base_plane_vec;
		ransac_base_plane_vec[2].y /= d_abs_base_plane_vec;
		ransac_base_plane_vec[2].z /= d_abs_base_plane_vec;


		//std::cout << "ransac:" << ransac_base_plane_vec[2].x << " " << ransac_base_plane_vec[2].y << " " << ransac_base_plane_vec[2].z << std::endl;



		n_ransac_points_num_on_base_plane = 0;

		for (int i = 0; i < points.size(); i++) {


			ransac_point_vec_on_base_plane.x = points[i].x - ransac_sampled_3points[0].x;
			ransac_point_vec_on_base_plane.y = points[i].y - ransac_sampled_3points[0].y;
			ransac_point_vec_on_base_plane.z = points[i].z - ransac_sampled_3points[0].z;


			d_distance_between_point_and_base_plane = ransac_point_vec_on_base_plane.x * ransac_base_plane_vec[2].x + ransac_point_vec_on_base_plane.y * ransac_base_plane_vec[2].y + ransac_point_vec_on_base_plane.z * ransac_base_plane_vec[2].z;

			d_distance_between_point_and_base_plane = fabs(d_distance_between_point_and_base_plane);

			if (d_distance_between_point_and_base_plane < d_ransac_distance_thresh_between_point_and_base_plane) {

				n_ransac_points_num_on_base_plane++;

			}



		}



		if (n_ransac_max_points_num_on_base_plane < n_ransac_points_num_on_base_plane) {

			n_ransac_max_points_num_on_base_plane = n_ransac_points_num_on_base_plane;

			ransac_origin_point_candidate = ransac_sampled_3points[0];


			ransac_base_plane_vec_candidate[2] = ransac_base_plane_vec[2];
			ransac_base_plane_vec_candidate[0] = ransac_base_plane_vec[0];
			ransac_base_plane_vec_candidate[1] = ransac_base_plane_vec[1];



			std::cout << "ransac max = " << n_ransac_max_points_num_on_base_plane << std::endl;


		}

	}


	std::cout << "ransac max = " << n_ransac_max_points_num_on_base_plane << std::endl;






	// pick up point candidates

	//std::vector<struct point_cloud> chessboard_points_candidates;
	struct point_cloud buf_chessboard_point_candidate;


	for (int i = 0; i < points.size(); i++) {


		ransac_point_vec_on_base_plane.x = points[i].x - ransac_origin_point_candidate.x;
		ransac_point_vec_on_base_plane.y = points[i].y - ransac_origin_point_candidate.y;
		ransac_point_vec_on_base_plane.z = points[i].z - ransac_origin_point_candidate.z;


		d_distance_between_point_and_base_plane = ransac_point_vec_on_base_plane.x * ransac_base_plane_vec_candidate[2].x + ransac_point_vec_on_base_plane.y * ransac_base_plane_vec_candidate[2].y + ransac_point_vec_on_base_plane.z * ransac_base_plane_vec_candidate[2].z;

		d_distance_between_point_and_base_plane = fabs(d_distance_between_point_and_base_plane);



		if (d_distance_between_point_and_base_plane < d_ransac_distance_thresh_between_point_and_base_plane) {

			buf_chessboard_point_candidate.x = points[i].x;
			buf_chessboard_point_candidate.y = points[i].y;
			buf_chessboard_point_candidate.z = points[i].z;
			buf_chessboard_point_candidate.label = points[i].label;
			buf_chessboard_point_candidate.intensity = points[i].intensity;
			points_candidates.push_back(buf_chessboard_point_candidate);

		}


	}



	std::cout << "ransac point candidates num = " << points_candidates.size() << std::endl;









	// Estimate the base plane using KL expansion //

	int n_ransac_KL_points_num;

	n_ransac_KL_points_num = (int)points_candidates.size();

	Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_ransac_KL_points_num, 3);
	Eigen::MatrixXd mat_t = Eigen::MatrixXd::Zero(3, n_ransac_KL_points_num);
	Eigen::Matrix3d MtM;
	Eigen::Vector3d vec_avg;




	for (int j = 0; j < points_candidates.size(); j++) {

		mat(j, 0) = points_candidates[j].x;
		mat(j, 1) = points_candidates[j].y;
		mat(j, 2) = points_candidates[j].z;


	}

	vec_avg = mat.colwise().mean();


	std::cout << "origin: " << vec_avg << std::endl;
	std::cout << std::endl;






	mat.rowwise() -= vec_avg.transpose();

	//for (int j = 0; j < chessboard_points_candidates.size(); j++) {
	//	mat(j, 0) -= vec_avg(0);
	//	mat(j, 1) -= vec_avg(1);
	//	mat(j, 2) -= vec_avg(2);
	//}



	mat_t = mat.transpose();


	MtM = mat_t * mat;

	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++) {

			std::cout << MtM(j, i) << " ";
		}
		std::cout << std::endl;

	}



	// KL expansion //


	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ES(MtM);
	double min_eigen;
	Eigen::Vector3d eigen_min_vector;
	Eigen::Vector3d eigen_vector1;
	Eigen::Vector3d eigen_vector2;
	Eigen::Vector3d eigen_vector3;


	if (ES.info() != Eigen::Success) {

		std::cout << "Eigen error!" << std::endl;
	}

	std::cout << std::endl;
	std::cout << "Eigen values: \n" << ES.eigenvalues() << std::endl;
	std::cout << "Eigen vectors: \n" << ES.eigenvectors() << std::endl;


	min_eigen = ES.eigenvalues()(0);
	eigen_min_vector = ES.eigenvectors().col(0);

	eigen_vector1 = ES.eigenvectors().col(2);
	eigen_vector2 = ES.eigenvectors().col(1);
	eigen_vector3 = ES.eigenvectors().col(0);

	std::cout << "Min eigen value: " << min_eigen << "( " << eigen_min_vector << " )" << std::endl;



	for (int i = 0; i < 3; i++) {

		//output_eigen_origin(i) = vec_avg(i);


	}


	output_eigen_origin = vec_avg;
	output_eigen_values = ES.eigenvalues();
	output_eigen_vectors = ES.eigenvectors();





}


double calc_min(double a, double b)
{
	double min;

	if (a < b) min = a;
	else min = b;

	return min;

}
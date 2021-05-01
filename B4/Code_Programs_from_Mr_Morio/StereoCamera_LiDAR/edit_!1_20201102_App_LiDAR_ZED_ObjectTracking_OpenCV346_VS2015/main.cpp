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
#define UNIT_MILLIMETER_COEFFICIENT	1.0 // ZED:1.0 Web:1000.0
#define	STEREO_Z_DEPTH_MAX	30
#define	STEREO_Z_DEPTH_MIN	0



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





	FILE *fp_lidar_point_cloud;
	FILE *fp_zed_point_cloud;
	//FILE *fp_w;
	FILE *fp_w_hist;
	FILE *fp_w_lidar_point_cloud;
	FILE *fp_w_zed_point_cloud;
	FILE *fp_w_lidar_labeled_points;
	FILE *fp_w_lidar_to_zed_point;
	FILE *fp_w_zed_stereo_calculated_points;
	
	errno_t err;

	std::string sMainFolder;
	std::string sFilePath_parameters;
	std::string sFname_point_cloud;
	std::string sFname_zed_point_cloud;
	std::string sFname_zed_image;
	std::string sFilePath_LiDAR_point_cloud;
	std::string sFilePath_ZED_point_cloud;
	std::string sFilePath_StereoImage;
	//std::string sSaveFilePath;
	std::string sSaveFilePath_ZED_point_cloud;
	std::string sSaveFilePath_LiDAR_point_cloud;
	std::string sSaveFilePath_ZED_calculated_stereo_point_cloud;
	std::string sInputImageFilePath;
	std::string sSaveInteisityHistogramFilePath;
	std::string sSaveFilePath_LabeledPointCloud;
	








	
	const Size IM_SIZE = Size(IMG_XSIZE, IMG_YSIZE);


	
	// the names of display windows //

	string win_src_cam1 = "cam1";
	string win_src_cam2 = "cam2";




	


	//bool found_cam1;
	//bool found_cam2;


	Mat xyz_ZED(IMG_YSIZE, IMG_XSIZE, CV_32FC3);
	Vec3f p_xyz_ZED;

	
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
	



	// [ 2D ] ZED image //
	
	vector<Point2f> imagePoints_ZED;
	vector<Point2f> imagePoints_LiDAR; // change the order of the corner Nos.
	Point2f buf_image_poin_ZED;
	Point2f buf_image_point_LiDAR;

	vector<Point2f> imagePoints_LiDAR2ZED;

	

	// [ 3D ] LiDAR //

	std::vector<struct point_cloud> point_cloud;
	struct point_cloud point;

	std::vector<Point3f> point_cloud_LiDAR_yzx;
	cv::Point3f buf_LiDAR_yzx;

	//distance to object
	std::vector<double>all_lidar_distance;

	std::vector<Point3f> point_cloud_LiDAR2ZED;
	cv::Point3f buf_LiDAR2ZED_xyz;
	std::vector<Point3f> point_cloud_LiDAR2ZED_ROS;
	cv::Point3f buf_LiDAR2ZED_xyz_ROS;

		

	// [ 3D ] ZED STEREO //
	vector<Point3f> point_cloud_ZED;
	Point3f buf_point_cloud_ZED;
	double zed_x, zed_y, zed_z;
	int zed_r, zed_g, zed_b;

	Mat mat_point_cloud_ZED(IMG_YSIZE, IMG_XSIZE, CV_32FC3);
	cv::Vec3f p_point_zed;


	Mat K1, K2;
	Mat D1, D2;
	vector<Mat> rvecs1, rvecs2;
	vector<Mat> tvecs1, tvecs2;

	Mat R, F, E;
	Vec3d T;

	Mat R1, R2, P1, P2;
	Mat Q;

	Mat map11, map12, map21, map22;




	// ZED stereo calculation 
	Mat xyz(IMG_YSIZE, IMG_XSIZE, CV_32FC3);
	Mat mask_with_xyz(IMG_YSIZE, IMG_XSIZE, CV_8UC1, cv::Scalar(0));





	// lidar points -> zed points

	cv::Mat mat_R_LiDAR2ZED = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::Mat mat_T_LiDAR2ZED = cv::Mat::zeros(3, 1, CV_64FC1);

	//set camera parameters //

	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);




	Eigen::Vector3d eigen_T_LiDAR2ZED; //[〇,△,×]
	Eigen::MatrixXd eigen_R_LiDAR2ZED(3, 3); //3x3の行列
	Eigen::MatrixXd lidar_point_IN(3, 1);
	Eigen::MatrixXd lidar_point_OUT(3, 1);





	bool flag_nan;


	double point_cloud_clustering_d_dist_thresh;



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


	std::cout << endl;
	std::cout << endl;




	initUndistortRectifyMap(K1, D1, R1, P1, IM_SIZE, CV_16SC2, map11, map12);
	initUndistortRectifyMap(K2, D2, R2, P2, IM_SIZE, CV_16SC2, map21, map22);
















	// LiDAR, ZED calibration //


	std::string sFilePath_PnP_rvec_tvec;

	std::cout << "[PnP PARAMETERS: rvec, tvec ] File path = ";
	std::cin >> sFilePath_PnP_rvec_tvec;




	cv::FileStorage fs_rvec_tvec(sFilePath_PnP_rvec_tvec, FileStorage::READ);


	fs_rvec_tvec["K1"] >> K1;
	fs_rvec_tvec["D1"] >> D1;
	fs_rvec_tvec["rvec"] >> rvec;
	fs_rvec_tvec["tvec"] >> tvec;

	fs_rvec_tvec["mat_T_LiDAR2ZED"] >> mat_T_LiDAR2ZED; //キャリブレーションで求めた並行行列
	fs_rvec_tvec["mat_R_LiDAR2ZED"] >> mat_R_LiDAR2ZED; //キャリブレーションで求めた回転行列


	fs_rvec_tvec.release();




	std::cout << endl;
	std::cout << "PnP parameters" << endl;
	std::cout << "K1:" << K1 << endl;
	std::cout << "D1:" << D1 << endl;
	std::cout << "rvec:" << rvec << endl;
	std::cout << "tvec:" << tvec << endl;
	std::cout << "mat_T_LiDAR2ZED:" << mat_T_LiDAR2ZED << endl;
	std::cout << "mat_R_LiDAR2ZED:" << mat_R_LiDAR2ZED << endl;
	std::cout << endl;
	std::cout << endl;

	


	// Set eigen matrix

	for (int j = 0; j < 3; j++) {

		eigen_T_LiDAR2ZED(j) = mat_T_LiDAR2ZED.at<double>(j, 0);

		for (int i = 0; i < 3; i++) {

			eigen_R_LiDAR2ZED(j, i) = mat_R_LiDAR2ZED.at<double>(j, i);

		}
	}























	/////////////////////////////////////////////
	//                 LIDAR                   //
	/////////////////////////////////////////////





	std::cout << endl;

	std::cout << "[WORKING FOLDER] main folder = ";
	std::cin >> sMainFolder;

	std::cout << "[PARAMETERS] Parameter file path = ";
	std::cin >> sFilePath_parameters;

	FileStorage fs_parameters(sFilePath_parameters, FileStorage::READ);


	fs_parameters["sFname_point_cloud"] >> sFname_point_cloud;			cout << "LiDAR POINT FNAME:" << sFname_point_cloud << endl;
	fs_parameters["sFname_zed_point_cloud"] >> sFname_zed_point_cloud;	cout << "ZED POINT FNAME:" << sFname_zed_point_cloud << endl;
	fs_parameters["sFname_zed_image"] >> sFname_zed_image;				cout << "ZED IMAGE:" << sFname_zed_image << endl;
	fs_parameters["point_cloud_clustering_d_dist_thresh"] >> point_cloud_clustering_d_dist_thresh; cout << "CLUSTERING POINTS. DIST THRESH:" << sFname_zed_image << endl;


	//std::cout << "Filepath for LiDAR point cloud data = ";
	//std::cin >> sFilePath_LiDAR_point_cloud;


	//////////////////////////////////


	int n_From;
	int n_To;

	std::cout << "[DATA NO.] From = ";		std::cin >> n_From;
	std::cout << "[DATA NO.] To = ";		std::cin >> n_To;








	for (int i = n_From; i <= n_To; i++) 
	{



		sFilePath_LiDAR_point_cloud = sMainFolder + "/" + sFname_point_cloud + to_string(i) + ".txt";

		std::cout << "[LiDAR POINTS]" << endl;
		std::cout << sFilePath_LiDAR_point_cloud << endl;





		err = fopen_s(&fp_lidar_point_cloud, sFilePath_LiDAR_point_cloud.c_str(), "rt");

		if (err != 0) {

			std::cout << "Cannot open " << sFilePath_LiDAR_point_cloud << std::endl;

			exit(1);
		}



		sSaveFilePath_LiDAR_point_cloud = sFilePath_LiDAR_point_cloud + "_NaN_data_removed.txt";

		err = fopen_s(&fp_w_lidar_point_cloud, sSaveFilePath_LiDAR_point_cloud.c_str(), "wt");









		while (fscanf_s(fp_lidar_point_cloud, "%lf\t%lf\t%lf\t%lf", &point.x, &point.y, &point.z, &point.intensity) != EOF) 
		{





			flag_nan = isfinite(point.x); //isfinite・・・引数の値が有限の値かどうかを判定します．

			if (flag_nan == true) {



				point_cloud.push_back(point);



				// change the coordinates of xyz -> yzx (LiDAR)


				buf_LiDAR_yzx.x = (float)(-point.y);
				buf_LiDAR_yzx.y = (float)(-point.z);
				buf_LiDAR_yzx.z = (float)point.x;

				//distance to object
				double lidar_distance = sqrt(pow(buf_LiDAR_yzx.x, 2.0) + pow(buf_LiDAR_yzx.y, 2.0) + pow(buf_LiDAR_yzx.z, 2.0));

				all_lidar_distance.push_back(lidar_distance);


				point_cloud_LiDAR_yzx.push_back(buf_LiDAR_yzx);


				fprintf(fp_w_lidar_point_cloud, "%lf\t%lf\t%lf\t%lf\n", point.x, point.y, point.z, point.intensity);


			}








		}


		fclose(fp_lidar_point_cloud);

		fclose(fp_w_lidar_point_cloud);


		std::cout << endl;
		std::cout << "Point cloud data size : " << to_string(point_cloud.size()) << endl;
		std::cout << endl;










		// calc intensity histogram //


		int n_intensity_histogram_level_num = 256;//0-255
		std::vector<int> histogram_Inteinsity;


		histogram_Inteinsity.resize(n_intensity_histogram_level_num);



		for (int i = 0; i < n_intensity_histogram_level_num; i++) {

			histogram_Inteinsity[i] = 0;
		}


		for (int i = 0; i < point_cloud.size(); i++) {


			histogram_Inteinsity[(int)(point_cloud[i].intensity)]++;


		}




		// save intensity histogram



		sSaveInteisityHistogramFilePath = sFilePath_LiDAR_point_cloud + "_inteisity_histogram.txt";

		fopen_s(&fp_w_hist, sSaveInteisityHistogramFilePath.c_str(), "wt");


		for (int i = 0; i < n_intensity_histogram_level_num; i++) {

			fprintf_s(fp_w_hist, "%d\t%d\n", i, histogram_Inteinsity[i]);
		}


		fclose(fp_w_hist);






























/*


		////////////// ZED Point Cloud ////////////////





		std::cout << endl;
		//std::cout << "[ZED POINTS] File path = ";
		//std::cin >> sFilePath_ZED_point_cloud;


		sFilePath_ZED_point_cloud = sMainFolder + "/" + sFname_zed_point_cloud + to_string(i) + ".txt";

		std::cout << "[ZED POINTS]" << endl;
		std::cout << sFilePath_ZED_point_cloud << endl;




		err = fopen_s(&fp_zed_point_cloud, sFilePath_ZED_point_cloud.c_str(), "rt");

		if (err != 0) {

			std::cout << "cannot open the zed point file" << endl;
			exit(1);
		}


		sSaveFilePath_ZED_point_cloud = sFilePath_ZED_point_cloud + "_NaN_data_removed.txt";

		err = fopen_s(&fp_w_zed_point_cloud, sSaveFilePath_ZED_point_cloud.c_str(), "wt");



		int n_zed_point_count;
		int zed_image_x, zed_image_y;



		img_cam1_dst = Mat::zeros(IMG_YSIZE, IMG_XSIZE, CV_8UC3);
		mat_point_cloud_ZED = cv::Mat::zeros(IMG_YSIZE, IMG_XSIZE, CV_32FC3);



		n_zed_point_count = 0;





		while (fscanf_s(fp_zed_point_cloud, "%lf\t%lf\t%lf\t%d\t%d\t%d", &zed_x, &zed_y, &zed_z, &zed_r, &zed_g, &zed_b) != EOF) {


			zed_image_x = n_zed_point_count % IMG_XSIZE;
			zed_image_y = int(n_zed_point_count / IMG_XSIZE);

			if (zed_image_x >= IMG_XSIZE || zed_image_y >= IMG_YSIZE) {

				std::cout << "error: zed image range over" << endl;
				exit(1);

			}

			p_point_zed[0] = (float)zed_x;
			p_point_zed[1] = (float)zed_y;
			p_point_zed[2] = (float)zed_z;

			mat_point_cloud_ZED.at<Vec3f>(zed_image_y, zed_image_x) = p_point_zed;




			flag_nan = isfinite(zed_x);

			if (flag_nan == true) {

				fprintf(fp_w_zed_point_cloud, "%lf\t%lf\t%lf\t%d\t%d\t%d\n", zed_x, zed_y, zed_z, zed_r, zed_g, zed_b);

				//img_cam1_dst.at<Vec3b>(zed_image_y, zed_image_x)(0) = zed_b;
				//img_cam1_dst.at<Vec3b>(zed_image_y, zed_image_x)(1) = zed_g;
				//img_cam1_dst.at<Vec3b>(zed_image_y, zed_image_x)(2) = zed_r;



			}


			n_zed_point_count++;

		}





		fclose(fp_zed_point_cloud);
		fclose(fp_w_zed_point_cloud);




*/















		////////// ZED image ////////




		std::cout << endl;
		//std::cout << "[STEREO CAMERA IMAGE] file path = ";
		//std::cin >> sFilePath_StereoImage;


		
		sFilePath_StereoImage = sMainFolder + "/" + sFname_zed_image + to_string(i) + ".png";

		std::cout << "[STEREO CAMERA IMAGE]" << endl;
		std::cout << sFilePath_StereoImage << endl;





		cv::namedWindow(win_src_cam1, CV_WINDOW_AUTOSIZE);
		cv::namedWindow(win_src_cam2, CV_WINDOW_AUTOSIZE);




		img_stereo = imread(sFilePath_StereoImage, 1);


		img_cam1 = img_stereo(cv::Rect(0, 0, img_stereo.cols / 2, img_stereo.rows));
		img_cam2 = img_stereo(cv::Rect(img_stereo.cols / 2, 0, img_stereo.cols / 2, img_stereo.rows));










		//cv::imshow(win_src_cam1, img_cam1);
		//cv::imshow(win_src_cam2, img_cam2);








		remap(img_cam1, img_cam1_remap, map11, map12, INTER_LINEAR);
		remap(img_cam2, img_cam2_remap, map21, map22, INTER_LINEAR);






		cv::imshow(win_src_cam1, img_cam1_remap);
		cv::imshow(win_src_cam2, img_cam2_remap);


		//std::cout << "Hit any key to continue... " << endl;
		cv::waitKey(1);









		// StereoSGBM with Weighted Least Squares filter //

		int sgbm_minDisparity = 0;
		int sgbm_numDisparities = 16 * 4;
		int sgbm_window_size = 3;
		int sgbm_P1 = 8 * 3 * sgbm_window_size*sgbm_window_size;
		int sgbm_P2 = 32 * 3 * sgbm_window_size*sgbm_window_size;
		int sgbm_disp12MaxDiff = 16 * 4;
		int sgbm_preFilterCap = 63;
		int sgbm_uniquenessRatio = 15;
		int sgbm_speckleWindowSize = 200;
		int sgbm_speckleRange = 1;
		double wls_lmbda = 200;// 80000.0;
		double wls_sigma = 2.0;// 1.5;




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







		// Calc the disparity filtered by using weighted Least Squares filter //

		// gray image

		cvtColor(img_cam1_remap, img_cam1_gray_remap, CV_BGR2GRAY);
		cvtColor(img_cam2_remap, img_cam2_gray_remap, CV_BGR2GRAY);



		left_for_matcher = img_cam1_gray_remap.clone();
		right_for_matcher = img_cam2_gray_remap.clone();




		left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
		right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);


		wls_filter->setLambda(wls_lmbda);
		wls_filter->setSigmaColor(wls_sigma);
		wls_filter->filter(left_disp, img_cam1_gray_remap, filtered_disp, right_disp); // output: filtered_disp






																					   //creating a disparity map visualization (clamped CV_8U image) form CV_16S depth

		getDisparityVis(left_disp, raw_disp_vis, vis_mult);


		getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);



		// gray to BGR //
		//cvtColor(filtered_disp_vis, img_merge, CV_GRAY2BGR);




		// calc 3D corrdinates //


		reprojectImageTo3D(filtered_disp, xyz, Q, true);







		for (int y = 0; y < xyz.rows; y++) {
			for (int x = 0; x < xyz.cols; x++) {




				xyz.at<Vec3f>(y, x) *= 16.0 / UNIT_MILLIMETER_COEFFICIENT;




				Vec3f point_xyz = xyz.at<Vec3f>(y, x);







				if (fabs(point_xyz[2] - STEREO_Z_DEPTH_MAX) < FLT_EPSILON || fabs(point_xyz[2]) > STEREO_Z_DEPTH_MAX || fabs(point_xyz[2]) < STEREO_Z_DEPTH_MIN) {


					mask_with_xyz.at<uchar>(y, x) = 0;


				}
				else {



					mask_with_xyz.at<uchar>(y, x) = 1;



				}


			}
		}









		sSaveFilePath_ZED_calculated_stereo_point_cloud = sFilePath_StereoImage + "_ZED_calculated_stereo_point_cloud.txt";

		fopen_s(&fp_w_zed_stereo_calculated_points, sSaveFilePath_ZED_calculated_stereo_point_cloud.c_str(), "wt");




		for (int y = 0; y < xyz.rows; y++) {
			for (int x = 0; x < xyz.cols; x++) {



				Vec3f point_xyz = xyz.at<Vec3f>(y, x);
				Vec3b point_rgb = img_cam1_remap.at<Vec3b>(y, x);


				if (mask_with_xyz.at<uchar>(y, x) == 1) {

					fprintf_s(fp_w_zed_stereo_calculated_points, "%f\t%f\t%f\t%d\t%d\t%d\n", point_xyz[0], point_xyz[1], point_xyz[2], point_rgb[2], point_rgb[1], point_rgb[0]);
				}



			}
		}




		fclose(fp_w_zed_stereo_calculated_points);






		//////////// CLUSTERING POINT CLOUD ///////////////





		
		int n_clustering_class_num;





		std::cout << endl;
		std::cout << "[CLUSTERING] Thresholding value for distance in labeling points" << endl;
		std::cout << "Distance thresh :" << point_cloud_clustering_d_dist_thresh << endl;









		clustering_point_cloud(point_cloud, point_cloud_clustering_d_dist_thresh, &n_clustering_class_num);


		std::cout << "clustering done!" << endl;









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


		sSavePointLabelHistogramFilePath = sFilePath_LiDAR_point_cloud + "_clustering_point_label_histogram_DistTh" + std::to_string(point_cloud_clustering_d_dist_thresh) + ".txt";

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



		sSaveFilePath_LabeledPointCloud = sFilePath_LiDAR_point_cloud + "_clustering_points_and_label_use_distTh" + std::to_string(point_cloud_clustering_d_dist_thresh) + ".txt";

		fopen_s(&fp_w_lidar_labeled_points, sSaveFilePath_LabeledPointCloud.c_str(), "wt");





		for (size_t i = 0; i < point_cloud.size(); i++) {


			fprintf(fp_w_lidar_labeled_points, "%lf\t%lf\t%lf\t%d\n", point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, point_cloud[i].label);


		}




		fclose(fp_w_lidar_labeled_points);


















		// LiDAR xyz coordinates -> ZED xyz //







		// in: point_cloud_LiDAR_yzx
		//out: point_cloud_LiDAR2ZED

		for (size_t i = 0; i < point_cloud_LiDAR_yzx.size(); i++) {


			lidar_point_IN(0, 0) = point_cloud_LiDAR_yzx[i].x + eigen_T_LiDAR2ZED(0);
			lidar_point_IN(1, 0) = point_cloud_LiDAR_yzx[i].y + eigen_T_LiDAR2ZED(1);
			lidar_point_IN(2, 0) = point_cloud_LiDAR_yzx[i].z + eigen_T_LiDAR2ZED(2);

			lidar_point_OUT = eigen_R_LiDAR2ZED*lidar_point_IN;

			buf_LiDAR2ZED_xyz.x = (float)lidar_point_OUT(0, 0);
			buf_LiDAR2ZED_xyz.y = (float)lidar_point_OUT(1, 0);
			buf_LiDAR2ZED_xyz.z = (float)lidar_point_OUT(2, 0);

			point_cloud_LiDAR2ZED.push_back(buf_LiDAR2ZED_xyz); //LiDARの点群座標をStereoに合わせた結果をプッシュバック

			buf_LiDAR2ZED_xyz_ROS.x = buf_LiDAR2ZED_xyz.z;
			buf_LiDAR2ZED_xyz_ROS.y = -buf_LiDAR2ZED_xyz.x;
			buf_LiDAR2ZED_xyz_ROS.z = -buf_LiDAR2ZED_xyz.y;

			point_cloud_LiDAR2ZED_ROS.push_back(buf_LiDAR2ZED_xyz_ROS);


		}



		string sSaveFilePath_point_cloud_LiDAR_to_ZED;


		sSaveFilePath_point_cloud_LiDAR_to_ZED = sFilePath_LiDAR_point_cloud + "_LiDAR_to_ZED_projected_points_xyz.txt";

		err = fopen_s(&fp_w_lidar_to_zed_point, sSaveFilePath_point_cloud_LiDAR_to_ZED.c_str(), "wt");


		for (size_t i = 0; i < point_cloud_LiDAR2ZED_ROS.size(); i++) {

			fprintf(fp_w_lidar_to_zed_point, "%f\t%f\t%f\n", point_cloud_LiDAR2ZED[i].x, point_cloud_LiDAR2ZED[i].y, point_cloud_LiDAR2ZED[i].z);


		}

		fclose(fp_w_lidar_to_zed_point);









		// Projection of LiDAR points onto image //

		img_cam1_dst = img_cam1_remap.clone();


		cv::projectPoints(point_cloud_LiDAR_yzx, rvec, tvec, K1, D1, imagePoints_LiDAR2ZED);
		

		FILE *fp_w_estimated_lidar_points_on_image;
		std::string sSaveFilePath_estimated_lidar_points_on_image;

		sSaveFilePath_estimated_lidar_points_on_image = sFilePath_LiDAR_point_cloud + "_lidar_points_on_image.txt";

		fopen_s(&fp_w_estimated_lidar_points_on_image, sSaveFilePath_estimated_lidar_points_on_image.c_str(), "wt");






		for (int j = 0; j < imagePoints_LiDAR2ZED.size(); j++) {


			if (point_cloud_LiDAR_yzx[j].z < 0) continue; //


			if (imagePoints_LiDAR2ZED[j].x < 0 || imagePoints_LiDAR2ZED[j].x >= IMG_XSIZE || imagePoints_LiDAR2ZED[j].y < 0 || imagePoints_LiDAR2ZED[j].y >= IMG_YSIZE) continue;


			fprintf_s(fp_w_estimated_lidar_points_on_image, "%f\t%f\t%f\n", imagePoints_LiDAR2ZED[j].x, imagePoints_LiDAR2ZED[j].y, all_lidar_distance[j]);


			cv::circle(img_cam1_dst, cv::Point((int)imagePoints_LiDAR2ZED[j].x, (int)imagePoints_LiDAR2ZED[j].y), 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);


		}






		fclose(fp_w_estimated_lidar_points_on_image);






		// save output image //

		std::string sSaveFilePath_image;
		sSaveFilePath_image = sFilePath_StereoImage + "_PnP_projected_points.png";


		imwrite(sSaveFilePath_image, img_cam1_dst);








		//memory clear

		point_cloud.clear(); point_cloud.shrink_to_fit();
		point_cloud_LiDAR_yzx.clear(); point_cloud_LiDAR_yzx.shrink_to_fit();
		all_lidar_distance.clear(); all_lidar_distance.shrink_to_fit();
		point_cloud_LiDAR2ZED.clear(); point_cloud_LiDAR2ZED.shrink_to_fit();
		point_cloud_LiDAR2ZED_ROS.clear(); point_cloud_LiDAR2ZED_ROS.shrink_to_fit();
		point_cloud_ZED.clear(); point_cloud_ZED.shrink_to_fit();

		imagePoints_ZED.clear(); imagePoints_ZED.shrink_to_fit();
		imagePoints_LiDAR.clear(); imagePoints_LiDAR.shrink_to_fit();
		imagePoints_LiDAR2ZED.clear(); imagePoints_LiDAR2ZED.shrink_to_fit();





	}









/*









	

	//lidar_point_IN(0, 0) = objectCorners_yzx[j].x + eigen_T_LiDAR2ZED(0);
	//lidar_point_IN(1, 0) = objectCorners_yzx[j].y + eigen_T_LiDAR2ZED(1);
	//lidar_point_IN(2, 0) = objectCorners_yzx[j].z + eigen_T_LiDAR2ZED(2);
	//
	//lidar_point_OUT = eigen_R_LiDAR2ZED * lidar_point_IN;

	


	





	img_cam1_dst = img_cam1_remap.clone();


	

	for (int j = 0; j < point_cloud.size(); j++) {


		buf_LiDAR2ZED_yzx.x = (float)(-point_cloud[j].y);
		buf_LiDAR2ZED_yzx.y = (float)(-point_cloud[j].z);
		buf_LiDAR2ZED_yzx.z = (float)(point_cloud[j].x);

		lidar_point_IN(0, 0) = buf_LiDAR2ZED_yzx.x + eigen_T_LiDAR2ZED(0);
		lidar_point_IN(1, 0) = buf_LiDAR2ZED_yzx.y + eigen_T_LiDAR2ZED(1);
		lidar_point_IN(2, 0) = buf_LiDAR2ZED_yzx.z + eigen_T_LiDAR2ZED(2);


		

		lidar_point_OUT = eigen_R_LiDAR2ZED * lidar_point_IN;


		buf_LiDAR2ZED_yzx.x = (float)lidar_point_OUT(0, 0);
		buf_LiDAR2ZED_yzx.y = (float)lidar_point_OUT(1, 0);
		buf_LiDAR2ZED_yzx.z = (float)lidar_point_OUT(2, 0);

	


		point_cloud_LiDAR2ZED.push_back(buf_LiDAR2ZED_yzx);


	
		

	}

	






	// solvePnP //

	
	//point_cloud -> image corrdinates ; directly //

	//cv::solvePnP(objectCorners_yzx, imageCorners1_LiDAR, K1, D1, rvec, tvec);
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


















		
	
	
	
	
	std::cout << "Hit any key to continue ... " << endl;


	cv::imshow(win_src_cam1, img_cam1_dst);
	cv::waitKey(0);





	

*/


	




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
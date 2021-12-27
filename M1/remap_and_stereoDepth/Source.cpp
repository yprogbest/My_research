#include <opencv2/opencv.hpp>
#include <opencv2\ximgproc.hpp>
#include <iostream>
#include <string>

#define IMG_XSIZE	1280
#define IMG_YSIZE	720

using namespace std;
using namespace cv;

int main(void) {
	const Size IM_SIZE = Size(IMG_XSIZE, IMG_YSIZE);

	string yaml_pass;
	cout << "yaml pass=";
	cin >> yaml_pass;

	FileStorage fs_stereo_param(yaml_pass, FileStorage::READ);

	cv::Mat K1, K2, D1, D2, R, R1, R2, P1, P2, Q;
	cv::Mat map11, map12, map21, map22;


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

	initUndistortRectifyMap(K1, D1, R1, P1, IM_SIZE, CV_16SC2, map11, map12);
	initUndistortRectifyMap(K2, D2, R2, P2, IM_SIZE, CV_16SC2, map21, map22);

	string stereo_pass;
	cv::Mat stereo_img;
	cv::Mat left_remap, right_remap;

	string p = ".";


	int mode;
	cout << "mode\n1: remap\n2: stereo vision" << endl;
	cin >> mode;

	switch (mode) {
	case 1:
		while (true) {
			cout << "stereo img pass=";
			cin >> stereo_pass;
			stereo_img = imread(stereo_pass, 1);

			cv::Mat left_image = stereo_img(cv::Rect(0, 0, stereo_img.cols / 2.0, stereo_img.rows));
			cv::Mat right_image = stereo_img(cv::Rect(stereo_img.cols / 2.0, 0, stereo_img.cols / 2.0, stereo_img.rows));

			remap(left_image, left_remap, map11, map12, INTER_LINEAR);
			remap(right_image, right_remap, map21, map22, INTER_LINEAR);

			size_t p_num = stereo_pass.find(p);
			string pass_name = stereo_pass.substr(0, p_num);

			string left_remap_pass = pass_name + "_left_remap.png";
			string right_remap_pass = pass_name + "_right_remap_png";

			imwrite(left_remap_pass, left_remap);
			imwrite(right_remap_pass, right_remap);
		}
		break;

	case 2:
		string disp_pass;
		cout << "disp pass =";
		cin >> disp_pass;
		ifstream ifs(disp_pass);
		/*
		int disp_width, disp_height;
		cout << "width=";
		cin >> disp_width;
		*/

		cout << "stereo img pass=";
		cin >> stereo_pass;

		stereo_img = imread(stereo_pass, 1);

		cv::Mat left_image = stereo_img(cv::Rect(0, 0, stereo_img.cols / 2.0, stereo_img.rows));
		cv::Mat right_image = stereo_img(cv::Rect(stereo_img.cols / 2.0, 0, stereo_img.cols / 2.0, stereo_img.rows));
		cv::Mat disp_image(left_image.rows, left_image.cols, CV_32FC1);
		Mat xyz(IMG_YSIZE, IMG_XSIZE, CV_64FC1);

		for (size_t i = 0; i < left_image.rows; i++) {
			for (size_t j = 0; j < left_image.cols; j++) {
				double temp_disp;
				ifs >> temp_disp;
				disp_image.at<float>(i, j) = temp_disp;
			}
		}


		reprojectImageTo3D(disp_image, xyz, Q, true);

		//	size_t p_num = disp_pass.find(p);
		//	string pass_name = disp_pass.substr(0, p_num);

		string depth_pass = "D:\\M1\\PSMNet\\result.txt";

		cout << depth_pass << endl;

		ofstream ofs(depth_pass);

		cout << "output" << endl;

		for (size_t i = 0; i < left_image.rows; i++) {
			for (size_t j = 0; j < left_image.cols; j++) {
				xyz.at<Vec3f>(i, j) *= 16.0;
				int B = left_image.at<Vec3b>(i, j)[0];
				int G = left_image.at<Vec3b>(i, j)[1];
				int R_color = left_image.at<Vec3b>(i, j)[2];
				ofs << xyz.at<Vec3f>(i, j)[0] << "\t" << xyz.at<Vec3f>(i, j)[1] << "\t" << xyz.at<Vec3f>(i, j)[2] << "\t" << R_color << "\t" << G << "\t" << B << endl;
			}
		}
		break;

	}

	return 0;
}

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //テキストファイルを扱うために用意
#include <string>
#include <process.h>
#include <time.h>
#include <random>

#include <opencv2/opencv.hpp>//OpenCVのインクルード
#include "opencv2/highgui/highgui.hpp"


#define IMG_XSIZE 672
#define IMG_YSIZE 376


//テキストファイル内の各行の可能最大文字数として定義
#define YOLO_FILE_LINE_MAX_BUFFER_SIZE	1024
#define YOLO_LABEL_STRING_MAX_LENGTH  100

struct yolo_bounding_box
{
	int x;
	int y;
	int width;
	int height;
	char name[YOLO_LABEL_STRING_MAX_LENGTH];
	double likelihood;

};



//LiDARのx,y,zと反射強度のテキストファイル内を読み込むために用意
struct point_cloud
{
	double x;
	double y;
	double z;
	double intensity;
};


struct lidar_int {
	int x;
	int y;
};




//using宣言
using namespace cv;
using namespace std;


//プロトタイプ宣言
void menu_screen();
int yolo_pixcel_count();
int seg_pixcel_count();
int seg_pixcel_count();
int yolo_box_coordinate();
int object_tracking();




int nCommand;
int i_yolo_lidar = 0;



void main(int argc, const char* argv[])
{


	int n = 0; //画像から動画を作成する際のフレーム数

	int nFlag;
	nFlag = 1;

	while (nFlag)
	{

		menu_screen();

		switch (nCommand)
		{
	
			case 1:

				yolo_pixcel_count();
				break;


			case 2:
				seg_pixcel_count();
				break;


			case 3:
				yolo_box_coordinate();
				break;


			case 4:
				object_tracking();

				break;



			case 0:
				nFlag = -1;
				exit(1);

		}
	}

}



//メニュー画面の関数
void menu_screen()
{

	printf("\n");
	printf("----------------------------------------------------\n");
	printf("<<1>>:YOLOの背景の画素の割合\n");
	printf("<<2>>:segmentationの背景の画素の割合\n");
	printf("<<3>>:YOLOの認識結果のファイルを1フレームごと保存\n");
	printf("<<4>>:【メイン】物体追跡\n");
	printf("<<5>>:○○\n");
	printf("<<0>>:終了します．\n");
	printf("----------------------------------------------------\n");
	printf("\n");

	printf("command=");
	scanf_s("%d", &nCommand);


}








int yolo_pixcel_count() {

	//ピクセルのカウント数
	int count = 0;

	//Bounding boxの幅と高さ
	int box_height = 203;
	int box_width = 79;

	//box内のピクセル数
	int box_pixcel_count = box_width * box_height;


	//背景の割合
	double back_ground_rate;



	string folder_path = "D:\\M1\\";
	string image_name = folder_path + "image385.png";

	Mat mask_img = cv::imread(image_name);

	int img_h = mask_img.rows;
	int img_w = mask_img.cols;


	for (int j = 0; j < img_h; j++) {
		for (int i = 0; i < img_w; i++) {
			
			if (mask_img.at<cv::Vec3b>(j, i)[0] >= 127 && mask_img.at<cv::Vec3b>(j, i)[1] >= 127 && mask_img.at<cv::Vec3b>(j, i)[2] >= 127) {

				count++;
			}
		}
	}


	printf("full_pixcel = %d\n", box_pixcel_count);

	printf("back_ground = %d\n", box_pixcel_count- count);

	back_ground_rate = (((double)box_pixcel_count - (double)count) / (double)box_pixcel_count);


	printf("back_ground_rate = %lf", back_ground_rate);


	cv::namedWindow("result", WINDOW_AUTOSIZE);
	cv::imshow("result", mask_img);

	waitKey();

	return 0;

}





int seg_pixcel_count() {

	int mask_back_count = 0;
	int seg_back_count = 0;

	string folder_path = "D:\\M1\\";
	string image_name = folder_path + "image385.png";
	Mat mask_img = cv::imread(image_name);

	string folder_path_seg = "D:\\M1\\Mask_RCNN\\result_image\\20210609143611_mask\\";
	string image_name_seg = folder_path_seg + "image385.png";
	Mat seg_img = cv::imread(image_name_seg);



	int img_h = mask_img.rows;
	int img_w = mask_img.cols;

	//背景の割合
	double back_ground;




	/*for (int j = 0; j < img_h; j++) {
		for (int i = 0; i < img_w; i++) {

			if (mask_img.at<cv::Vec3b>(j, i)[0] <= 127 && mask_img.at<cv::Vec3b>(j, i)[1] <= 127 && mask_img.at<cv::Vec3b>(j, i)[2] <= 127) {

				mask_back_count++;
			}
		}
	}*/


	for (int j = 0; j < img_h; j++) {
		for (int i = 0; i < img_w; i++) {

			if ((mask_img.at<cv::Vec3b>(j, i)[0] > 127 && mask_img.at<cv::Vec3b>(j, i)[1] > 127 && mask_img.at<cv::Vec3b>(j, i)[2] > 127)|| 
				(seg_img.at<cv::Vec3b>(j, i)[0] > 127 && seg_img.at<cv::Vec3b>(j, i)[1] > 127 && seg_img.at<cv::Vec3b>(j, i)[2] > 127)) {

				mask_back_count++;
			}
		}
	}



	for (int j = 0; j < img_h; j++) {
		for (int i = 0; i < img_w; i++) {

			//もし，マスク画像が黒（背景）でセグメンテーションの画像が白（物体上）なら，背景なのでカウントする
			if ((mask_img.at<cv::Vec3b>(j, i)[0] <= 127 && mask_img.at<cv::Vec3b>(j, i)[1] <= 127 && mask_img.at<cv::Vec3b>(j, i)[2] <= 127) &&
				(seg_img.at<cv::Vec3b>(j, i)[0] > 127 && seg_img.at<cv::Vec3b>(j, i)[1] > 127 && seg_img.at<cv::Vec3b>(j, i)[2] > 127)) {

				seg_back_count++;
			}

		}
	}




	back_ground = (double)seg_back_count / (double)mask_back_count;


	printf("mask_back_count = %d\n", mask_back_count);
	printf("seg_back_count = %d\n", seg_back_count);
	printf("mask_count = %lf\n", back_ground);


	cv::namedWindow("result", WINDOW_AUTOSIZE);
	cv::imshow("result", mask_img);

	cv::namedWindow("result2", WINDOW_AUTOSIZE);
	cv::imshow("result2", seg_img);

	waitKey();


	return 0;
}





int yolo_box_coordinate() {


	int count = 0;

	struct yolo_bounding_box yolo_buf_bbox;


	//YOLOの認識結果ファイルの読み込み
	std::cout << "File path : Yolo Bounding Box Info = " << std::endl;
	string sFilePath_YOLO_BoundingBox;
	cin >> sFilePath_YOLO_BoundingBox;


	//書き込む用のフォルダー指定
	std::cout << "folder for saving box coodinate = " << std::endl;
	string output_folder;
	cin >> output_folder;


	FILE *fp_yolo_bb = NULL;

	//errno_t・・・正常終了0，異常終了0以外
	errno_t err_yolo_bb;

	err_yolo_bb = fopen_s(&fp_yolo_bb, sFilePath_YOLO_BoundingBox.c_str(), "rt");

	//もし，異常終了なら
	if (err_yolo_bb != 0)
	{
		std::cout << "can not open!" << std::endl;
		std::cout << sFilePath_YOLO_BoundingBox << std::endl;
		return -1;
	}



	int n_yolo_bb_file_stream_size; //各行の文字の数





	while (1) {

		std::string filename = "output" + std::to_string(count) + ".txt";
		std::string output_data = output_folder + "\\" + filename;
		std::ofstream file_output;
		file_output.open(output_data);


		std::cout << filename << std::endl;

		while (1) {
			//テキストファイル内の各1行目の文字を格納するために用意
			char yolo_bb_file_BUFFER[YOLO_FILE_LINE_MAX_BUFFER_SIZE]; //1024

																	  //初期化
			n_yolo_bb_file_stream_size = 0;


			if (fgets(yolo_bb_file_BUFFER, YOLO_FILE_LINE_MAX_BUFFER_SIZE, fp_yolo_bb) != NULL) {

				n_yolo_bb_file_stream_size = (int)strlen(yolo_bb_file_BUFFER);

				if (n_yolo_bb_file_stream_size == 2) {

					file_output.close();

					count++;

					break;
				}
				else {

					sscanf_s(yolo_bb_file_BUFFER, "%s\t%lf\t%d\t%d\t%d\t%d", &yolo_buf_bbox.name, YOLO_LABEL_STRING_MAX_LENGTH, &yolo_buf_bbox.likelihood, &yolo_buf_bbox.x, &yolo_buf_bbox.y, &yolo_buf_bbox.width, &yolo_buf_bbox.height);

					
					file_output<<yolo_buf_bbox.name << "\t" << yolo_buf_bbox.x << "\t" << yolo_buf_bbox.y << "\t" << yolo_buf_bbox.width << "\t" << yolo_buf_bbox.height << "\n";
				}
			}
			else {
				printf_s("Detected the end of file!!\n");
				break;
			}
		}

		//もし，YOLOのテキストファイルの行の文字数が0なら…(＝テキストファイル内を全て開き終えたら…)
		if (n_yolo_bb_file_stream_size == 0)
		{
			break; //while文から抜ける
		}
	}


	return 0;
}





//動画で処理を行う
int object_tracking() {


	vector<struct point_cloud>point_cloud; //push_backするために用意
	struct point_cloud point; //LiDARの出力結果(x,y,z,intensity)

	std::vector<Point3f> point_cloud_LiDAR_yzx; //LiDAR 座標変換
	Point3f buf_LiDAR_yzx;

	vector<Point2f> imagePoints_LiDAR2ZED; //LiDAR 3D → 2D
	float fraction_x; //点群の小数部分
	float fraction_y; //点群の小数部分
	struct lidar_int imagePoints_LiDAR2ZED_int;//LiDARの点群の整数化
	vector<struct lidar_int>all_imagePoints_LiDAR2ZED_int;;
	vector<Point2f> imagePoints_LiDAR2ZED_in_region_person;
	vector<Point2f> imagePoints_LiDAR2ZED_in_region_container;

	//mask画像の色（person→赤，container→青）
	int b, g, r;

	string object_name;




	//yamlファイルの読み込み(rvecとtvecを使う)

	cv::Mat K1, D1;
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat mat_R_LiDAR2ZED = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::Mat mat_T_LiDAR2ZED = cv::Mat::zeros(3, 1, CV_64FC1);


	std::string sFilePath_PnP_rvec_tvec = "D:\\OpenCV_C++_practice\\!2_point_cloud_xyz11.txt_solve_PnP_K_D_rvec_tvec.yaml";

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



	//LiDARの点群データのテキストファイルのパスを標準入力するために用意
	std::cout << "File path : LiDAR Point Cloud Info = " << std::endl;
	string sFilePath_LiDAR_PointCloud;
	cin >> sFilePath_LiDAR_PointCloud;



	//MaskRCNNの画像を入力するためにファイルパスを指定
	std::cout << "input color movie = " << std::endl;
	std::string in_put_image_file_path;
	std::cin >> in_put_image_file_path;


	//MaskRCNNの画像を入力するためにファイルパスを指定
	std::cout << "input mask movie = " << std::endl;
	std::string in_put_image_file_path_mask;
	std::cin >> in_put_image_file_path_mask;


	//画像を出力するためにファイルパスを指定
	std::cout << "Files for storing movie = " << std::endl;
	std::string out_put_image_file_path;
	std::cin >> out_put_image_file_path;






	Mat color_image;
	Mat mask_image;
	Mat out_image;
	std::string img_out_name;

	//NNで求めた3次元点群を2次元に変換
	vector<Point3f> NN_3D;
	Point3f NN_3D_buf;
	vector<Point2f> NN_2D;



	//maskrcnnの認識の最小画素と最大画素を定義
	Point mask_min, mask_max;

	Point person_xy, container_xy;



	//動画（Videocapture）
	cv::VideoCapture frame_color;
	frame_color.open(in_put_image_file_path);

	int w = (int)frame_color.get(cv::CAP_PROP_FRAME_WIDTH);
	int h = (int)frame_color.get(cv::CAP_PROP_FRAME_HEIGHT);

	cv::VideoCapture frame_mask;
	frame_color.open(in_put_image_file_path_mask);


	cv::VideoWriter out_frame(out_put_image_file_path+"/output_movie.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 10, Size(w, h));




	while (1)
	{
		//カラー動画の読み込み
		frame_color >> color_image;

		int color_rows = color_image.rows;
		int color_cols = color_image.cols;


		//mask動画の読み込み
		frame_mask >> mask_image;

		int mask_rows = mask_image.rows;
		int mask_cols = mask_image.cols;


		out_image = mask_image.clone();




		//LiDARのテキストファイルを読み込むためのファイルを用意
		std::string sFilePath_point_cloud_on_image;
		sFilePath_point_cloud_on_image = sFilePath_LiDAR_PointCloud + "/point_cloud" + std::to_string(i_yolo_lidar) + ".txt_NaN_data_removed.txt";

		//ファイルを開く
		std::ifstream lidar_text_file;
		lidar_text_file.open(sFilePath_point_cloud_on_image);

		if (!lidar_text_file.is_open())
		{
			std::cerr << "Can not open " + sFilePath_point_cloud_on_image << std::endl;

			return -1;

		}



		//Maskrcnnの認識範囲内の『LiDAR』の点群を取得
		while (1)
		{


			if (!lidar_text_file.eof())
			{


				//LiDARのファイル内をスキャンしていく（全ての行を読み込む）
				lidar_text_file >> point.x;
				lidar_text_file >> point.y;
				lidar_text_file >> point.z;
				lidar_text_file >> point.intensity;


				point_cloud.push_back(point); //push_back




			} //もし，LiDARのテキストファイルに文字があるなら・・・の終わり
			else
			{
				lidar_text_file.close();
				break;
			}

		} //『Maskrcnnの認識範囲内の『LiDAR』の点群を取得』終了(while文)



		// change the coordinates of xyz -> yzx (LiDAR)
		for (int k = 0; k < (int)point_cloud.size(); k++)
		{
			buf_LiDAR_yzx.x = (float)-point_cloud[k].y;
			buf_LiDAR_yzx.y = (float)-point_cloud[k].z;
			buf_LiDAR_yzx.z = (float)point_cloud[k].x;

			point_cloud_LiDAR_yzx.push_back(buf_LiDAR_yzx);
		}

		cv::projectPoints(point_cloud_LiDAR_yzx, rvec, tvec, K1, D1, imagePoints_LiDAR2ZED);

		//LiDARの点群の整数化
		for (int k = 0; k < (int)imagePoints_LiDAR2ZED.size(); k++)
		{
			fraction_x = imagePoints_LiDAR2ZED[k].x - int(imagePoints_LiDAR2ZED[k].x);
			fraction_y = imagePoints_LiDAR2ZED[k].y - int(imagePoints_LiDAR2ZED[k].y);

			//x
			if (fraction_x < 0.5)
			{
				imagePoints_LiDAR2ZED_int.x = (int)imagePoints_LiDAR2ZED[k].x;
			}
			else if(fraction_x >= 0.5)
			{
				imagePoints_LiDAR2ZED_int.x = int(imagePoints_LiDAR2ZED[k].x + 1.0);
			}

			//y
			if (fraction_y < 0.5)
			{
				imagePoints_LiDAR2ZED_int.y = (int)imagePoints_LiDAR2ZED[k].y;
			}
			else if (fraction_y >= 0.5)
			{
				imagePoints_LiDAR2ZED_int.y = int(imagePoints_LiDAR2ZED[k].y + 1.0);
			}


			std::cout << imagePoints_LiDAR2ZED_int.x << "\t" << imagePoints_LiDAR2ZED_int.y << std::endl;

			all_imagePoints_LiDAR2ZED_int.push_back(imagePoints_LiDAR2ZED_int);

		}



		int person_num = 0;
		int container_num = 0;



		//for (int a = 206; a > 0; a = a - 50)
		//{

		//	for (int y = 0; y < mask_rows; y++)
		//	{

		//		int count_person = 0;
		//		int count_container = 0;


		//		for (int x = 0; x < mask_cols; x++)
		//		{

		//			b = mask_image.at<cv::Vec3b>(y, x)[0];
		//			g = mask_image.at<cv::Vec3b>(y, x)[1];
		//			r = mask_image.at<cv::Vec3b>(y, x)[2];



		//			//person
		//			if (b < 10 && g < 10 && r > a)
		//			{
		//				person_xy.x = x;
		//				person_xy.y = y;

		//				object_name = "person" + to_string(person_num);


		//				std::cout << object_name << std::endl;



		//				count_person++;

		//				if (count_person == 1)
		//				{
		//					mask_min.x = person_xy.x;
		//					mask_min.y = person_xy.y;

		//					mask_max.x = person_xy.x;
		//					mask_max.y = person_xy.y;
		//				}



		//				if (count_person > 1)
		//				{
		//					if (person_xy.x > mask_max.x)
		//					{
		//						mask_max.x = person_xy.x;
		//						mask_max.y = person_xy.y;
		//					}
		//				}

		//				std::cout << "x_min= " << mask_min.x << "\t" << "y_min=" << mask_min.y << "\t" << "x_max= " << mask_max.x << "\t" << "y_max=" << mask_max.y << std::endl;

		//				for (int j = 0; j < (int)all_imagePoints_LiDAR2ZED_int.size(); j++)
		//				{
		//					if (point_cloud_LiDAR_yzx[j].z < 0) continue; //
		//					if (all_imagePoints_LiDAR2ZED_int[j].x < 0 || all_imagePoints_LiDAR2ZED_int[j].x >= IMG_XSIZE || all_imagePoints_LiDAR2ZED_int[j].y < 0 || all_imagePoints_LiDAR2ZED_int[j].y >= IMG_YSIZE) continue;




		//					if (((all_imagePoints_LiDAR2ZED_int[j].x > mask_min.x && all_imagePoints_LiDAR2ZED_int[j].y > mask_min.y) && (all_imagePoints_LiDAR2ZED_int[j].x < mask_max.x && all_imagePoints_LiDAR2ZED_int[j].y < mask_max.y)))
		//					{
		//						if (mask_image.at<cv::Vec3b>(all_imagePoints_LiDAR2ZED_int[j].y, all_imagePoints_LiDAR2ZED_int[j].x)[0] != 0 && mask_image.at<cv::Vec3b>(all_imagePoints_LiDAR2ZED_int[j].y, all_imagePoints_LiDAR2ZED_int[j].x)[1] != 0 && mask_image.at<cv::Vec3b>(all_imagePoints_LiDAR2ZED_int[j].y, all_imagePoints_LiDAR2ZED_int[j].x)[2] != 0)
		//						{
		//							//認識範囲内のLiDAR点群をプッシュバック
		//							imagePoints_LiDAR2ZED_in_region_person.push_back(all_imagePoints_LiDAR2ZED_int[j]);

		//							cv::circle(out_image, cv::Point((int)all_imagePoints_LiDAR2ZED_int[j].x, (int)all_imagePoints_LiDAR2ZED_int[j].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);
		//						}
		//					}
		//				}

		//			}
		//			//container
		//			else if (b > a && g < 10 && r < 10)
		//			{
		//				container_xy.x = x;
		//				container_xy.y = y;

		//				object_name = "container" + to_string(container_num);


		//				std::cout << object_name << std::endl;



		//				container_num++;

		//				if (container_num == 1)
		//				{
		//					mask_min.x = container_xy.x;
		//					mask_min.y = container_xy.y;

		//					mask_max.x = container_xy.x;
		//					mask_max.y = container_xy.y;
		//				}



		//				if (container_num > 1)
		//				{
		//					if (container_xy.x > mask_max.x)
		//					{
		//						mask_max.x = container_xy.x;
		//						mask_max.y = container_xy.y;
		//					}
		//				}

		//				//std::cout << "x_min= " << mask_min.x << "\t" << "y_min=" << mask_min.y << "\t" << "x_max= " << mask_max.x << "\t" << "y_max=" << mask_max.y << std::endl;

		//				for (int j = 0; j < (int)all_imagePoints_LiDAR2ZED_int.size(); j++)
		//				{
		//					if (point_cloud_LiDAR_yzx[j].z < 0) continue; //
		//					if (all_imagePoints_LiDAR2ZED_int[j].x < 0 || all_imagePoints_LiDAR2ZED_int[j].x >= IMG_XSIZE || all_imagePoints_LiDAR2ZED_int[j].y < 0 || all_imagePoints_LiDAR2ZED_int[j].y >= IMG_YSIZE) continue;




		//					if (((all_imagePoints_LiDAR2ZED_int[j].x > mask_min.x && all_imagePoints_LiDAR2ZED_int[j].y > mask_min.y) && (all_imagePoints_LiDAR2ZED_int[j].x < mask_max.x && all_imagePoints_LiDAR2ZED_int[j].y < mask_max.y)))
		//					{
		//						if (mask_image.at<cv::Vec3b>(all_imagePoints_LiDAR2ZED_int[j].y, all_imagePoints_LiDAR2ZED_int[j].x)[0] != 0 && mask_image.at<cv::Vec3b>(all_imagePoints_LiDAR2ZED_int[j].y, all_imagePoints_LiDAR2ZED_int[j].x)[1] != 0 && mask_image.at<cv::Vec3b>(all_imagePoints_LiDAR2ZED_int[j].y, all_imagePoints_LiDAR2ZED_int[j].x)[2] != 0)
		//						{
		//							//認識範囲内のLiDAR点群をプッシュバック
		//							imagePoints_LiDAR2ZED_in_region_person.push_back(all_imagePoints_LiDAR2ZED_int[j]);

		//							cv::circle(out_image, cv::Point((int)all_imagePoints_LiDAR2ZED_int[j].x, (int)all_imagePoints_LiDAR2ZED_int[j].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);
		//						}
		//					}
		//				}

		//			}
		//			else if (b < 10 && g < 10 && r < 10)
		//			{
		//				continue;
		//			}

		//		}

		//	}


		//	if (imagePoints_LiDAR2ZED_in_region_person.size() != 0) person_num++;
		//	if (imagePoints_LiDAR2ZED_in_region_container.size() != 0) container_num++;;

		//	imagePoints_LiDAR2ZED_in_region_person.clear();
		//	imagePoints_LiDAR2ZED_in_region_person.shrink_to_fit();
		//	imagePoints_LiDAR2ZED_in_region_container.clear();
		//	imagePoints_LiDAR2ZED_in_region_container.shrink_to_fit();

		//}



		//動画の出力
		out_frame << out_image;


		std::cout << i_yolo_lidar << std::endl;


		i_yolo_lidar++;

	}


	frame_color.release();
	frame_mask.release();
	out_frame.release();
	cv::destroyAllWindows();


	return 0;
}









//試作（画像で処理を行う）
/*
int object_tracking() {


	vector<struct point_cloud>point_cloud; //push_backするために用意
	struct point_cloud point; //LiDARの出力結果(x,y,z,intensity)
	
	std::vector<Point3f> point_cloud_LiDAR_yzx; //LiDAR 座標変換
	Point3f buf_LiDAR_yzx;
	
	vector<Point2f> imagePoints_LiDAR2ZED; //LiDAR 3D → 2D
	float fraction_x; //点群の小数部分
	float fraction_y; //点群の小数部分
	Point imagePoints_LiDAR2ZED_int;//LiDARの点群の整数化
	vector<Point> all_imagePoints_LiDAR2ZED_int; 
	vector<Point2f> imagePoints_LiDAR2ZED_in_region_person;
	vector<Point2f> imagePoints_LiDAR2ZED_in_region_container;

	//mask画像の色（person→赤，container→青）
	int b, g, r;

	string object_name;




	//yamlファイルの読み込み(rvecとtvecを使う)

	cv::Mat K1, D1;
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat mat_R_LiDAR2ZED = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::Mat mat_T_LiDAR2ZED = cv::Mat::zeros(3, 1, CV_64FC1);


	std::string sFilePath_PnP_rvec_tvec = "D:\\OpenCV_C++_practice\\!2_point_cloud_xyz11.txt_solve_PnP_K_D_rvec_tvec.yaml";

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



	//LiDARの点群データのテキストファイルのパスを標準入力するために用意
	std::cout << "File path : LiDAR Point Cloud Info = " << std::endl;
	string sFilePath_LiDAR_PointCloud;
	cin >> sFilePath_LiDAR_PointCloud;



	//MaskRCNNの画像を入力するためにファイルパスを指定
	std::cout << "input color images = " << std::endl;
	std::string in_put_image_file_path;
	std::cin >> in_put_image_file_path;


	//MaskRCNNの画像を入力するためにファイルパスを指定
	std::cout << "input mask images = " << std::endl;
	std::string in_put_image_file_path_mask;
	std::cin >> in_put_image_file_path_mask;


	//画像を出力するためにファイルパスを指定
	std::cout << "Files for storing images = " << std::endl;
	std::string out_put_image_file_path;
	std::cin >> out_put_image_file_path;






	Mat color_image;
	Mat mask_image;
	Mat out_image;
	std::string img_out_name;

	//NNで求めた3次元点群を2次元に変換
	vector<Point3f> NN_3D;
	Point3f NN_3D_buf;
	vector<Point2f> NN_2D;



	//maskrcnnの認識の最小画素と最大画素を定義
	Point mask_min, mask_max;

	Point person_xy, container_xy;




	while (1) 
	{
	
		//カラー画像の読み込み
		std::string color_img_in_name;
		color_img_in_name = in_put_image_file_path + "/image" + std::to_string(i_yolo_lidar) + ".png";
		color_image = cv::imread(color_img_in_name);

		int color_rows = color_image.rows;
		int color_cols = color_image.cols;


		//mask画像の読み込み
		std::string mask_img_in_name;
		mask_img_in_name = in_put_image_file_path_mask + "/image" + std::to_string(i_yolo_lidar) + ".png";
		mask_image = cv::imread(mask_img_in_name);

		int mask_rows = mask_image.rows;
		int mask_cols = mask_image.cols;


		out_image = mask_image.clone();


		

		//LiDARのテキストファイルを読み込むためのファイルを用意
		std::string sFilePath_point_cloud_on_image;
		sFilePath_point_cloud_on_image = sFilePath_LiDAR_PointCloud + "/point_cloud" + std::to_string(i_yolo_lidar) + ".txt_NaN_data_removed.txt";

		//ファイルを開く
		std::ifstream lidar_text_file;
		lidar_text_file.open(sFilePath_point_cloud_on_image);

		if (!lidar_text_file.is_open())
		{
			std::cerr << "Can not open " + sFilePath_point_cloud_on_image << std::endl;

			return -1;

		}



		//Maskrcnnの認識範囲内の『LiDAR』の点群を取得
		while (1)
		{


			if (!lidar_text_file.eof())
			{


				//LiDARのファイル内をスキャンしていく（全ての行を読み込む）
				lidar_text_file >> point.x;
				lidar_text_file >> point.y;
				lidar_text_file >> point.z;
				lidar_text_file >> point.intensity;


				point_cloud.push_back(point); //push_back




			} //もし，LiDARのテキストファイルに文字があるなら・・・の終わり
			else
			{
				lidar_text_file.close();
				break;
			}

		} //『Maskrcnnの認識範囲内の『LiDAR』の点群を取得』終了(while文)



		// change the coordinates of xyz -> yzx (LiDAR)
		for (int k = 0; k < (int)point_cloud.size(); k++)
		{
			buf_LiDAR_yzx.x = (float)-point_cloud[k].y;
			buf_LiDAR_yzx.y = (float)-point_cloud[k].z;
			buf_LiDAR_yzx.z = (float)point_cloud[k].x;

			point_cloud_LiDAR_yzx.push_back(buf_LiDAR_yzx);
		}
		
		cv::projectPoints(point_cloud_LiDAR_yzx, rvec, tvec, K1, D1, imagePoints_LiDAR2ZED);


		//LiDARの点群の整数化
		for (int k = 0; k < (int)imagePoints_LiDAR2ZED.size(); k++)
		{
			fraction_x = imagePoints_LiDAR2ZED[k].x - int(imagePoints_LiDAR2ZED[k].x);
			fraction_y = imagePoints_LiDAR2ZED[k].y - int(imagePoints_LiDAR2ZED[k].y);
		
			//x
			if (fraction_x < 0.5)
			{
				imagePoints_LiDAR2ZED_int.x = (int)imagePoints_LiDAR2ZED[k].x;
			}
			else if(fraction_x >= 0.5)
			{
				imagePoints_LiDAR2ZED_int.x = (int)imagePoints_LiDAR2ZED[k].x + 1;
			}
		
			//y
			if (fraction_y < 0.5)
			{
				imagePoints_LiDAR2ZED_int.y = (int)imagePoints_LiDAR2ZED[k].y;
			}
			else if (fraction_y >= 0.5)
			{
				imagePoints_LiDAR2ZED_int.y = (int)imagePoints_LiDAR2ZED[k].y + 1;
			}
		
		
			all_imagePoints_LiDAR2ZED_int.push_back(imagePoints_LiDAR2ZED_int);
		
		}



		int person_num = 0;
		int container_num = 0;
		


		for (int a = 206; a > 0; a = a - 50)
		{

			for (int y = 0; y < mask_rows; y++)
			{

				int count_person = 0;
				int count_container = 0;


				for (int x = 0; x < mask_cols; x++)
				{

					b = mask_image.at<cv::Vec3b>(y, x)[0];
					g = mask_image.at<cv::Vec3b>(y, x)[1];
					r = mask_image.at<cv::Vec3b>(y, x)[2];



					//person
					if (b < 10 && g < 10 && r > a)
					{
						person_xy.x = x;
						person_xy.y = y;

						object_name = "person" + to_string(person_num);


						std::cout << object_name << std::endl;



						count_person++;

						if (count_person == 1)
						{
							mask_min.x = person_xy.x;
							mask_min.y = person_xy.y;

							mask_max.x = person_xy.x;
							mask_max.y = person_xy.y;
						}



						if (count_person > 1)
						{
							if (person_xy.x > mask_max.x)
							{
								mask_max.x = person_xy.x;
								mask_max.y = person_xy.y;
							}
						}

						std::cout << "x_min= " << mask_min.x << "\t" << "y_min=" << mask_min.y << "\t" << "x_max= " << mask_max.x << "\t" << "y_max=" << mask_max.y << std::endl;

						for (int j = 0; j < (int)imagePoints_LiDAR2ZED.size(); j++)
						{
							if (point_cloud_LiDAR_yzx[j].z < 0) continue; //
							if (imagePoints_LiDAR2ZED[j].x < 0 || imagePoints_LiDAR2ZED[j].x >= IMG_XSIZE || imagePoints_LiDAR2ZED[j].y < 0 || imagePoints_LiDAR2ZED[j].y >= IMG_YSIZE) continue;




							if (((int)imagePoints_LiDAR2ZED[j].x > mask_min.x && (int)imagePoints_LiDAR2ZED[j].y > mask_min.y - 3) && ((int)imagePoints_LiDAR2ZED[j].x < mask_max.x && (int)imagePoints_LiDAR2ZED[j].y < mask_max.y))
							{
								if (mask_image.at<cv::Vec3b>((int)imagePoints_LiDAR2ZED[j].y, (int)imagePoints_LiDAR2ZED[j].x)[0] != 0 && mask_image.at<cv::Vec3b>((int)imagePoints_LiDAR2ZED[j].y, (int)imagePoints_LiDAR2ZED[j].x)[1] != 0 && mask_image.at<cv::Vec3b>((int)imagePoints_LiDAR2ZED[j].y, (int)imagePoints_LiDAR2ZED[j].x)[2] != 0)
								{
									//認識範囲内のLiDAR点群をプッシュバック
									imagePoints_LiDAR2ZED_in_region_person.push_back(imagePoints_LiDAR2ZED[j]);
									
									cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[j].x, (int)imagePoints_LiDAR2ZED[j].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);
								}
							}
						}

					}
					//container
					else if (b > a && g < 10 && r < 10)
					{
						container_xy.x = x;
						container_xy.y = y;

						object_name = "container" + to_string(container_num);


						std::cout << object_name << std::endl;



						container_num++;

						if (container_num == 1)
						{
							mask_min.x = container_xy.x;
							mask_min.y = container_xy.y;

							mask_max.x = container_xy.x;
							mask_max.y = container_xy.y;
						}



						if (container_num > 1)
						{
							if (container_xy.x > mask_max.x)
							{
								mask_max.x = container_xy.x;
								mask_max.y = container_xy.y;
							}
						}

						//std::cout << "x_min= " << mask_min.x << "\t" << "y_min=" << mask_min.y << "\t" << "x_max= " << mask_max.x << "\t" << "y_max=" << mask_max.y << std::endl;

						for (int j = 0; j < (int)imagePoints_LiDAR2ZED.size(); j++)
						{
							if (point_cloud_LiDAR_yzx[j].z < 0) continue; //
							if (imagePoints_LiDAR2ZED[j].x < 0 || imagePoints_LiDAR2ZED[j].x >= IMG_XSIZE || imagePoints_LiDAR2ZED[j].y < 0 || imagePoints_LiDAR2ZED[j].y >= IMG_YSIZE) continue;




							if (((int)imagePoints_LiDAR2ZED[j].x > mask_min.x && (int)imagePoints_LiDAR2ZED[j].y > mask_min.y - 3) && ((int)imagePoints_LiDAR2ZED[j].x < mask_max.x && (int)imagePoints_LiDAR2ZED[j].y < mask_max.y))
							{
								if (mask_image.at<cv::Vec3b>((int)imagePoints_LiDAR2ZED[j].y, (int)imagePoints_LiDAR2ZED[j].x)[0] != 0 && mask_image.at<cv::Vec3b>((int)imagePoints_LiDAR2ZED[j].y, (int)imagePoints_LiDAR2ZED[j].x)[1] != 0 && mask_image.at<cv::Vec3b>((int)imagePoints_LiDAR2ZED[j].y, (int)imagePoints_LiDAR2ZED[j].x)[2] != 0)
								{
									//認識範囲内のLiDAR点群をプッシュバック
									imagePoints_LiDAR2ZED_in_region_container.push_back(imagePoints_LiDAR2ZED[j]);

									cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[j].x, (int)imagePoints_LiDAR2ZED[j].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);
								}
							}
						}


					}
					else if (b == 0 && g == 0 && r == 0)
					{
						continue;
					}





					//for (int j = 0; j < (int)imagePoints_LiDAR2ZED.size(); j++)
					//{
					//	if (point_cloud_LiDAR_yzx[j].z < 0) continue; //
					//	if (imagePoints_LiDAR2ZED[j].x < 0 || imagePoints_LiDAR2ZED[j].x >= IMG_XSIZE || imagePoints_LiDAR2ZED[j].y < 0 || imagePoints_LiDAR2ZED[j].y >= IMG_YSIZE) continue;




					//}


				}

			}


			if (imagePoints_LiDAR2ZED_in_region_person.size() != 0) person_num++;
			if (imagePoints_LiDAR2ZED_in_region_container.size() != 0) container_num++;;

			imagePoints_LiDAR2ZED_in_region_person.clear();
			imagePoints_LiDAR2ZED_in_region_person.shrink_to_fit();
			imagePoints_LiDAR2ZED_in_region_container.clear();
			imagePoints_LiDAR2ZED_in_region_container.shrink_to_fit();

		}

	

		//動画の出力
		img_out_name = out_put_image_file_path + "/image" + std::to_string(i_yolo_lidar) + ".png";
		cv::imwrite(img_out_name, out_image);

		std::cout << i_yolo_lidar << std::endl;


		i_yolo_lidar++;
	
	}




	return 0;
}

*/
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
#define LiDAR_FILE_LINE_MAX_BUFFER_SIZE 1024

//YOLOで学習させる物体の名称を入れるファイル可能最大領域を定義
#define YOLO_LABEL_STRING_MAX_LENGTH  100


//using宣言
using namespace cv;
using namespace std;




//構造体//
//YOLOの認識結果のテキストファイル内の項目を読み込むために用意
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

struct point_cloud_after_conversion {
	float x;
	float y;
	float z;

	int type;
};

// nn method
struct NN_type
{
	double cx;
	double cy;
	double cz;
	double sumx;
	double sumy;
	double sumz;
	int count;

	string yolo_name;
};

struct tracking_object_type {

	float x;
	float y;
	float z;
	int type; //Object name in the bounding box(ex:container→0, person→1)

};

//ヒストグラムを表示するために用意（仮）
struct hist_data
{
	int i;
	int count;
};



//gnuplotで降順の結果を表示
struct sort_data {
	int i;
	double distance;
	float x;
	float y;
	float z;
};




struct struct_rectangle {

	double x0;
	double y0;
	double x1;
	double y1;

};



//プロトタイプ宣言
void menu_screen(int &ncommand);
int yolo_lidar();
double masuda_calc_IoU(vector<vector<double>> &all_d_IoU_input1, vector<vector<double>> &all_d_IoU_input2,
	vector<struct yolo_bounding_box> &IN_yolo_BBox_buf_input1, vector<struct yolo_bounding_box> &IN_yolo_BBox_buf_input2,
	vector<vector<struct yolo_bounding_box>> &Objects_yolo_BBox_input1, vector<vector<struct yolo_bounding_box>> &Objects_yolo_BBox_input2);
double calc_IoU(struct struct_rectangle *a, struct struct_rectangle *b);
int NN_method(vector<struct point_cloud_after_conversion> &input_data_xyz_3D, double Thresh, vector<struct NN_type> &nn);
int NN_method_representative(vector<struct NN_type> nn, vector<struct NN_type> &all_nn_representative, double object_range, int Thresh_clustering_num);





char *container;
char *person;



//main関数
void main(int argc, const char* argv[])
{
	int nCommand;
	int n = 0; //画像から動画を作成する際のフレーム数
	int nFlag;
	nFlag = 1;

	while (nFlag)
	{
		menu_screen(nCommand);

		switch (nCommand)
		{
		case 1:
			yolo_lidar();
			break;




		case 0:
			nFlag = -1;
			exit(1);
		}
	}
}



//メニュー画面の関数
void menu_screen(int &ncommand)
{

	printf("\n");
	printf("----------------------------------------------------\n");
	printf("<<1>>:||main||物体追跡\n");
	printf("<<2>>:〇〇\n");
	printf("<<0>>:終了します．\n");
	printf("----------------------------------------------------\n");
	printf("\n");

	printf("command=");
	scanf_s("%d", &ncommand);


}



// ||main||物体追跡
int yolo_lidar()
{
	int i_yolo_lidar = 0;
	int n_yolo_bb_file_stream_size; //各行の文字の数
	char yolo_bb_file_BUFFER[YOLO_FILE_LINE_MAX_BUFFER_SIZE]; //テキストファイル内の各1行目の文字を格納するために用意(1024)

	vector<struct yolo_bounding_box> IN_yolo_BBox_buf_container;
	vector<struct yolo_bounding_box> IN_yolo_BBox_buf_person;

	vector<vector<struct yolo_bounding_box>> Objects_yolo_BBox_container;
	vector<vector<struct yolo_bounding_box>> Objects_yolo_BBox_person;

	struct yolo_bounding_box yolo_buf_bbox;

	vector<struct point_cloud>point_cloud; //push_backするために用意
	struct point_cloud point; //LiDARの出力結果(x,y,z,intensity)

	std::vector<Point3f> point_cloud_LiDAR_yzx;
	Point3f buf_LiDAR_yzx;

	std::vector<double>all_lidar_distance;
	double lidar_distance;

	vector<Point2f> imagePoints_LiDAR2ZED;//LiDAR 3D → 2D//

	std::vector<Point2f>all_point_cloud_in_a_boundingbox_2D;
	cv::Point2f point_cloud_in_a_boundingbox_2D;

	std::vector<struct point_cloud_after_conversion>all_point_cloud_in_a_boundingbox_3D;
	struct point_cloud_after_conversion point_cloud_in_a_boundingbox_3D;

	//gnuplotにBounding box内の全点群を表示するために用意
	std::vector<struct point_cloud_after_conversion>all_point_cloud_in_a_boundingbox_3D_container_for_gnuplot;
	struct point_cloud_after_conversion point_cloud_in_a_boundingbox_3D_container_for_gnuplot;

	std::vector<struct point_cloud_after_conversion>all_point_cloud_in_a_boundingbox_3D_person_for_gnuplot;
	struct point_cloud_after_conversion point_cloud_in_a_boundingbox_3D_person_for_gnuplot;

	std::vector<double>all_lidar_distance_in_boundingbox;
	double lidar_distance_in_boundingbox;

	vector<struct tracking_object_type> IN_Objects_buf_container;
	vector<struct tracking_object_type> IN_Objects_container;

	vector<struct tracking_object_type> IN_Objects_buf_person;
	vector<struct tracking_object_type> IN_Objects_person;

	double min_nn_distance;
	double nn_distance;

	vector<struct NN_type> NN;
	vector<struct NN_type> all_NN_representative;


	Mat img_in;
	Mat img_in_left;
	Mat img_out;
	std::string img_out_name;
	//NNで求めた3次元点群を2次元に変換
	vector<Point3f> NN_3D;
	Point3f NN_3D_buf;
	vector<Point2f> NN_2D;
	//IoUの結果をテキストファイルに書き出す
	vector<vector<double>> All_d_IoU_container;
	vector<vector<double>> All_d_IoU_person;


	FILE *gid;
	gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");



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


	//YOLOの認識結果ファイルの読み込み
	std::cout << "File path : Yolo Bounding Box Info = " << std::endl;
	string sFilePath_YOLO_BoundingBox;
	cin >> sFilePath_YOLO_BoundingBox;


	FILE *fp_yolo_bb = NULL;
	errno_t err_yolo_bb;
	err_yolo_bb = fopen_s(&fp_yolo_bb, sFilePath_YOLO_BoundingBox.c_str(), "rt");

	if (err_yolo_bb != 0)
	{
		std::cout << "can not open!" << std::endl;
		std::cout << sFilePath_YOLO_BoundingBox << std::endl;
		return -1;
	}


	//LiDARの点群データのテキストファイルのパスを標準入力するために用意
	std::cout << "File path : LiDAR Point Cloud Info = " << std::endl;
	string sFilePath_LiDAR_PointCloud;
	cin >> sFilePath_LiDAR_PointCloud;


	//抽出した点群をテキストファイルに保存
	/*std::cout << "Files for storing text file（per_sheet...） = " << std::endl;
	std::string per_yolo_lidar_distance_result_data_path;
	std::cin >> per_yolo_lidar_distance_result_data_path;*/


	//画像を入力するためにファイルパスを指定
	std::cout << "input images = " << std::endl;
	std::string in_put_image_file_path;
	std::cin >> in_put_image_file_path;

	//画像を出力するためにファイルパスを指定
	std::cout << "Files for storing images = " << std::endl;
	std::string out_put_image_file_path;
	std::cin >> out_put_image_file_path;


	//NN法の結果をテキストファイルに保存するために用意
	/*std::cout << "Folder name for saving the results of the NN method = " << std::endl;
	std::string NN_textfile;
	std::cin >> NN_textfile;

	string sFilePath_NN;
	FILE *fp_w_NN;
	errno_t err;

	sFilePath_NN = NN_textfile + "/" + "output_NN.txt";
	err = fopen_s(&fp_w_NN, sFilePath_NN.c_str(), "wt");

	if (err != 0)
	{
	std::cout << "cannot open the NN file." << endl;
	}*/


	/*std::cout << "Folder name for saving the results of the IoU = " << std::endl;
	std::string IoU_textfile;
	std::cin >> IoU_textfile;

	string sFilePath_IoU_conteiner;
	FILE *fp_w_IoU_conteiner;
	errno_t err_IoU_conteiner;

	sFilePath_IoU_conteiner = IoU_textfile + "/container/output_IoU.txt";
	err_IoU_conteiner = fopen_s(&fp_w_IoU_conteiner, sFilePath_IoU_conteiner.c_str(), "wt");

	if (err_IoU_conteiner != 0)
	{
	std::cout << "can not open container" << std::endl;
	}


	string sFilePath_IoU_person;
	FILE *fp_w_IoU_person;
	errno_t err_IoU_person;

	sFilePath_IoU_person = IoU_textfile + "/person/output_IoU.txt";
	err_IoU_person = fopen_s(&fp_w_IoU_person, sFilePath_IoU_person.c_str(), "wt");

	if (err_IoU_person != 0)
	{
	std::cout << "can not open person" << std::endl;
	}*/




	//全体の画像に対しての処理
	while (1)
	{
		//fprintf_s(yolo_lidar_distance_result_data, "%d\n", i); //何枚目の画像なのかが分かるように，出力用のテキストファイルに枚数を書いておく
		//fprintf_s(yolo_result_textfile_path, "%d\n", i); //YOLOの結果のテキストファイルに番号を加えておく




		////////////////////////////////////////////////////////////////////////////

		//addition date is 2020_11_27 
		//1つのLiDARのデータごとにテキストファイルに保存するためにファイルを用意
		//std::string per_yolo_lidar_distance_result_data_path_name;
		//per_yolo_lidar_distance_result_data_path_name = per_yolo_lidar_distance_result_data_path + "/YOLO_LiDAR_distance_result_data" + std::to_string(i_yolo_lidar) + ".txt";


		////ファイルを開く
		//std::ofstream per_yolo_lidar_distance_result_data;
		//per_yolo_lidar_distance_result_data.open(per_yolo_lidar_distance_result_data_path_name);

		//if (!per_yolo_lidar_distance_result_data.is_open())
		//{
		//	std::cerr << "Can not open " + per_yolo_lidar_distance_result_data_path_name << std::endl;
		//	return -1;
		//}


		////////////////////////////////////////////////////////////////////////////




		//12_1
		//入力画像を用意
		////////////////////////////////////////////////////////////////////////////
		std::string img_in_name;
		img_in_name = in_put_image_file_path + "/image" + std::to_string(i_yolo_lidar) + ".png";
		img_in = cv::imread(img_in_name);

		//ステレオ画像の半分だけ使いたい時
		//img_in_left = img_in(cv::Rect(0, 0, img_in.cols / 2, img_in.rows));
		//img_out = img_in_left.clone();


		//ステレオ画像の処理が終わった後の画像を使いたい時
		img_out = img_in.clone();
		////////////////////////////////////////////////////////////////////////////






		//1枚ずつの画像に対しての処理
		while (1)
		{


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


			//テキストファイル内の各1行目の文字を格納するために用意
			char yolo_bb_file_BUFFER[YOLO_FILE_LINE_MAX_BUFFER_SIZE]; //1024


																	  //初期化
			n_yolo_bb_file_stream_size = 0;



			//もし，テキストファイル内に文字があるなら
			if (fgets(yolo_bb_file_BUFFER, YOLO_FILE_LINE_MAX_BUFFER_SIZE, fp_yolo_bb) != NULL)
			{

				n_yolo_bb_file_stream_size = (int)strlen(yolo_bb_file_BUFFER);



				//transporting = strstr(yolo_bb_file_BUFFER, "transporting");
				//container = strstr(yolo_bb_file_BUFFER, "container");
				//person = strstr(yolo_bb_file_BUFFER, "person");



				if (n_yolo_bb_file_stream_size == 2)
				{


					//2021_1_20 Addition to calculate the minimum point between the previous point cloud and old point cloud.
					//int moving_trajectory();




					//////////////////////////////////////////////////////////////////////////////////////////////////////////////

					//2021_1_26 add for write the median to the text file.
					//2021_1_26 add 
					//Bounding box内のLiDAR点群の中央値をテキストファイルに出力
					/*for (size_t i = 0; i < IN_Objects_buf_container.size(); i++)
					{
					OutputFile_container << IN_Objects_buf_container[i].x << "\t" << IN_Objects_buf_container[i].y << "\t" << IN_Objects_buf_container[i].z << "\n";

					}

					OutputFile_container << "\n";
					OutputFile_container << "\n";



					for (size_t i = 0; i < IN_Objects_buf_person.size(); i++)
					{
					OutputFile_person << IN_Objects_buf_person[i].x << "\t" << IN_Objects_buf_person[i].y << "\t" << IN_Objects_buf_person[i].z << "\n";
					}

					OutputFile_person << "\n";
					OutputFile_person << "\n";*/

					//////////////////////////////////////////////////////////////////////////////////////////////////////////////




					//2021_1_17 add for draw the result of IoU
					masuda_calc_IoU(All_d_IoU_container, All_d_IoU_person, );


					/*for (size_t a = 0; a < Objects_yolo_BBox_container.size(); a++)
					{
					for (size_t b = 1; b < Objects_yolo_BBox_container[a].size(); b++)
					{

					cv::line(img_out, cv::Point(Objects_yolo_BBox_container[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].width, Objects_yolo_BBox_container[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].height), cv::Point(Objects_yolo_BBox_container[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].width, Objects_yolo_BBox_container[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].height), cv::Scalar(155, 114, 155), 1, cv::LINE_AA);
					}
					}


					for (size_t a = 0; a < Objects_yolo_BBox_person.size(); a++)
					{
					for (size_t b = 1; b < Objects_yolo_BBox_person[a].size(); b++)
					{

					cv::line(img_out, cv::Point(Objects_yolo_BBox_person[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].width, Objects_yolo_BBox_person[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].height), cv::Point(Objects_yolo_BBox_person[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].width, Objects_yolo_BBox_person[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].height), cv::Scalar(0, 217, 255), 1, cv::LINE_AA);

					}
					}*/





					IN_yolo_BBox_buf_container.clear();    IN_yolo_BBox_buf_container.shrink_to_fit();
					IN_yolo_BBox_buf_person.clear();       IN_yolo_BBox_buf_person.shrink_to_fit();
					//In_yolo_BBox_container.clear();        In_yolo_BBox_container.shrink_to_fit();
					//In_yolo_BBox_person.clear();           In_yolo_BBox_person.shrink_to_fit();




					//↓add for drawing the moving trajectory 
					//vector_test_sample(gid);

					IN_Objects_buf_container.clear();      IN_Objects_buf_container.shrink_to_fit();
					IN_Objects_container.clear();          IN_Objects_container.shrink_to_fit();
					IN_Objects_buf_person.clear();      IN_Objects_buf_person.shrink_to_fit();
					IN_Objects_person.clear();          IN_Objects_person.shrink_to_fit();





					//Bounding box内のLiDAR点群をgnuplotに表示（2021_1_6追記） (2つ目のgnuplot_mapping)
					//gnuplot_mapping();





					//2021_1_25 add
					all_point_cloud_in_a_boundingbox_3D_container_for_gnuplot.clear();    all_point_cloud_in_a_boundingbox_3D_container_for_gnuplot.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_3D_person_for_gnuplot.clear();       all_point_cloud_in_a_boundingbox_3D_person_for_gnuplot.shrink_to_fit();


					//2021_1_20 addition
					//extract median only(per one frame)
					//all_only_median.clear();                all_only_median.shrink_to_fit();



					i_yolo_lidar++; //次のLiDARの点群の座標が書かれたテキストファイルを読み込む

					std::cout << i_yolo_lidar << std::endl; //何枚目かを表示しておく


					break; //while文から抜ける

				}
				else
				{


					sscanf_s(yolo_bb_file_BUFFER, "%s\t%lf\t%d\t%d\t%d\t%d", &yolo_buf_bbox.name, YOLO_LABEL_STRING_MAX_LENGTH, &yolo_buf_bbox.likelihood, &yolo_buf_bbox.x, &yolo_buf_bbox.y, &yolo_buf_bbox.width, &yolo_buf_bbox.height);






					//抜き出したYOLOのデータをテキストファイルに保存する．
					//fprintf_s(yolo_result_textfile_path, "%s\t%d\t%d\t%d\t%d\n", yolo_buf_bbox.name, yolo_buf_bbox.x, yolo_buf_bbox.y, yolo_buf_bbox.width, yolo_buf_bbox.height);


					//YOLO内のLiDARの結果を書き込むファイルに『YOLOの結果』を書き込んでいく
					//fprintf_s(yolo_lidar_distance_result_data, "%s\t%d\t%d\t%d\t%d\n", yolo_buf_bbox.name, yolo_buf_bbox.x, yolo_buf_bbox.y, yolo_buf_bbox.width, yolo_buf_bbox.height);





					////////////////////////////////////////////////////////////////////////////

					//addition date is 2020_11_27 →　2021_1_6 コメントアウト
					//ファイルごとにバウンディングボックス内のLiDARの結果を保存
					//per_yolo_lidar_distance_result_data << yolo_buf_bbox.name << "\t" << yolo_buf_bbox.x << "\t" << yolo_buf_bbox.y << "\t" << yolo_buf_bbox.width << "\t" << yolo_buf_bbox.height << "\n";






					//fprintf_s(per_yolo_lidar_distance_result_data, "%s\t%d\t%d\t%d\t%d\n", yolo_buf_bbox.name, yolo_buf_bbox.x, yolo_buf_bbox.y, yolo_buf_bbox.width, yolo_buf_bbox.height);

					////////////////////////////////////////////////////////////////////////////

					container = strstr(yolo_bb_file_BUFFER, "container");
					person = strstr(yolo_bb_file_BUFFER, "person");


					//2021_1_27 add for draw the result of IoU
					if (container)
					{
						IN_yolo_BBox_buf_container.push_back(yolo_buf_bbox);
					}
					else if (person)
					{
						IN_yolo_BBox_buf_person.push_back(yolo_buf_bbox);
					}




					/*std::random_device rd;
					std::mt19937 mt(rd());
					std::uniform_int_distribution<int> rand(0, 255);
					int color_b = rand(mt);
					int color_g = rand(mt);
					int color_r = rand(mt);*/





					/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

					//⓪

					//drawing the result of IoU
					for (size_t a = 0; a < Objects_yolo_BBox_container.size(); a++)
					{

						for (size_t b = 1; b < Objects_yolo_BBox_container[a].size(); b++)
						{

							int color_b = 255 / (a + 1);
							int color_g = 255 / (2 * a + 1);
							int color_r = (a + 1) * 30;

							cv::line(img_out, cv::Point(Objects_yolo_BBox_container[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].width, Objects_yolo_BBox_container[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].height), cv::Point(Objects_yolo_BBox_container[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].width, Objects_yolo_BBox_container[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].height), cv::Scalar(color_b, color_g, color_r), 2, cv::LINE_AA);





							//cv::line(img_out, cv::Point(Objects_yolo_BBox_container[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].width, Objects_yolo_BBox_container[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].height), cv::Point(Objects_yolo_BBox_container[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].width, Objects_yolo_BBox_container[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].height), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);


							//155, 114, 155
						}
					}


					for (size_t a = 0; a < Objects_yolo_BBox_person.size(); a++)
					{
						for (size_t b = 1; b < Objects_yolo_BBox_person[a].size(); b++)
						{


							int color_b = 255 / (2 * a + 1);
							int color_g = 255 / ((1 / 2)*a + 1);
							int color_r = 255 / ((1 / 3)*a + 1);
							cv::line(img_out, cv::Point(Objects_yolo_BBox_person[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].width, Objects_yolo_BBox_person[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].height), cv::Point(Objects_yolo_BBox_person[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].width, Objects_yolo_BBox_person[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].height), cv::Scalar(color_b, color_g, color_r), 2, cv::LINE_AA);





							//cv::line(img_out, cv::Point(Objects_yolo_BBox_person[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].width, Objects_yolo_BBox_person[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].height), cv::Point(Objects_yolo_BBox_person[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].width, Objects_yolo_BBox_person[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].height), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);


							//0, 217, 255
						}
					}



					/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////







					
				
					cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(0, 0, 255), 1);



					//2021_1_27 add
					//拡大した場合に使う
					//cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y - (1.0 / 5.0)*yolo_buf_bbox.height)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width + (1.0 / 5.0)*yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(0, 0, 0), 2);


					/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

					//①

					//2021_1_28 add
					//拡大した場合に使う
					cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x - (1.0 / 5.0)*yolo_buf_bbox.width), int(yolo_buf_bbox.y - (1.0 / 5.0)*yolo_buf_bbox.height)), cv::Point(int(yolo_buf_bbox.x + (6.0 / 5.0)*yolo_buf_bbox.width), int(yolo_buf_bbox.y + (6.0 / 5.0)*yolo_buf_bbox.height)), cv::Scalar(0, 0, 0), 1);

					/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





					//12_17追記
					//縮小したバウンディングボックスを描写
					//cv::rectangle(img_out, cv::Point((int)((double)yolo_buf_bbox.x + (double)yolo_buf_bbox.width / denominator), (int)((double)yolo_buf_bbox.y + (double)yolo_buf_bbox.height / denominator)), cv::Point((int)((double)yolo_buf_bbox.x + numerator * (double)yolo_buf_bbox.width / denominator), (int)((double)yolo_buf_bbox.y + numerator * (double)yolo_buf_bbox.height / denominator)), cv::Scalar(0, 0, 0), 1);





					//12_1
					////////////////////////////////////////////////////////////////////////////
					//画像内にYOLOの結果を反映させる

					//cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(0, 0, 255), 1);

					////////////////////////////////////////////////////////////////////////////








					//LiDAR






					//1行ずつ格納ためのバッファファイルを用意
					std::string lidar_textfile_line_BUFFER;




					//YOLOのバウンディングボックス内の『LiDAR』の点群を取得
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

					} //『YOLOのバウンディングボックス内の『LiDAR』の点群を取得』終了(while文)







					  // change the coordinates of xyz -> yzx (LiDAR)
					for (int k = 0; k < (int)point_cloud.size(); k++)
					{
						buf_LiDAR_yzx.x = (float)-point_cloud[k].y;
						buf_LiDAR_yzx.y = (float)-point_cloud[k].z;
						buf_LiDAR_yzx.z = (float)point_cloud[k].x;


						lidar_distance = sqrt(pow(buf_LiDAR_yzx.x, 2.0) + pow(buf_LiDAR_yzx.y, 2.0) + pow(buf_LiDAR_yzx.z, 2.0));

						all_lidar_distance.push_back(lidar_distance);
						point_cloud_LiDAR_yzx.push_back(buf_LiDAR_yzx);


					}





					cv::projectPoints(point_cloud_LiDAR_yzx, rvec, tvec, K1, D1, imagePoints_LiDAR2ZED);










					//プッシュバックしたpoint_cloudのデータをテキストファイルに書き込んでいく
					for (int j = 0; j < (int)imagePoints_LiDAR2ZED.size(); j++)
					{

						if (point_cloud_LiDAR_yzx[j].z < 0) continue; //
						if (imagePoints_LiDAR2ZED[j].x < 0 || imagePoints_LiDAR2ZED[j].x >= IMG_XSIZE || imagePoints_LiDAR2ZED[j].y < 0 || imagePoints_LiDAR2ZED[j].y >= IMG_YSIZE) continue;







						//YOLOのバウンディングボックス内での処理



						//LiDARの点群がYOLOのバウンディングボックス内の時

						//if ((imagePoints_LiDAR2ZED[j].x >= yolo_buf_bbox.x && imagePoints_LiDAR2ZED[j].x <= yolo_buf_bbox.x + yolo_buf_bbox.width) && (imagePoints_LiDAR2ZED[j].y >= yolo_buf_bbox.y && imagePoints_LiDAR2ZED[j].y <= yolo_buf_bbox.y + yolo_buf_bbox.height))
						//if ((imagePoints_LiDAR2ZED[j].x >= yolo_buf_bbox.x + (1.0 / 5.0)*yolo_buf_bbox.width && imagePoints_LiDAR2ZED[j].x <= yolo_buf_bbox.x + (4.0 / 5.0)*yolo_buf_bbox.width) && (imagePoints_LiDAR2ZED[j].y >= yolo_buf_bbox.y + (1.0 / 5.0)*yolo_buf_bbox.height && imagePoints_LiDAR2ZED[j].y <= yolo_buf_bbox.y + (4.0 / 5.0)*yolo_buf_bbox.height))
						//if ((imagePoints_LiDAR2ZED[j].x >= yolo_buf_bbox.x - (1.0 / 5.0)*yolo_buf_bbox.width && imagePoints_LiDAR2ZED[j].x <= yolo_buf_bbox.x + (6.0 / 5.0)*yolo_buf_bbox.width) && (imagePoints_LiDAR2ZED[j].y >= yolo_buf_bbox.y - (1.0 / 5.0)*yolo_buf_bbox.height && imagePoints_LiDAR2ZED[j].y <= yolo_buf_bbox.y + (6.0 / 5.0)*yolo_buf_bbox.height))
						if ((imagePoints_LiDAR2ZED[j].x >= yolo_buf_bbox.x - (1.0 / 5.0)*yolo_buf_bbox.width && imagePoints_LiDAR2ZED[j].x <= yolo_buf_bbox.x + (6.0 / 5.0)*yolo_buf_bbox.width) && (imagePoints_LiDAR2ZED[j].y >= yolo_buf_bbox.y - (1.0 / 5.0)*yolo_buf_bbox.height && imagePoints_LiDAR2ZED[j].y <= yolo_buf_bbox.y + (1.0 / 2.0)*yolo_buf_bbox.height))
						{

							//YOLO内のLiDARの結果を書き込むファイルに『LiDARの結果』を書き込んでいく
							//fprintf_s(yolo_lidar_distance_result_data, "%lf\t%lf\t%lf\n", point_cloud[j].x, point_cloud[j].y, point_cloud[j].z);




							//Point cloud in the bounding box(2D)
							point_cloud_in_a_boundingbox_2D.x = imagePoints_LiDAR2ZED[j].x;
							point_cloud_in_a_boundingbox_2D.y = imagePoints_LiDAR2ZED[j].y;

							all_point_cloud_in_a_boundingbox_2D.push_back(point_cloud_in_a_boundingbox_2D);





							//Point cloud in the bounding box(3D)
							point_cloud_in_a_boundingbox_3D.x = point_cloud_LiDAR_yzx[j].x;
							point_cloud_in_a_boundingbox_3D.y = point_cloud_LiDAR_yzx[j].y;
							point_cloud_in_a_boundingbox_3D.z = point_cloud_LiDAR_yzx[j].z;


							all_point_cloud_in_a_boundingbox_3D.push_back(point_cloud_in_a_boundingbox_3D);


							//Bounding box内の全点群をgnuplotに表示するために用意
							if (container)
							{
								point_cloud_in_a_boundingbox_3D_container_for_gnuplot.x = point_cloud_in_a_boundingbox_3D.x;
								point_cloud_in_a_boundingbox_3D_container_for_gnuplot.y = point_cloud_in_a_boundingbox_3D.y;
								point_cloud_in_a_boundingbox_3D_container_for_gnuplot.z = point_cloud_in_a_boundingbox_3D.z;


								all_point_cloud_in_a_boundingbox_3D_container_for_gnuplot.push_back(point_cloud_in_a_boundingbox_3D_container_for_gnuplot);


								point_cloud_in_a_boundingbox_3D_person_for_gnuplot.x = 0;
								point_cloud_in_a_boundingbox_3D_person_for_gnuplot.y = 0;
								point_cloud_in_a_boundingbox_3D_person_for_gnuplot.z = 0;


								all_point_cloud_in_a_boundingbox_3D_person_for_gnuplot.push_back(point_cloud_in_a_boundingbox_3D_person_for_gnuplot);



							}
							else if (person)
							{
								point_cloud_in_a_boundingbox_3D_person_for_gnuplot.x = point_cloud_in_a_boundingbox_3D.x;
								point_cloud_in_a_boundingbox_3D_person_for_gnuplot.y = point_cloud_in_a_boundingbox_3D.y;
								point_cloud_in_a_boundingbox_3D_person_for_gnuplot.z = point_cloud_in_a_boundingbox_3D.z;


								all_point_cloud_in_a_boundingbox_3D_person_for_gnuplot.push_back(point_cloud_in_a_boundingbox_3D_person_for_gnuplot);


								point_cloud_in_a_boundingbox_3D_container_for_gnuplot.x = 0;
								point_cloud_in_a_boundingbox_3D_container_for_gnuplot.y = 0;
								point_cloud_in_a_boundingbox_3D_container_for_gnuplot.z = 0;


								all_point_cloud_in_a_boundingbox_3D_container_for_gnuplot.push_back(point_cloud_in_a_boundingbox_3D_container_for_gnuplot);

							}



							lidar_distance_in_boundingbox = all_lidar_distance[j];

							all_lidar_distance_in_boundingbox.push_back(lidar_distance_in_boundingbox);


							////////////////////////////////////////////////////////////////////////////

							//ファイルごとにバウンディングボックス内のLiDARの結果を保存
							//addition date is 2020_11_27 
							//per_yolo_lidar_distance_result_data << point_cloud_LiDAR_yzx[j].x << "\t" << point_cloud_LiDAR_yzx[j].y << "\t" << point_cloud_LiDAR_yzx[j].z << "\n";


							////////////////////////////////////////////////////////////////////////////








							//2020_12_1
							////////////////////////////////////////////////////////////////////////////////

							//②

							//LiDARの点群を画像に出力
							cv::circle(img_out, cv::Point((int)imagePoints_LiDAR2ZED[j].x, (int)imagePoints_LiDAR2ZED[j].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);


							/////////////////////////////////////////////////////////////////////////////////






						}


					}



					//2021_1_6追記
					//YOLO_LiDAR_distance_result_dataに改行を2行追加
					//per_yolo_lidar_distance_result_data << "\n" << "\n";





					//2021_2_2 add
					NN_method(all_point_cloud_in_a_boundingbox_3D, 1.0, NN);




					double min_nn_distance;
					double nn_distance;


					///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

					//③

					if (NN.size() != 0)
					{

						//2021_2_3 add
						NN_method_representative(NN, all_NN_representative, 1.0, 2);

						if (all_NN_representative.size() != 0)
						{
							min_nn_distance = sqrt((all_NN_representative[0].cx * all_NN_representative[0].cx) + (all_NN_representative[0].cy * all_NN_representative[0].cy) + (all_NN_representative[0].cz * all_NN_representative[0].cz));

							for (size_t i = 0; i < all_NN_representative.size(); i++)
							{
								nn_distance = sqrt((all_NN_representative[i].cx * all_NN_representative[i].cx) + (all_NN_representative[i].cy * all_NN_representative[i].cy) + (all_NN_representative[i].cz * all_NN_representative[i].cz));


								if (nn_distance < min_nn_distance)
								{
									min_nn_distance = nn_distance;
								}
							}
						}



						//printf("all_NN_representative = %d\n", all_NN_representative.size());



						//NN法の結果を書き出す（全点群 OR 上半分の点群）
						/*for (int i = 0; i < (int)NN.size(); i++)
						{
						fprintf_s(fp_w_NN, "%d\t%lf\t%lf\t%lf\t%d\t%s\n", i_yolo_lidar, NN[i].cx, NN[i].cy, NN[i].cz, NN[i].count, NN[i].yolo_name);
						}*/


						//NN法の結果を書き出す（最終）
						/*for (int i = 0; i < (int)all_NN_representative.size(); i++)
						{
							fprintf_s(fp_w_NN, "%d\t%lf\t%lf\t%lf\t%d\t%s\n", i_yolo_lidar, all_NN_representative[i].cx, all_NN_representative[i].cy, all_NN_representative[i].cz, all_NN_representative[i].count, all_NN_representative[i].yolo_name);
						}




						fprintf_s(fp_w_NN, "\n\n");*/

					}


					///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






					///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

					//④

					if (all_NN_representative.size() != 0)
					{

						for (size_t i = 0; i < all_NN_representative.size(); i++)
						{
							NN_3D_buf.x = all_NN_representative[i].cx;
							NN_3D_buf.y = all_NN_representative[i].cy;
							NN_3D_buf.z = all_NN_representative[i].cz;


							NN_3D.push_back(NN_3D_buf);
						}


						cv::projectPoints(NN_3D, rvec, tvec, K1, D1, NN_2D);

						for (size_t j = 0; j < NN_2D.size(); j++)
						{
							//画像に描写
							cv::circle(img_out, cv::Point((int)NN_2D[j].x, (int)NN_2D[j].y), 3, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
						}



						NN_3D.clear();                        NN_3D.shrink_to_fit();
						NN_2D.clear();                        NN_2D.shrink_to_fit();
					}

					///////////////////////////////////////////////////////////////////////////////////////////////////////////////////






					//write the result of nn method to the text file






					//2021_1_12追記
					//Calculate the median

					//get_mediam(histgram_textfile_path, all_lidar_distance_in_boundingbox, all_point_cloud_in_a_boundingbox_3D, all_point_cloud_in_a_boundingbox_2D, median_distance);





					///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

					//⑤

					if (all_NN_representative.size() != 0)
					{
						//小数点以下の指定(https://mstn.hateblo.jp/entry/2018/01/25/160457)(http://koshinran.hateblo.jp/entry/2018/04/04/234111)
						int digit = 1; //小数第1位まで表示
						std::ostringstream oss;
						oss << std::fixed << std::setprecision(digit) << min_nn_distance;
						std::string min_sitance_num_new = oss.str();

						
						cv::putText(img_out, min_sitance_num_new + "m", cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width + 5), int(yolo_buf_bbox.y)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
						
					}

					///////////////////////////////////////////////////////////////////////////////////////////////////////////////////





					//memory clear
					point_cloud.clear();                                  point_cloud.shrink_to_fit();
					point_cloud_LiDAR_yzx.clear();                        point_cloud_LiDAR_yzx.shrink_to_fit();
					all_lidar_distance.clear();                           all_lidar_distance.shrink_to_fit();
					imagePoints_LiDAR2ZED.clear();                        imagePoints_LiDAR2ZED.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_2D.clear();          all_point_cloud_in_a_boundingbox_2D.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_3D.clear();          all_point_cloud_in_a_boundingbox_3D.shrink_to_fit();
					all_lidar_distance_in_boundingbox.clear();            all_lidar_distance_in_boundingbox.shrink_to_fit();

					//2021_2_2 add
					NN.clear();                           NN.shrink_to_fit();
					all_NN_representative.clear();        all_NN_representative.shrink_to_fit();


					////////////////////////////////////////////////////////////////////////////


				} //else ←行の文字数が2以外なら


			} //『もし，テキストファイル内に文字があるなら』終了(if文)

			else //もし，テキストファイルの最後までいけば…
			{
				printf_s("Detected the end of file!!\n");


				//fclose(fp_yolo_bb); //YOLOのテキストファイルを閉じる


				break;
			}




			//fprintf_s(yolo_lidar_distance_result_data, "-\n"); //認識した物体ごとに改行する(バウンディングボックス内のLiDARの点群の結果を出力したファイル)



		} //1枚の画像に対するYOLOの認識結果の終わり





		  //動画の出力
		img_out_name = out_put_image_file_path + "/image" + std::to_string(i_yolo_lidar - 1) + ".png";
		cv::imwrite(img_out_name, img_out);


		//display movie of result
		/*cv::imshow("Result for each sheet", img_out);
		if (cv::waitKey(1) == 13)
		{
		break;
		}*/



		//per_yolo_lidar_distance_result_data.close();



		//もし，YOLOのテキストファイルの行の文字数が0なら…(＝テキストファイル内を全て開き終えたら…)
		if (n_yolo_bb_file_stream_size == 0)
		{

			fclose(fp_yolo_bb); //YOLOのテキストファイルを閉じる


			break; //while文から抜ける

		}





		//fprintf_s(yolo_result_textfile_path, "--\n"); //YOLOの結果に対して改行する


		//fprintf_s(yolo_lidar_distance_result_data, "\n"); //バウンディングボックス内のliDAR点群の画像1枚ごとに改行する






	} //全ての画像に対するYOLOの認識結果の終わり



	  //fclose(yolo_result_textfile_path);
	  //fclose(yolo_lidar_distance_result_data);
	  //fclose(fp_yolo_bb);



	std::cout << "\n" << std::endl;
	std::cout << "OK!!" << std::endl;






	//fclose(fp_w_NN);




	//IoUの結果をテキストファイルに書き出す
	/*for (int j = 0; j < (int)All_d_IoU_container.size(); j++)
	{
		for (int i = 0; i < (int)All_d_IoU_container[j].size(); i++)
		{
			fprintf(fp_w_IoU_conteiner, "%d\t%lf\n", j, All_d_IoU_container[j][i]);
		}

		fprintf(fp_w_IoU_conteiner, "\n\n");
	}



	for (int j = 0; j < (int)All_d_IoU_person.size(); j++)
	{
		for (int i = 0; i < (int)All_d_IoU_person[j].size(); i++)
		{
			fprintf(fp_w_IoU_person, "%d\t%lf\n", j, All_d_IoU_person[j][i]);
		}

		fprintf(fp_w_IoU_person, "\n\n");
	}*/



	/*fclose(fp_w_IoU_conteiner);
	fclose(fp_w_IoU_person);*/





	//prepare to close gnuplot 
	//_pclose(gid); 







	/*std::ofstream ofs_BBox_container;
	ofs_BBox_container.open("D:\\MyTemp\\BBox_container_ver4.txt");
	for (size_t j = 0; j < Objects_yolo_BBox_container.size(); j++)
	{
	for (size_t i = 0; i < Objects_yolo_BBox_container[j].size(); i++)
	{
	ofs_BBox_container << Objects_yolo_BBox_container[j][i].x + (1.0 / 2.0)*Objects_yolo_BBox_container[j][i].width << "\t" << Objects_yolo_BBox_container[j][i].y + (1.0 / 2.0)*Objects_yolo_BBox_container[j][i].height << endl;
	}
	ofs_BBox_container << endl;
	ofs_BBox_container << endl;
	}
	ofs_BBox_container.close();


	std::ofstream ofs_BBox_person;
	ofs_BBox_person.open("D:\\MyTemp\\BBox_person_ver4.txt");
	for (size_t j = 0; j < Objects_yolo_BBox_person.size(); j++)
	{
	for (size_t i = 0; i < Objects_yolo_BBox_person[j].size(); i++)
	{
	ofs_BBox_person << Objects_yolo_BBox_person[j][i].x + (1.0 / 2.0)*Objects_yolo_BBox_person[j][i].width << "\t" << Objects_yolo_BBox_person[j][i].y + (1.0 / 2.0)*Objects_yolo_BBox_person[j][i].height << endl;
	}
	ofs_BBox_person << endl;
	ofs_BBox_person << endl;
	}

	ofs_BBox_person.close();*/









	/*std::ofstream ofs_Objects_container;
	ofs_Objects_container.open("D:\\MyTemp\\Objects_container.txt");
	for (size_t j = 0; j < Objects_container.size(); j++)
	{
	for (size_t i = 0; i < Objects_container[j].size(); i++)
	{
	ofs_Objects_container << to_string(Objects_container[j][i].x) << "\t" << to_string(Objects_container[j][i].y) << "\t" << to_string(Objects_container[j][i].z) << endl;
	}
	ofs_Objects_container << endl;
	ofs_Objects_container << endl;
	}
	ofs_Objects_container.close();


	std::ofstream ofs_Objects_person;
	ofs_Objects_person.open("D:\\MyTemp\\Objects_person.txt");
	for (size_t j = 0; j < Objects_person.size(); j++)
	{
	for (size_t i = 0; i < Objects_person[j].size(); i++)
	{
	ofs_Objects_person << to_string(Objects_person[j][i].x) << "\t" << to_string(Objects_person[j][i].y) << "\t" << to_string(Objects_person[j][i].z) << endl;
	}
	ofs_Objects_person << endl;
	ofs_Objects_person << endl;
	}

	ofs_Objects_person.close();*/




	//write the result of trajectory to the text file. 

	/*std::ofstream ofs_container;
	ofs_container.open("D:\\MyTemp\\all_median_points_mine_container.txt");
	for (size_t i = 0; i < d_median_points_container.size(); i++) {
	ofs_container << d_median_points_container[i].x << "\t" << d_median_points_container[i].y << "\t" << d_median_points_container[i].z << "\t" << std::endl;
	}
	ofs_container.close();



	std::ofstream ofs_person;
	ofs_person.open("D:\\MyTemp\\all_median_points_mine_person.txt");
	for (size_t i = 0; i < d_median_points_person.size(); i++) {
	ofs_person << d_median_points_person[i].x << "\t" << d_median_points_person[i].y << "\t" << d_median_points_person[i].z << "\t" << std::endl;
	}
	ofs_person.close();*/








	/*OutputFile_container.close();
	OutputFile_person.close();*/




	return 0;
}





double masuda_calc_IoU(vector<vector<double>> &all_d_IoU_input1, vector<vector<double>> &all_d_IoU_input2,
	vector<struct yolo_bounding_box> &IN_yolo_BBox_buf_input1, vector<struct yolo_bounding_box> &IN_yolo_BBox_buf_input2,
	vector<vector<struct yolo_bounding_box>> &Objects_yolo_BBox_input1, vector<vector<struct yolo_bounding_box>> &Objects_yolo_BBox_input2)
{
	struct struct_rectangle bbox_prev_input1, bbox_current_input1;
	struct struct_rectangle bbox_prev_input2, bbox_current_input2;



	struct yolo_bounding_box bbox_input1_vec;
	struct yolo_bounding_box bbox_input2_vec;

	struct yolo_bounding_box in_yolo_bbox_input1;
	struct yolo_bounding_box in_yolo_bbox_input2;

	vector<struct yolo_bounding_box> targeted_yolo_BBox_input1;
	vector<struct yolo_bounding_box> targeted_yolo_BBox_input2;


	double d_IoU_input1;
	vector<double> buf_d_IoU_input1;


	double d_IoU_input2;
	vector<double> buf_d_IoU_input2;


	double IoU_max_input1;
	int n_object_IoU_max_id_input1;

	double IoU_max_input2;
	int n_object_IoU_max_id_input2;


	int n_object_tracked_bbox_count_input1;
	int n_object_tracked_bbox_count_input2;


	vector<struct yolo_bounding_box> In_yolo_BBox_input1;
	vector<struct yolo_bounding_box> In_yolo_BBox_input2;

	double IoU_thresh = 0.01;







	//container

	for (size_t num = 0; num < IN_yolo_BBox_buf_input1.size(); num++)
	{


		In_yolo_BBox_input1.clear();
		In_yolo_BBox_input1.shrink_to_fit();

		bbox_input1_vec = IN_yolo_BBox_buf_input1[num];

		In_yolo_BBox_input1.push_back(bbox_input1_vec);


		//printf("%d", (int)In_yolo_BBox_input1.size());

		for (size_t f = 0; f < In_yolo_BBox_input1.size(); f++)
		{
			in_yolo_bbox_input1.x = In_yolo_BBox_input1[f].x;
			in_yolo_bbox_input1.y = In_yolo_BBox_input1[f].y;
			in_yolo_bbox_input1.width = In_yolo_BBox_input1[f].width;
			in_yolo_bbox_input1.height = In_yolo_BBox_input1[f].height;


			if (Objects_yolo_BBox_input1.size() == 0)
			{
				targeted_yolo_BBox_input1.clear();
				targeted_yolo_BBox_input1.shrink_to_fit();

				targeted_yolo_BBox_input1.push_back(in_yolo_bbox_input1);

				Objects_yolo_BBox_input1.push_back(targeted_yolo_BBox_input1);
			}
			else
			{
				buf_d_IoU_input1.clear();    buf_d_IoU_input1.shrink_to_fit();


				IoU_max_input1 = -1.0;

				for (size_t k = 0; k < Objects_yolo_BBox_input1.size(); k++)
				{
					n_object_tracked_bbox_count_input1 = (int)Objects_yolo_BBox_input1[k].size();



					bbox_prev_input1.x0 = Objects_yolo_BBox_input1[k][n_object_tracked_bbox_count_input1 - 1].x;
					bbox_prev_input1.y0 = Objects_yolo_BBox_input1[k][n_object_tracked_bbox_count_input1 - 1].y;

					bbox_prev_input1.x1 = bbox_prev_input1.x0 + Objects_yolo_BBox_input1[k][n_object_tracked_bbox_count_input1 - 1].width;
					bbox_prev_input1.y1 = bbox_prev_input1.y0 + Objects_yolo_BBox_input1[k][n_object_tracked_bbox_count_input1 - 1].height;


					bbox_current_input1.x0 = in_yolo_bbox_input1.x;
					bbox_current_input1.y0 = in_yolo_bbox_input1.y;

					bbox_current_input1.x1 = bbox_current_input1.x0 + in_yolo_bbox_input1.width;
					bbox_current_input1.y1 = bbox_current_input1.y0 + in_yolo_bbox_input1.height;



					d_IoU_input1 = calc_IoU(&bbox_prev_input1, &bbox_current_input1);





					//2021_1_28 add Hangarian method ?










					//printf("IoU = %lf.", d_IoU_input1);

					if (d_IoU_input1 > IoU_max_input1)
					{
						IoU_max_input1 = d_IoU_input1;
						n_object_IoU_max_id_input1 = k;

						buf_d_IoU_input1.push_back(IoU_max_input1);

						if (all_d_IoU_input1.size() <= 1)
						{
							all_d_IoU_input1.push_back(buf_d_IoU_input1);
						}
					}

				}


				if (IoU_max_input1 > IoU_thresh)
				{
					Objects_yolo_BBox_input1[n_object_IoU_max_id_input1].push_back(in_yolo_bbox_input1);

					if (all_d_IoU_input1.size() >= 2)
					{
						all_d_IoU_input1[n_object_IoU_max_id_input1].push_back(IoU_max_input1);
					}
				}
				else
				{
					targeted_yolo_BBox_input1.clear();
					targeted_yolo_BBox_input1.shrink_to_fit();

					targeted_yolo_BBox_input1.push_back(in_yolo_bbox_input1);
					Objects_yolo_BBox_input1.push_back(targeted_yolo_BBox_input1);


					all_d_IoU_input1.push_back(buf_d_IoU_input1);
				}

			}


		}


	}




	//person

	for (size_t num = 0; num < IN_yolo_BBox_buf_input2.size(); num++)
	{
		In_yolo_BBox_input2.clear();
		In_yolo_BBox_input2.shrink_to_fit();

		bbox_input2_vec = IN_yolo_BBox_buf_input2[num];

		In_yolo_BBox_input2.push_back(bbox_input2_vec);



		for (size_t f = 0; f < In_yolo_BBox_input2.size(); f++)
		{
			in_yolo_bbox_input2.x = In_yolo_BBox_input2[f].x;
			in_yolo_bbox_input2.y = In_yolo_BBox_input2[f].y;
			in_yolo_bbox_input2.width = In_yolo_BBox_input2[f].width;
			in_yolo_bbox_input2.height = In_yolo_BBox_input2[f].height;


			if (Objects_yolo_BBox_input2.size() == 0)
			{
				targeted_yolo_BBox_input2.clear();
				targeted_yolo_BBox_input2.shrink_to_fit();

				targeted_yolo_BBox_input2.push_back(in_yolo_bbox_input2);

				Objects_yolo_BBox_input2.push_back(targeted_yolo_BBox_input2);
			}
			else
			{
				buf_d_IoU_input2.clear();    buf_d_IoU_input2.shrink_to_fit();


				IoU_max_input2 = -1.0;

				for (size_t k = 0; k < Objects_yolo_BBox_input2.size(); k++)
				{
					n_object_tracked_bbox_count_input2 = (int)Objects_yolo_BBox_input2[k].size();


					bbox_prev_input2.x0 = Objects_yolo_BBox_input2[k][n_object_tracked_bbox_count_input2 - 1].x;
					bbox_prev_input2.y0 = Objects_yolo_BBox_input2[k][n_object_tracked_bbox_count_input2 - 1].y;

					bbox_prev_input2.x1 = bbox_prev_input2.x0 + Objects_yolo_BBox_input2[k][n_object_tracked_bbox_count_input2 - 1].width;
					bbox_prev_input2.y1 = bbox_prev_input2.y0 + Objects_yolo_BBox_input2[k][n_object_tracked_bbox_count_input2 - 1].height;


					bbox_current_input2.x0 = in_yolo_bbox_input2.x;
					bbox_current_input2.y0 = in_yolo_bbox_input2.y;

					bbox_current_input2.x1 = bbox_current_input2.x0 + in_yolo_bbox_input2.width;
					bbox_current_input2.y1 = bbox_current_input2.y0 + in_yolo_bbox_input2.height;



					d_IoU_input2 = calc_IoU(&bbox_prev_input2, &bbox_current_input2);





					//2021_1_28 add Hangarian method ?










					if (d_IoU_input2 > IoU_max_input2)
					{
						IoU_max_input2 = d_IoU_input2;
						n_object_IoU_max_id_input2 = k;


						buf_d_IoU_input2.push_back(IoU_max_input2);

						if (all_d_IoU_input2.size() <= 1)
						{
							all_d_IoU_input2.push_back(buf_d_IoU_input2);
						}
					}

				}



				if (IoU_max_input2 > IoU_thresh)
				{
					Objects_yolo_BBox_input2[n_object_IoU_max_id_input2].push_back(in_yolo_bbox_input2);

					if (all_d_IoU_input2.size() >= 2)
					{
						all_d_IoU_input2[n_object_IoU_max_id_input2].push_back(IoU_max_input2);
					}
				}
				else
				{
					targeted_yolo_BBox_input2.clear();
					targeted_yolo_BBox_input2.shrink_to_fit();

					targeted_yolo_BBox_input2.push_back(in_yolo_bbox_input2);
					Objects_yolo_BBox_input2.push_back(targeted_yolo_BBox_input2);


					all_d_IoU_input2.push_back(buf_d_IoU_input2);
				}

			}


		}


	}



	return 0;

}


double calc_IoU(struct struct_rectangle *a, struct struct_rectangle *b)
{

	double d_cross_xmin, d_cross_xmax;
	double d_cross_ymin, d_cross_ymax;

	double d_cross_width;
	double d_cross_height;

	double d_OR_area;
	double d_CROSS_area;
	double d_IoU;

	// min(a.x1, b.x1)
	if (a->x1 < b->x1) d_cross_xmax = a->x1;
	else d_cross_xmax = b->x1;

	// max(a.x0, b.x0)
	if (a->x0 > b->x0) d_cross_xmin = a->x0;
	else d_cross_xmin = b->x0;


	d_cross_width = d_cross_xmax - d_cross_xmin;


	// min(a.y1, b.y1)
	if (a->y1 < b->y1) d_cross_ymax = a->y1;
	else d_cross_ymax = b->y1;

	// max(a.y0, b.y0)
	if (a->y0 > b->y0) d_cross_ymin = a->y0;
	else d_cross_ymin = b->y0;


	d_cross_height = d_cross_ymax - d_cross_ymin;


	if (d_cross_width >= 0 && d_cross_height >= 0) {

		d_CROSS_area = d_cross_width*d_cross_height;

		d_OR_area = (a->x1 - a->x0)*(a->y1 - a->y0) + (b->x1 - b->x0)*(b->y1 - b->y0) - d_CROSS_area;



		d_IoU = d_CROSS_area / d_OR_area;

		//printf("cross:%lf OR_area:%lf\n", d_CROSS_area, d_OR_area);

		return d_IoU;

	}
	else return 0;


}




int NN_method(vector<struct point_cloud_after_conversion> &input_data_xyz_3D, double Thresh, vector<struct NN_type> &nn)
{
	struct NN_type buf_nn;
	double min;
	double distance;
	int min_number;

	//double Thresh = 1.0;

	int counter;

	for (size_t k = 0; k < input_data_xyz_3D.size(); k++)
	{


		//printf("input_data_xyz_3D = %f\t%f\t%f\n", input_data_xyz_3D[k].x, input_data_xyz_3D[k].y, input_data_xyz_3D[k].z);
	}


	counter = 0;

	for (size_t k = 0; k < input_data_xyz_3D.size(); k++)
	{

		if (k == 0)
		{
			buf_nn.cx = input_data_xyz_3D[k].x;
			buf_nn.cy = input_data_xyz_3D[k].y;
			buf_nn.cz = input_data_xyz_3D[k].z;
			buf_nn.sumx = input_data_xyz_3D[k].x;
			buf_nn.sumy = input_data_xyz_3D[k].y;
			buf_nn.sumz = input_data_xyz_3D[k].z;

			if (container)
			{
				buf_nn.yolo_name = "container";
			}
			else if (person)
			{
				buf_nn.yolo_name = "person";
			}


			buf_nn.count = 1;

			input_data_xyz_3D[k].type = counter;

			nn.push_back(buf_nn);

			counter++;

			continue;
		}

		min = 100000;

		for (size_t i = 0; i < nn.size(); i++)
		{

			distance =
				(input_data_xyz_3D[k].x - nn[i].cx) * (input_data_xyz_3D[k].x - nn[i].cx) +
				(input_data_xyz_3D[k].y - nn[i].cy) * (input_data_xyz_3D[k].y - nn[i].cy) +
				(input_data_xyz_3D[k].z - nn[i].cz) * (input_data_xyz_3D[k].z - nn[i].cz);

			distance = sqrt(distance);

			//printf("distance = %lf\n", distance);

			if (min > distance)
			{
				min = distance;
				min_number = i;
			}
		}


		//printf("min_distance = %lf\n", min);

		if (min > Thresh)
		{
			buf_nn.cx = input_data_xyz_3D[k].x;
			buf_nn.cy = input_data_xyz_3D[k].y;
			buf_nn.cz = input_data_xyz_3D[k].z;
			buf_nn.sumx = input_data_xyz_3D[k].x;
			buf_nn.sumy = input_data_xyz_3D[k].y;
			buf_nn.sumz = input_data_xyz_3D[k].z;

			if (container)
			{
				buf_nn.yolo_name = "container";
			}
			else if (person)
			{
				buf_nn.yolo_name = "person";
			}



			buf_nn.count = 1;


			nn.push_back(buf_nn);

			input_data_xyz_3D[k].type = counter;

			counter++;

		}
		else
		{
			nn[min_number].sumx += input_data_xyz_3D[k].x;
			nn[min_number].sumy += input_data_xyz_3D[k].y;
			nn[min_number].sumz += input_data_xyz_3D[k].z;

			nn[min_number].count++;


			nn[min_number].cx = nn[min_number].sumx / nn[min_number].count;
			nn[min_number].cy = nn[min_number].sumy / nn[min_number].count;
			nn[min_number].cz = nn[min_number].sumz / nn[min_number].count;

		}

	}

	return 0;
}




int NN_method_representative(vector<struct NN_type> nn, vector<struct NN_type> &all_nn_representative, double object_range, int Thresh_clustering_num)
{

	vector<double> distance(nn.size());
	struct NN_type nn_representative;

	NN_type min_nn;

	int min_distance_number;

	double min_distance;





	//initialization
	for (size_t i = 0; i < nn.size(); i++)
	{
		if (nn[i].count >= Thresh_clustering_num)
		{
			min_distance = sqrt((nn[i].cx * nn[i].cx) + (nn[i].cy * nn[i].cy) + (nn[i].cz * nn[i].cz));
			min_distance_number = i;
		}
	}



	//calculate the minimun clustering
	for (size_t i = 0; i < nn.size(); i++)
	{

		if (nn[i].count >= Thresh_clustering_num)
		{
			distance[i] = sqrt((nn[i].cx * nn[i].cx) + (nn[i].cy * nn[i].cy) + (nn[i].cz * nn[i].cz));

			if (distance[i] < min_distance)
			{
				min_distance = distance[i];
				min_distance_number = i;
			}
		}
	}


	//printf("min_distance = %lf\n", min_distance);

	for (size_t i = 0; i < nn.size(); i++)
	{
		if (nn[i].count >= Thresh_clustering_num)
		{
			//printf("d = %lf\n", distance[i]);

			if (distance[i] < (min_distance + object_range / 2.0)) //←最小の距離（min_distance）からどの範囲までの点群を取り出すか？
			{
				nn_representative.cx = nn[i].cx;
				nn_representative.cy = nn[i].cy;
				nn_representative.cz = nn[i].cz;
				nn_representative.count = nn[i].count;

				if (container)
				{
					nn_representative.yolo_name = "container";
				}
				else if (person)
				{
					nn_representative.yolo_name = "person";
				}


				all_nn_representative.push_back(nn_representative);
			}
		}
	}


	return 0;
}




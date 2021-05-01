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



char *transporting;
char *container;
char *person;


int n_class = 0;


int nCommand;


int i_yolo_lidar = 0;












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



//2021_2_2 add
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
void menu_screen();
int OpencvVideocaputure(int argc, const char* argv[]);
int Labeling(int argc, const char* argv[]);
int ImageToMovie(int n);
int LidarImageToMovie(int n);
int Movie_to_image();
int ImageDownloadFile();
int textfile_to_another_folder();
int remove_image_text();
int yolo_lidar();
int get_mediam(std::string outputfile_path, std::vector<double> input_data_distance, std::vector<Point3f> input_data_xyz_3D, std::vector<Point2f> input_data_xy_2D, double &median_num);
int median(vector<double> input_data_distance_3D, vector<Point3f> input_data_xyz_3D, double &median_num);
int vector_test_sample(FILE *gid);
int show_objects_pose_via_gnuplot(FILE *gid);

int gnuplot_mapping();
int gnuplot_histgram();
int gnuplot_sort_result();


double calc_IoU(struct struct_rectangle *a, struct struct_rectangle *b);
double masuda_calc_IoU();

int calculate_Representative_LiDAR_point(vector<Point2f> input_data_xy_2D, std::vector<Point3f>input_data_xyz_3D,
	vector<double> &all_distance_from_min, vector<Point3f>&all_distance_from_min_3d, vector<Point2f> &all_distance_from_min_2d);


//nn method
int NN_method(vector<struct point_cloud_after_conversion> &input_data_xyz_3D, double Thresh, vector<struct NN_type> &nn);




struct point_cloud_after_conversion in_obj_points_container;
std::vector<struct point_cloud_after_conversion> d_median_points_container;
struct point_cloud_after_conversion in_obj_points_person;
std::vector<struct point_cloud_after_conversion> d_median_points_person;





//YOLO Variables

//yolo bounding box parameters
//vector<struct yolo_bounding_box> yolo_BBox; //1枚の画像結果ごとに追加していくので，配列型（vector）にしておく
struct yolo_bounding_box yolo_buf_bbox;

vector<struct yolo_bounding_box> IN_yolo_BBox_buf_container;
vector<struct yolo_bounding_box> IN_yolo_BBox_buf_person;

vector<struct yolo_bounding_box> In_yolo_BBox_container;
vector<struct yolo_bounding_box> In_yolo_BBox_person;

vector<vector<struct yolo_bounding_box>> Objects_yolo_BBox_container;
vector<vector<struct yolo_bounding_box>> Objects_yolo_BBox_person;







//LiDAR Variables
vector<struct point_cloud>point_cloud; //push_backするために用意
struct point_cloud point; //LiDARの出力結果(x,y,z,intensity)



std::vector<Point3f> point_cloud_LiDAR_yzx;
Point3f buf_LiDAR_yzx;

std::vector<double>all_lidar_distance;
double lidar_distance;

//LiDAR 3D → 2D//
vector<Point2f> imagePoints_LiDAR2ZED;

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



double median_distance;




//////////////////////////////////////

//中央値を含めて，上下何点抽出するか？
//const int NumFromMedian = 7;

//////////////////////////////////////

vector<double>all_median_distance_container;
vector<double>all_median_distance_person;

vector<struct point_cloud_after_conversion>all_median_num_container;
vector<struct point_cloud_after_conversion>all_median_num_person;




///////////////////////////////////////////////////////////////

//2021_1_21 addition
//define only median value
vector<struct tracking_object_type> IN_Objects_buf_container;
vector<struct tracking_object_type> IN_Objects_container;

struct tracking_object_type in_object_container;

vector<struct tracking_object_type> targeted_object_container;

vector<vector<struct tracking_object_type>> Objects_container;



vector<struct tracking_object_type> IN_Objects_buf_person;
vector<struct tracking_object_type> IN_Objects_person;

struct tracking_object_type in_object_person;

vector<struct tracking_object_type> targeted_object_person;

vector<vector<struct tracking_object_type>> Objects_person;

///////////////////////////////////////////////////////////////






//2021_2_1 add for calculating the representative LiDAR point cloud
vector<double> all_distance_from_min;
vector<Point2f>all_distance_from_min_2D;
vector<struct point_cloud_after_conversion>all_distance_from_min_3D;


//2021_2_2 add
//NN method
vector<struct NN_type> NN;















//argcは引数の個数、argvには引数の実際の値が入る
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
			//動画の入出力
		case 1:

			OpencvVideocaputure(argc, argv);

			break;

		case 2:

			Labeling(argc, argv);

			break;


		case 3:

			ImageToMovie(n);

			break;

		case 4:

			LidarImageToMovie(n);

			break;


		case 5:

			Movie_to_image();

			break;

		case 6:


			ImageDownloadFile();


			break;

		case 7:

			textfile_to_another_folder();

			break;


			/*
			case 8:
			remove_image_text();
			break;
			*/


		case 9:

			yolo_lidar();

			break;


		case 10:


			//gnuplot_histgram();

			gnuplot_sort_result();

			//histgram_Lidar_YOLO();

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
	printf("<<1>>:動画の入出力\n");
	printf("<<2>>:画像のラベリング\n");
	printf("<<3>>:元画像ファイルから動画を生成\n");
	printf("<<4>>:元画像(半分部分)以外の画像から動画を生成\n");
	printf("<<5>>:動画から画像を作成\n");
	printf("<<6>>:別のフォルダーに画像を保存\n");
	printf("<<7>>:テキストファイルを別のフォルダーへコピーします．\n");
	//printf("<<8>>:LiDARの点群の写像画像の除去とテキストファイルの除去\n");
	printf("<<9>>:YOLOのバウンディングボックス内にあるLiDAR点群の距離を求める\n");
	printf("<<10>>:バウンディングボックス内のLiDAR点群をヒストグラムで表示する\n");
	printf("<<11>>:〇〇\n");
	printf("<<0>>:終了します．\n");
	printf("----------------------------------------------------\n");
	printf("\n");

	printf("command=");
	scanf_s("%d", &nCommand);


}



//動画の入出力
int OpencvVideocaputure(int argc, const char* argv[]) {

	Mat img;
	Mat gray_img;
	Mat faussian_img;
	Mat bin_img;
	Mat canny_img;


	//動画ファイルを指定
	std::cout << "FileName=" << std::flush;


	//文字型のファイルを用意
	std::string VideoFile = argv[argc - 1];



	//コマンドプロンプからファイル名を入力
	std::cin >> VideoFile;



	//https://cvtech.cc/opencv/2/
	//動画読み込み用のオブジェクトを生成
	VideoCapture cap(VideoFile);


	//http://shibafu3.hatenablog.com/entry/2016/11/13/151118
	//動画書き込み用のオブジェクトを生成
	VideoWriter writer("D:\\latter_graduation_research\\recording.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 100, Size(672, 376));




	// cv::VideoCaptureが正常に動作しているかをチェック！
	if (!cap.isOpened())
	{

		std::cerr << "cannot open this movie file!" << std::endl;

		return -1;
	}

	// cv::VideoWriterが正常に動作しているかをチェック！
	if (!writer.isOpened())
	{

		std::cerr << "failed to record" << std::endl;

		return -1;
	}



	double max_frame = cap.get(CAP_PROP_FRAME_COUNT);//フレーム数　←（castした方がいい）（castとは：https://www.sejuku.net/blog/25737）


	for (int i = 0; i < int(max_frame); i++)
	{


		cap >> img;//1フレーム分取り出してimgに保存させる



				   //グレイスケール化
		cvtColor(img, gray_img, CV_BGR2GRAY);



		//平滑化
		GaussianBlur(gray_img, faussian_img, Size(7, 7), 0.0);




		//2値化
		threshold(faussian_img, bin_img, 50, 255, THRESH_BINARY);



		//輪郭抽出
		Canny(img, canny_img, 60.0, 150.0);




		imshow("Video1", img);

		imshow("Video2", bin_img);

		imshow("Video3", canny_img);



		//録画(動画ファイルに画像を出力)
		writer << bin_img;


		//もし，Enterキーが押されたら強制終了
		if (waitKey(1) == 13)
		{
			break;
		}

	}



	cap.release();//読み込み用のオブジェクトを解放
	writer.release();//書き込み用のオブジェクトを解放
	destroyAllWindows();//ウィンドウを破棄


	return 0;
}





//ラベリング
//http://imgprolab.sys.fit.ac.jp/~yama/imgproc/proc/Document_OpenCVforC_5_2019.pdf
int Labeling(int argc, const char* argv[]) {

	Mat img;
	Mat gray_img;
	Mat bin_img;
	Mat label_img;


	//画像ファイルを指定
	std::cout << "FileName=" << std::flush;



	//文字型のファイルを用意
	std::string ImageFile = argv[argc - 1];



	//コマンドプロンプからファイル名を入力
	std::cin >> ImageFile;



	img = imread(ImageFile);




	imshow("Original Image", img);



	//グレイスケール化
	cvtColor(img, gray_img, COLOR_BGR2GRAY);



	//2値化
	threshold(gray_img, bin_img, 50, 255, THRESH_BINARY);



	//2値化した画像に2回Dilate処理(膨張処理)
	dilate(bin_img, bin_img, noArray(), Point(-1, -1), 2);//Q.なぜ，noArray()が使われているのか？ & Point(-1, -1)は何を表しているのか？


	imshow("Binary-Dilate Image", bin_img);




	//ラベリング(8近傍)　出力：ラベリングされた図形成分の個数
	int nLabels = connectedComponents(bin_img, label_img, 8, CV_32S);





	//ラベリング結果の描画色を決定する

	std::vector<Vec3b>colors(nLabels);//Vec3b・・・0から255の数値を格納できる箱を3つ用意する変数



	colors[0] = Vec3b(0, 0, 0);



	for (int label = 1; label < nLabels; label++)
	{


		//ラベル番号に対して，ランダムに色を割り当てる(&255・・・0から255)
		colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));

	}





	//ラベリング結果の描写(CV_8UC3・・・unsigned char [3])
	Mat dst(img.size(), CV_8UC3);


	for (int y = 0; y < dst.rows; y++)
	{
		for (int x = 0; x < dst.cols; x++)
		{
			//ラベリング画像の(x,y)上のラベル番号を抽出
			int label = label_img.at<int>(y, x);


			//ラベル番号に割り当てられた色（画素値）を結果画像の(x,y)に格納する　（※参照(reference)）
			cv::Vec3b &pixel = dst.at<cv::Vec3b>(y, x);


			pixel = colors[label];


		}

	}


	imshow("Labeling Image", dst);



	waitKey();


	return 0;
}






//元画像ファイルから動画を生成(https://kisqragi.hatenablog.com/entry/2019/11/02/142240)
int ImageToMovie(int n) {

	int start, goal;


	//読み込む画像のファイルパスを指定
	std::cout << "Path name of the image file to be loaded = " << std::endl;


	std::string image_file_path;
	std::cin >> image_file_path;



	//ファイルの始めと終わりを指定
	std::cout << "File Start Number = " << std::endl;
	std::cin >> start;

	std::cout << "File Goal Number = " << std::endl;
	std::cin >> goal;




	//保存先のパス名を指定
	std::cout << "Destination path name = " << std::endl;

	std::string destination_file_path;
	std::cin >> destination_file_path;




	std::cout << "Number of frames=" << std::endl;
	std::cin >> n;





	//テキストファイルにフレーム数を書き込む(https://qiita.com/fantm21/items/8489b944698f9d3818ea)
	std::ofstream outputfile(destination_file_path + "\\frame_count.txt");



	if ((!outputfile.is_open()))
	{
		std::cerr << "Textfile does not exist!!" << std::endl;

		return -1;

	}


	outputfile << n;



	outputfile.close();



	std::cout << "Text file is OK!! " << std::endl;








	VideoWriter writer_left(destination_file_path + "\\movie_stereo_cam1.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), n, Size(672, 376));
	VideoWriter writer_right(destination_file_path + "\\movie_stereo_cam2.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), n, Size(672, 376));



	if ((!writer_left.isOpened()) || (!writer_right.isOpened()))
	{
		std::cerr << "This File can not be opend!" << std::endl;

		return -1;
	}




	for (int i = start; i <= goal; i++)
	{


		Mat img = imread(image_file_path + "\\image" + std::to_string(i) + ".png");




		//ステレオ画像を右と左で分割
		Mat img_cam_left = img(cv::Rect(0, 0, img.cols / 2, img.rows));
		Mat img_cam_right = img(cv::Rect(img.cols / 2, 0, img.cols / 2, img.rows));





		if ((img_cam_left.empty()) || (img_cam_right.empty()))
		{
			std::cerr << "Error!!" << std::endl;

			return -1;
		}


		writer_left << img_cam_left;
		writer_right << img_cam_right;



		std::cout << i << std::endl;
	}

	std::cout << "success\n" << std::endl;



	writer_left.release();
	writer_right.release();




	return 0;

}





//LiDARの点群を写像した画像から動画を生成
int LidarImageToMovie(int n) {

	int start, goal;


	//読み込む画像のファイルパスを指定
	std::cout << "Path name of the image file to be loaded = " << std::endl;


	std::string image_file_path;
	std::cin >> image_file_path;



	//ファイルの始めと終わりを指定
	std::cout << "File Start Number = " << std::endl;
	std::cin >> start;

	std::cout << "File Goal Number = " << std::endl;
	std::cin >> goal;


	//画像パスの番号より後ろの文字を標準入力
	std::cout << "Characters after numbers" << std::endl;
	std::string characters_after_numbers;
	std::cin >> characters_after_numbers;



	//保存先のパス名を指定
	std::cout << "Destination path name = " << std::endl;

	std::string destination_file_path;
	std::cin >> destination_file_path;



	std::cout << "Number of frames=" << std::endl;
	std::cin >> n;







	//テキストファイルにフレーム数を書き込む(https://qiita.com/fantm21/items/8489b944698f9d3818ea)
	std::ofstream outputfile(destination_file_path + "\\frame_count_lidar.txt");



	if (!outputfile.is_open())
	{
		std::cerr << "Textfile does not exist!!" << std::endl;

		return -1;

	}


	outputfile << n;

	outputfile.close();

	std::cout << "Text file is OK!!" << std::endl;







	VideoWriter writer(destination_file_path + "\\camera_lidar_calibration.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), n, Size(672, 376));


	//画像用
	//Size(672, 376)

	//gnuplot用
	//Size(640, 480)



	if (!writer.isOpened())
	{
		std::cerr << "This File can not be opend!" << std::endl;

		return -1;
	}




	for (int i = start; i <= goal; i++)
	{

		Mat img = imread(image_file_path + "\\image" + std::to_string(i) + characters_after_numbers);



		//もし，LiDARの点群結果の画像が無いなら．．．(途中の番号からでも使えるように用意しておく)
		if (img.empty())
		{
			//次の画像に行く
			continue;

		}




		writer << img;


		std::cout << i << std::endl;
	}


	std::cout << "success\n" << std::endl;


	writer.release();




	return 0;
}







//動画から画像を作成
int Movie_to_image()
{
	Mat img;


	std::wcout << "Movie Path =" << std::endl;
	std::string movie_file_path;
	std::cin >> movie_file_path;

	//動画をキャプチャ
	VideoCapture cap(movie_file_path); //Windowsの場合　パス中の¥は重ねて¥¥とする
									   //VideoCapture cap("videos/sample.mp4"); //Macの場合

									   //動画の最大フレーム数
	int max_frame = (int)cap.get(CAP_PROP_FRAME_COUNT);
	// 動画の高さ
	int img_h = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
	// 動画の幅
	int img_w = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
	// 動画のfps
	double fps = cap.get(CAP_PROP_FPS);


	//保存用ファイルパスの指定
	std::cout << "File path for storage=" << std::endl;
	std::string file_path_for_storage;
	std::cin >> file_path_for_storage;


	for (int i = 0; i < max_frame; i++) {

		//1フレーム分取り出してimgに保持させる
		cap >> img;

		//現在のフレーム番号を表示
		std::cout << "フレーム番号 " << i << std::endl;


		//取得した画像を連番画像で保存

		std::string image_name;
		image_name = file_path_for_storage + "/image" + std::to_string(i) + ".png";
		cv::imwrite(image_name, img);

	}
	//動画情報の出力
	std::cout << "フレーム番号 " << max_frame - 1 << std::endl;
	std::cout << "動画の高さ " << img_h << std::endl;
	std::cout << "動画の幅 " << img_w << std::endl;
	std::cout << "動画のfps " << fps << std::endl;



	return 0;

}






//フォルダー内の画像を取り出して，他のフォルダーに保存する
int ImageDownloadFile() {

	int start, goal;


	//読み込む画像のファイルパスを指定
	std::cout << "Path name of the image file to be loaded = " << std::endl;


	std::string image_file_path;
	std::cin >> image_file_path;



	//ファイルの始めと終わりを指定
	std::cout << "File Start Number = " << std::endl;
	std::cin >> start;

	std::cout << "File Goal Number = " << std::endl;
	std::cin >> goal;




	//保存先のパス名を指定
	std::cout << "Destination path name = " << std::endl;

	std::string destination_file_path;
	std::cin >> destination_file_path;



	//保存する時の画像の開始番号を指定（例：image50.jpg←50の部分を自由に決める．）
	std::cout << "Starting number when saving the image = " << std::endl;
	int j;
	std::cin >> j;






	for (int i = start; i <= goal; i++)
	{

		//読み込む画像
		Mat img = imread(image_file_path + "\\image" + std::to_string(i) + ".png");




		//ステレオ画像を右と左で分割
		Mat img_cam1 = img(cv::Rect(0, 0, img.cols / 2, img.rows));
		Mat img_cam2 = img(cv::Rect(img.cols / 2, 0, img.cols / 2, img.rows));



		if (img.empty())
		{
			std::cerr << "Error!!" << std::endl;

			return -1;
		}


		//別のフォルダーに保存する

		std::string img_name;
		img_name = destination_file_path + "\\image" + std::to_string(j + i) + ".jpg";

		cv::imwrite(img_name, img_cam1);

		std::cout << img_name << std::endl;
	}

	std::cout << "OK!" << std::endl;


	return 0;

}




//テキストファイルを別のフォルダーにコピー(https://programming-place.net/ppp/contents/c/rev_res/file001.html)
int textfile_to_another_folder()
{

	//読み込むテキストファイルが保存されているフォルダーを標準入力
	std::cout << "input_folder = " << std::endl;
	std::string input_foder_path;
	std::cin >> input_foder_path;


	//書き込むテキストファイルが保存されているフォルダーを標準入力
	std::cout << "output_folder = " << std::endl;
	std::string output_foder_path;
	std::cin >> output_foder_path;


	//テキストファイル数を標準入力
	std::cout << "Number of all text files = " << std::endl;
	int n;
	std::cin >> n;



	for (int i = 0; i <= n; i++)
	{


		//input text file path
		std::string input_filename;
		input_filename = input_foder_path + "/point_cloud" + std::to_string(i) + ".txt_lidar_points_on_image.txt";


		//output text file path
		std::string output_filename;
		output_filename = output_foder_path + "/point_cloud" + std::to_string(i) + ".txt_lidar_points_on_image.txt";


		//copy text file 
		if (CopyFileA(input_filename.c_str(), output_filename.c_str(), FALSE) != 0)
		{
			printf("Success\n");
		}
		else
		{
			printf("Error\n");
		}


	}

	printf("Finish!");


	return 0;

}




//LiDARの点群の写像画像の除去とテキストファイルの除去
int remove_image_text() {

	int start, goal;

	//フォルダー名の指定
	std::cout << "Filepath name = " << std::endl;

	std::string filepath;
	std::cin >> filepath;


	//ファイルの始めと終わりを指定
	std::cout << "File Start Number = " << std::endl;
	std::cin >> start;

	std::cout << "File Goal Number = " << std::endl;
	std::cin >> goal;



	for (int i = start; i <= goal; i++)
	{

		std::string img_filename = filepath + "\\image" + std::to_string(i) + ".png_PnP_projected_points.png";
		std::string text_filename = filepath + "\\image" + std::to_string(i) + ".png_ZED_calculated_stereo_point_cloud.txt";


		//次のファイル
		std::string next_img_filename = filepath + "\\image" + std::to_string(i + 1) + ".png_PnP_projected_points.png";
		std::string next_text_filename = filepath + "\\image" + std::to_string(i + 1) + ".png_ZED_calculated_stereo_point_cloud.txt";



		//もし，i番目のファイルがないなら，スルーする．
		if ((img_filename.empty()) || (text_filename.empty()))
		{
			continue;
		}



		//エラー処理(正常(=0)でない時)
		//画像の除去
		//テキストファイルの除去
		if (remove(img_filename.c_str()) != 0 && remove(text_filename.c_str()) != 0)
		{
			std::cout << "Could not remove file No" + std::to_string(i) << std::endl;
		}
		else
		{

			std::cout << "Remove image" + std::to_string(i) + ".png_PnP_projected_points.png" << std::endl;

			std::cout << "Remove image" + std::to_string(i) + ".png_ZED_calculated_stereo_point_cloud.txt" << std::endl;

		}






		//ファイルが終了した時の処理
		if (next_img_filename.empty() || next_text_filename.empty())
		{
			std::cerr << "Finish!!" << std::endl;

			return -1;

		}


	}

}











//YOLOのバウンディングボックス内にあるLiDAR点群の距離を求める(自作)     int yolo_lidar()



int yolo_lidar() {

	/////////////////////////////////////////////////////////////////////////

	//2021_1_26 addition
	/*std::ofstream OutputFile_container;
	OutputFile_container.open("D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\median_value_textfile_for_Mr_Morio\\median_container.txt");

	std::ofstream OutputFile_person;
	OutputFile_person.open("D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\median_value_textfile_for_Mr_Morio\\median_person.txt");*/

	/////////////////////////////////////////////////////////////////////////




	FILE *gid;
	gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");




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








	//YOLOの認識結果ファイルの読み込み
	std::cout << "File path : Yolo Bounding Box Info = " << std::endl;
	string sFilePath_YOLO_BoundingBox;
	cin >> sFilePath_YOLO_BoundingBox;


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






	//LiDARの点群データのテキストファイルのパスを標準入力するために用意
	std::cout << "File path : LiDAR Point Cloud Info = " << std::endl;
	string sFilePath_LiDAR_PointCloud;
	cin >> sFilePath_LiDAR_PointCloud;








	int n_yolo_bb_file_stream_size; //各行の文字の数








	std::cout << "Files for storing text file（per_sheet...） = " << std::endl;

	std::string per_yolo_lidar_distance_result_data_path;
	std::cin >> per_yolo_lidar_distance_result_data_path;

	////////////////////////////////////////////////////////////////////////////



	//画像を入力するためにファイルパスを指定
	std::cout << "input images = " << std::endl;
	std::string in_put_image_file_path;
	std::cin >> in_put_image_file_path;

	//画像を出力するためにファイルパスを指定
	std::cout << "Files for storing images = " << std::endl;
	std::string out_put_image_file_path;
	std::cin >> out_put_image_file_path;



	//ヒストグラムの結果（テキストファイル）
	//std::cout << "Folder for storing the text file of the histogram results = " << std::endl;
	//std::string histgram_textfile_path;
	//std::cin >> histgram_textfile_path;



	//NN法の結果をテキストファイルに保存するために用意
	std::cout << "Folder name for saving the results of the NN method = " << std::endl;
	std::string NN_textfile;
	std::cin >> NN_textfile;







	Mat img_in;
	Mat img_in_left;
	Mat img_out;
	std::string img_out_name;



	string sFilePath_NN;
	FILE *fp_w_NN;
	errno_t err;

	sFilePath_NN = NN_textfile + "/" + "output_NN.txt";

	err = fopen_s(&fp_w_NN, sFilePath_NN.c_str(), "wt");

	if (err != 0) {

		std:cout << "cannot open the NN file." << endl;
	}




	//全体の画像に対しての処理
	while (1)
	{
		//fprintf_s(yolo_lidar_distance_result_data, "%d\n", i); //何枚目の画像なのかが分かるように，出力用のテキストファイルに枚数を書いておく
		//fprintf_s(yolo_result_textfile_path, "%d\n", i); //YOLOの結果のテキストファイルに番号を加えておく




		////////////////////////////////////////////////////////////////////////////

		//addition date is 2020_11_27 
		//1つのLiDARのデータごとにテキストファイルに保存するためにファイルを用意
		std::string per_yolo_lidar_distance_result_data_path_name;
		per_yolo_lidar_distance_result_data_path_name = per_yolo_lidar_distance_result_data_path + "/YOLO_LiDAR_distance_result_data" + std::to_string(i_yolo_lidar) + ".txt";


		//ファイルを開く
		std::ofstream per_yolo_lidar_distance_result_data;
		per_yolo_lidar_distance_result_data.open(per_yolo_lidar_distance_result_data_path_name);

		if (!per_yolo_lidar_distance_result_data.is_open())
		{
			std::cerr << "Can not open " + per_yolo_lidar_distance_result_data_path_name << std::endl;
			return -1;
		}


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
					masuda_calc_IoU();


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
					In_yolo_BBox_container.clear();        In_yolo_BBox_container.shrink_to_fit();
					In_yolo_BBox_person.clear();           In_yolo_BBox_person.shrink_to_fit();



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


					all_median_num_container.clear();                     all_median_num_container.shrink_to_fit();
					all_median_num_person.clear();                        all_median_num_person.shrink_to_fit();

					all_median_distance_container.clear();                all_median_distance_container.shrink_to_fit();
					all_median_distance_person.clear();                   all_median_distance_person.shrink_to_fit();



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


					transporting = strstr(yolo_bb_file_BUFFER, "transporting");
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


					//drawing the result of IoU
					for (size_t a = 0; a < Objects_yolo_BBox_container.size(); a++)
					{
				
						for (size_t b = 1; b < Objects_yolo_BBox_container[a].size(); b++)
						{
							
							int color_b = 255 / (a + 1);
							int color_g = 255 / (2 * a + 1);
							int color_r = (a + 1) * 10;

							cv::line(img_out, cv::Point(Objects_yolo_BBox_container[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].width, Objects_yolo_BBox_container[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].height), cv::Point(Objects_yolo_BBox_container[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].width, Objects_yolo_BBox_container[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].height), cv::Scalar(color_b, color_g, color_r), 1, cv::LINE_AA);





							//cv::line(img_out, cv::Point(Objects_yolo_BBox_container[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].width, Objects_yolo_BBox_container[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b - 1].height), cv::Point(Objects_yolo_BBox_container[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].width, Objects_yolo_BBox_container[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_container[a][b].height), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);


							//155, 114, 155
						}
					}


					for (size_t a = 0; a < Objects_yolo_BBox_person.size(); a++)
					{
						for (size_t b = 1; b < Objects_yolo_BBox_person[a].size(); b++)
						{


							int color_b = 255 / (a + 1);
							int color_g = 255 / (a + 1);
							int color_r = 255 / (2 * a + 1);
							cv::line(img_out, cv::Point(Objects_yolo_BBox_person[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].width, Objects_yolo_BBox_person[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].height), cv::Point(Objects_yolo_BBox_person[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].width, Objects_yolo_BBox_person[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].height), cv::Scalar(color_b, color_g, color_r), 1, cv::LINE_AA);





							//cv::line(img_out, cv::Point(Objects_yolo_BBox_person[a][b - 1].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].width, Objects_yolo_BBox_person[a][b - 1].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b - 1].height), cv::Point(Objects_yolo_BBox_person[a][b].x + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].width, Objects_yolo_BBox_person[a][b].y + (1.0 / 2.0)*Objects_yolo_BBox_person[a][b].height), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);


							//0, 217, 255
						}
					}









					if (transporting)
					{
						//cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(255, 0, 0), 3);

						//cv::putText(img_out, "transporting", cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

					}
					else
					{
						cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(0, 0, 255), 1);



						//2021_1_27 add
						//拡大した場合に使う
						//cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y - (1.0 / 5.0)*yolo_buf_bbox.height)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width + (1.0 / 5.0)*yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(0, 0, 0), 2);


						//2021_1_28 add
						//縮小した場合に使う
						cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x - (1.0 / 5.0)*yolo_buf_bbox.width), int(yolo_buf_bbox.y - (1.0 / 5.0)*yolo_buf_bbox.height)), cv::Point(int(yolo_buf_bbox.x + (6.0 / 5.0)*yolo_buf_bbox.width), int(yolo_buf_bbox.y + (6.0 / 5.0)*yolo_buf_bbox.height)), cv::Scalar(0, 0, 0), 1);

					}



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
							per_yolo_lidar_distance_result_data << point_cloud_LiDAR_yzx[j].x << "\t" << point_cloud_LiDAR_yzx[j].y << "\t" << point_cloud_LiDAR_yzx[j].z << "\n";


							//fprintf_s(per_yolo_lidar_distance_result_data, "%lf\t%lf\t%lf\n", point_cloud[j].x, point_cloud[j].y, point_cloud[j].z);

							////////////////////////////////////////////////////////////////////////////








							//2020_12_1
							////////////////////////////////////////////////////////////////////////////


							//LiDARの点群を画像に出力
							cv::circle(img_out, cv::Point((int)imagePoints_LiDAR2ZED[j].x, (int)imagePoints_LiDAR2ZED[j].y), 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);


							////////////////////////////////////////////////////////////////////////////






						}


					}



					//2021_1_6追記
					//YOLO_LiDAR_distance_result_dataに改行を2行追加
					per_yolo_lidar_distance_result_data << "\n" << "\n";





					//2021_2_2 add
					NN_method(all_point_cloud_in_a_boundingbox_3D, 1.0, NN);



					//NN法の結果を書き出す
					for (int i = 0; i < (int)NN.size(); i++)
					{
						fprintf_s(fp_w_NN, "%d\t%lf\t%lf\t%lf\t%d\n", i_yolo_lidar, NN[i].cx, NN[i].cy, NN[i].cz, NN[i].count);
					}

					fprintf_s(fp_w_NN, "\n\n");

					






					//write the result of nn method to the text file





					////////////////////////////////////////////////////////////////////////////

					//2021_2_1 add ←計算量が半端ない！（特にPerson）
					//calculate_Representative_LiDAR_point(all_point_cloud_in_a_boundingbox_2D, all_point_cloud_in_a_boundingbox_3D, all_distance_from_min, all_distance_from_min_3D, all_distance_from_min_2D);


					//for (size_t a = 0; a < (int)all_distance_from_min_2D.size(); a++)
					//{
					//	//printf("all_distance_from_min_container_2D[%d] = %f\t%f\n", a, all_distance_from_min_container_2D[a].x, all_distance_from_min_container_2D[a].y);


					//	//LiDARの点群を画像に出力
					//	cv::circle(img_out, cv::Point((int)all_distance_from_min_2D[a].x, (int)all_distance_from_min_2D[a].y), 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);

					//}




					//get_mediam(histgram_textfile_path, all_distance_from_min, all_distance_from_min_3D, all_distance_from_min_2D);


					////////////////////////////////////////////////////////////////////////////






					//2021_1_12追記
					//Calculate the median

					//get_mediam(histgram_textfile_path, all_lidar_distance_in_boundingbox, all_point_cloud_in_a_boundingbox_3D, all_point_cloud_in_a_boundingbox_2D, median_distance);











					//memory clear
					point_cloud.clear();                                  point_cloud.shrink_to_fit();
					point_cloud_LiDAR_yzx.clear();                        point_cloud_LiDAR_yzx.shrink_to_fit();
					all_lidar_distance.clear();                           all_lidar_distance.shrink_to_fit();
					imagePoints_LiDAR2ZED.clear();                        imagePoints_LiDAR2ZED.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_2D.clear();          all_point_cloud_in_a_boundingbox_2D.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_3D.clear();          all_point_cloud_in_a_boundingbox_3D.shrink_to_fit();
					all_lidar_distance_in_boundingbox.clear();            all_lidar_distance_in_boundingbox.shrink_to_fit();


					//2021_1_30 add
					


					//2021_2_1 add
					all_distance_from_min.clear();                      all_distance_from_min.shrink_to_fit();
					all_distance_from_min_2D.clear();                   all_distance_from_min_2D.shrink_to_fit();
					all_distance_from_min_3D.clear();                   all_distance_from_min_3D.shrink_to_fit();


					//2021_2_2 add
					NN.clear();            NN.shrink_to_fit();







					//小数点以下の指定(https://mstn.hateblo.jp/entry/2018/01/25/160457)(http://koshinran.hateblo.jp/entry/2018/04/04/234111)
					int digit = 1; //小数第1位まで表示
					std::ostringstream oss;
					oss << std::fixed << std::setprecision(digit) << median_distance;
					std::string median_num_new = oss.str();



					if (transporting)
					{
						//cv::putText(img_out, median_num_new + "m", cv::Point(int(yolo_buf_bbox.x - 40), int(yolo_buf_bbox.y)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
						continue;
					}
					else
					{
						cv::putText(img_out, median_num_new + "m", cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width + 5), int(yolo_buf_bbox.y)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
					}


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



		per_yolo_lidar_distance_result_data.close();



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




	//prepare to close gnuplot 
	//_pclose(gid); 








	std::ofstream ofs_BBox_container;
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

	ofs_BBox_person.close();








	fclose(fp_w_NN);








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






//2021_1_26 add
double masuda_calc_IoU()
{

	struct struct_rectangle bbox_prev_container, bbox_current_container;
	struct struct_rectangle bbox_prev_person, bbox_current_person;



	struct yolo_bounding_box input_bbox_container_vec;
	struct yolo_bounding_box input_bbox_person_vec;

	struct yolo_bounding_box in_yolo_bbox_container;
	struct yolo_bounding_box in_yolo_bbox_person;

	vector<struct yolo_bounding_box> targeted_yolo_BBox_container;
	vector<struct yolo_bounding_box> targeted_yolo_BBox_person;



	double d_IoU_container;
	double d_IoU_person;


	double IoU_max_container;
	int n_object_IoU_max_id_container;

	double IoU_max_person;
	int n_object_IoU_max_id_person;


	int n_object_tracked_bbox_count_container;
	int n_object_tracked_bbox_count_person;



	double IoU_thresh = 0.01;







	//container

	for (size_t num = 0; num < IN_yolo_BBox_buf_container.size(); num++)
	{


		In_yolo_BBox_container.clear();
		In_yolo_BBox_container.shrink_to_fit();

		input_bbox_container_vec = IN_yolo_BBox_buf_container[num];

		In_yolo_BBox_container.push_back(input_bbox_container_vec);


		//printf("%d", (int)In_yolo_BBox_container.size());

		for (size_t f = 0; f < In_yolo_BBox_container.size(); f++)
		{
			in_yolo_bbox_container.x = In_yolo_BBox_container[f].x;
			in_yolo_bbox_container.y = In_yolo_BBox_container[f].y;
			in_yolo_bbox_container.width = In_yolo_BBox_container[f].width;
			in_yolo_bbox_container.height = In_yolo_BBox_container[f].height;


			if (Objects_yolo_BBox_container.size() == 0)
			{
				targeted_yolo_BBox_container.clear();
				targeted_yolo_BBox_container.shrink_to_fit();

				targeted_yolo_BBox_container.push_back(in_yolo_bbox_container);

				Objects_yolo_BBox_container.push_back(targeted_yolo_BBox_container);
			}
			else
			{
				IoU_max_container = -1.0;

				for (size_t k = 0; k < Objects_yolo_BBox_container.size(); k++)
				{
					n_object_tracked_bbox_count_container = (int)Objects_yolo_BBox_container[k].size();



					bbox_prev_container.x0 = Objects_yolo_BBox_container[k][n_object_tracked_bbox_count_container - 1].x;
					bbox_prev_container.y0 = Objects_yolo_BBox_container[k][n_object_tracked_bbox_count_container - 1].y;

					bbox_prev_container.x1 = bbox_prev_container.x0 + Objects_yolo_BBox_container[k][n_object_tracked_bbox_count_container - 1].width;
					bbox_prev_container.y1 = bbox_prev_container.y0 + Objects_yolo_BBox_container[k][n_object_tracked_bbox_count_container - 1].height;


					bbox_current_container.x0 = in_yolo_bbox_container.x;
					bbox_current_container.y0 = in_yolo_bbox_container.y;

					bbox_current_container.x1 = bbox_current_container.x0 + in_yolo_bbox_container.width;
					bbox_current_container.y1 = bbox_current_container.y0 + in_yolo_bbox_container.height;



					d_IoU_container = calc_IoU(&bbox_prev_container, &bbox_current_container);





					//2021_1_28 add Hangarian method ?










					//printf("IoU = %lf.", d_IoU_container);

					if (d_IoU_container > IoU_max_container)
					{
						IoU_max_container = d_IoU_container;
						n_object_IoU_max_id_container = k;
					}

				}


				if (IoU_max_container > IoU_thresh)
				{
					Objects_yolo_BBox_container[n_object_IoU_max_id_container].push_back(in_yolo_bbox_container);
				}
				else
				{
					targeted_yolo_BBox_container.clear();
					targeted_yolo_BBox_container.shrink_to_fit();

					targeted_yolo_BBox_container.push_back(in_yolo_bbox_container);
					Objects_yolo_BBox_container.push_back(targeted_yolo_BBox_container);
				}


				/*printf("\n--------------------------------------------\n");

				for (int p = 0; p < Objects_yolo_BBox_container.size(); p++)
				{
				for (int q = 0; q < Objects_yolo_BBox_container[p].size(); q++)
				{
				printf("[%d %d]\t", Objects_yolo_BBox_container[p][q].x, Objects_yolo_BBox_container[p][q].y);
				}

				printf("\n");
				}*/



			}


		}


	}




	//person

	for (size_t num = 0; num < IN_yolo_BBox_buf_person.size(); num++)
	{
		In_yolo_BBox_person.clear();
		In_yolo_BBox_person.shrink_to_fit();

		input_bbox_person_vec = IN_yolo_BBox_buf_person[num];

		In_yolo_BBox_person.push_back(input_bbox_person_vec);



		for (size_t f = 0; f < In_yolo_BBox_person.size(); f++)
		{
			in_yolo_bbox_person.x = In_yolo_BBox_person[f].x;
			in_yolo_bbox_person.y = In_yolo_BBox_person[f].y;
			in_yolo_bbox_person.width = In_yolo_BBox_person[f].width;
			in_yolo_bbox_person.height = In_yolo_BBox_person[f].height;


			if (Objects_yolo_BBox_person.size() == 0)
			{
				targeted_yolo_BBox_person.clear();
				targeted_yolo_BBox_person.shrink_to_fit();

				targeted_yolo_BBox_person.push_back(in_yolo_bbox_person);

				Objects_yolo_BBox_person.push_back(targeted_yolo_BBox_person);
			}
			else
			{
				IoU_max_person = -1.0;

				for (size_t k = 0; k < Objects_yolo_BBox_person.size(); k++)
				{
					n_object_tracked_bbox_count_person = (int)Objects_yolo_BBox_person[k].size();


					bbox_prev_person.x0 = Objects_yolo_BBox_person[k][n_object_tracked_bbox_count_person - 1].x;
					bbox_prev_person.y0 = Objects_yolo_BBox_person[k][n_object_tracked_bbox_count_person - 1].y;

					bbox_prev_person.x1 = bbox_prev_person.x0 + Objects_yolo_BBox_person[k][n_object_tracked_bbox_count_person - 1].width;
					bbox_prev_person.y1 = bbox_prev_person.y0 + Objects_yolo_BBox_person[k][n_object_tracked_bbox_count_person - 1].height;


					bbox_current_person.x0 = in_yolo_bbox_person.x;
					bbox_current_person.y0 = in_yolo_bbox_person.y;

					bbox_current_person.x1 = bbox_current_person.x0 + in_yolo_bbox_person.width;
					bbox_current_person.y1 = bbox_current_person.y0 + in_yolo_bbox_person.height;



					d_IoU_person = calc_IoU(&bbox_prev_person, &bbox_current_person);





					//2021_1_28 add Hangarian method ?










					if (d_IoU_person > IoU_max_person)
					{
						IoU_max_person = d_IoU_person;
						n_object_IoU_max_id_person = k;
					}

				}



				if (IoU_max_person > IoU_thresh)
				{
					Objects_yolo_BBox_person[n_object_IoU_max_id_person].push_back(in_yolo_bbox_person);
				}
				else
				{
					targeted_yolo_BBox_person.clear();
					targeted_yolo_BBox_person.shrink_to_fit();

					targeted_yolo_BBox_person.push_back(in_yolo_bbox_person);
					Objects_yolo_BBox_person.push_back(targeted_yolo_BBox_person);
				}

			}


		}


	}







	return 0;

}








//2021_1_28 add  HangarianMethod










//2021_2_2 add for calculating nn method
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



	/*for (size_t k = 0; k < input_data_xyz_3D.size(); k++)
	{
		printf("type cx xy cz = %d\t%f\t%f\t%f\n", input_data_xyz_3D[k].type, input_data_xyz_3D[k].x, input_data_xyz_3D[k].y, input_data_xyz_3D[k].z);

	}


	for (size_t k = 0; k < nn.size(); k++)
	{
		printf("type cx xy cz = %d\t%f\t%f\t%f\n",k, nn[k].cx, nn[k].cy, nn[k].cz);

	}*/




	//nn.clear();
	//nn.shrink_to_fit();



	return 0;
}











//2021_2_1 add for calculating the representative LiDAR point cloud←たぶん使わない（処理時間がかなりかかる！）
int calculate_Representative_LiDAR_point(vector<Point2f> input_data_xy_2D, std::vector<Point3f>input_data_xyz_3D,
	vector<double> &all_distance_from_min, vector<Point3f>&all_distance_from_min_3d, vector<Point2f> &all_distance_from_min_2d)
{



	//input data・・・ all_point_cloud_in_a_boundingbox_2D
	//output data・・・all_lidar_distance_in_boundingbox(Containerとpersonに分けた結果を出力するかも)


	cv::Point2f point_cloud_in_a_boundingbox_2d;
	cv::Point3f point_cloud_in_a_boundingbox_3d;

	vector<Point2f> all_point_cloud_in_a_boundingbox_2d;
	vector<Point3f> all_point_cloud_in_a_boundingbox_3d;


	double distance_container_3d;

	vector<double> Pointcloud_in_BB_pointcloud_distance;


	double min_distance_3d;
	double distance_from_min_3d;

	vector<Point2f> Pointcloud_in_BB_pointcloud_2d;
	vector<Point3f> Pointcloud_in_BB_pointcloud_3d;






	//threshold (2D)
	float threshold_2d = 50.0;


	//distance from the minimum of the point cloud
	double threshold_distance_3d = 5.0;







	for (size_t i = 0; i < input_data_xy_2D.size(); i++)
	{

		//2D
		point_cloud_in_a_boundingbox_2d.x = input_data_xy_2D[i].x;
		point_cloud_in_a_boundingbox_2d.y = input_data_xy_2D[i].y;

		all_point_cloud_in_a_boundingbox_2d.push_back(point_cloud_in_a_boundingbox_2d);



		//3D
		point_cloud_in_a_boundingbox_3d.x = input_data_xyz_3D[i].x;
		point_cloud_in_a_boundingbox_3d.y = input_data_xyz_3D[i].y;
		point_cloud_in_a_boundingbox_3d.z = input_data_xyz_3D[i].z;

		all_point_cloud_in_a_boundingbox_3d.push_back(point_cloud_in_a_boundingbox_3d);

	}



	for (size_t i = 0; i < all_point_cloud_in_a_boundingbox_2d.size(); i++)
	{

		//printf("//////////////////////////////////////////////////////////////////////////////////////////////////////////\n");
		//printf("//////////////////////////////////////////////////////////////////////////////////////////////////////////\n\n");


		//printf("all_point_cloud_in_a_boundingbox_2d[%d]=%f\t%f\n\n\n", i, all_point_cloud_in_a_boundingbox_2d[i].x, all_point_cloud_in_a_boundingbox_2d[i].y);




		for (size_t j = 0; j < all_point_cloud_in_a_boundingbox_2d.size(); j++)
		{
			if (j != i)
			{

				if ((all_point_cloud_in_a_boundingbox_2d[j].x >= all_point_cloud_in_a_boundingbox_2d[i].x - threshold_2d) &&
					(all_point_cloud_in_a_boundingbox_2d[j].x <= all_point_cloud_in_a_boundingbox_2d[i].x + threshold_2d) &&
					(all_point_cloud_in_a_boundingbox_2d[j].y >= all_point_cloud_in_a_boundingbox_2d[i].y - threshold_2d) &&
					(all_point_cloud_in_a_boundingbox_2d[j].y <= all_point_cloud_in_a_boundingbox_2d[i].y + threshold_2d))
				{
					//コマンドプロンプト上に書き出す
					//書き出す項目↓
					//obuject name, all_point_cloud_in_a_boundingbox_2d[i], all_point_cloud_in_a_boundingbox_2d[j]



					//std::cout << all_point_cloud_in_a_boundingbox_2d[j] << std::endl;
					//std::cout << all_point_cloud_in_a_boundingbox_32d[j] << std::endl;



					Pointcloud_in_BB_pointcloud_2d.push_back(all_point_cloud_in_a_boundingbox_2d[j]);    //2D push_back
					Pointcloud_in_BB_pointcloud_3d.push_back(all_point_cloud_in_a_boundingbox_3d[j]);    //3D push_back


					distance_container_3d = sqrt((all_point_cloud_in_a_boundingbox_3d[j].x*all_point_cloud_in_a_boundingbox_3d[j].x) + (all_point_cloud_in_a_boundingbox_3d[j].y*all_point_cloud_in_a_boundingbox_3d[j].y) + (all_point_cloud_in_a_boundingbox_3d[j].z*all_point_cloud_in_a_boundingbox_3d[j].z));
					Pointcloud_in_BB_pointcloud_distance.push_back(distance_container_3d);  //3D distance push_back


					//printf("distance_container_3d = %lf\n", distance_container_3d);

				}

			}
			else
			{
				continue;
			}

		}






		//calculate the minimum distance

		min_distance_3d = Pointcloud_in_BB_pointcloud_distance[0];


		for (size_t a = 0; a < Pointcloud_in_BB_pointcloud_distance.size(); a++)
		{

			if (Pointcloud_in_BB_pointcloud_distance[a] < min_distance_3d)
			{
				min_distance_3d = Pointcloud_in_BB_pointcloud_distance[a];
			}

		}



		//printf("min_distance_3d = %lf\n\n", min_distance_3d);



		//calculate from the minimum distance
		for (size_t a = 0; a < Pointcloud_in_BB_pointcloud_distance.size(); a++)
		{

			distance_from_min_3d = Pointcloud_in_BB_pointcloud_distance[a] - min_distance_3d;


			if (distance_from_min_3d <= threshold_distance_3d)
			{

				//printf("Pointcloud_in_BB_pointcloud_2d[%d] = %lf\t%lf\n", a, Pointcloud_in_BB_pointcloud_2d[a].x, Pointcloud_in_BB_pointcloud_2d[a].y);
				//printf("Pointcloud_in_BB_pointcloud_distance[%d] = %lf\n\n", a, Pointcloud_in_BB_pointcloud_distance[a]);



				//distance
				all_distance_from_min.push_back(Pointcloud_in_BB_pointcloud_distance[a]);

				//2D
				all_distance_from_min_2d.push_back(Pointcloud_in_BB_pointcloud_2d[a]);

				//3D
				all_distance_from_min_3d.push_back(Pointcloud_in_BB_pointcloud_3d[a]);
			}




		}



		//printf("//////////////////////////////////////////////////////////////////////////////////////////////////////////\n");
		//printf("//////////////////////////////////////////////////////////////////////////////////////////////////////////\n\n");



	}





	//clear
	all_point_cloud_in_a_boundingbox_2d.clear();             all_point_cloud_in_a_boundingbox_2d.shrink_to_fit();
	all_point_cloud_in_a_boundingbox_3d.clear();             all_point_cloud_in_a_boundingbox_3d.shrink_to_fit();
	Pointcloud_in_BB_pointcloud_2d.clear();                  Pointcloud_in_BB_pointcloud_2d.shrink_to_fit();
	Pointcloud_in_BB_pointcloud_3d.clear();                  Pointcloud_in_BB_pointcloud_3d.shrink_to_fit();
	Pointcloud_in_BB_pointcloud_distance.clear();            Pointcloud_in_BB_pointcloud_distance.shrink_to_fit();




	return 0;

}









//Calculate the median
int get_mediam(std::string outputfile_path, std::vector<double> input_data_distance, std::vector<Point3f> input_data_xyz_3D, std::vector<Point2f> input_data_xy_2D, double &median_num)
{

	double tmp;
	float tmp_x;
	float tmp_y;
	float tmp_z;

	float tmp_x_2D;
	float tmp_y_2D;



	//C++
	//出力用
	//テキストファイルを用意
	/*std::string outputfile_name = outputfile_path + "\\yolo_distance_histgram_image_count_" + std::to_string(i_yolo_lidar) + "_" + yolo_buf_bbox.name + "_" + std::to_string(n_class) + ".txt";
	std::ofstream outputfile;
	outputfile.open(outputfile_name);

	if (!outputfile.is_open())
	{
		std::cerr << "Can not open " + outputfile_name << std::endl;
		return -1;
	}*/



	//Sort in descending order
	for (int a = 0; a < (int)input_data_distance.size(); a++)
	{
		for (int b = a + 1; b < (int)input_data_distance.size(); b++)
		{
			if (input_data_distance[a] < input_data_distance[b])
			{

				//distance
				tmp = input_data_distance[b];
				input_data_distance[b] = input_data_distance[a];
				input_data_distance[a] = tmp;


				//3D
				tmp_x = input_data_xyz_3D[b].x;
				input_data_xyz_3D[b].x = input_data_xyz_3D[a].x;
				input_data_xyz_3D[a].x = tmp_x;

				tmp_y = input_data_xyz_3D[b].y;
				input_data_xyz_3D[b].y = input_data_xyz_3D[a].y;
				input_data_xyz_3D[a].y = tmp_y;

				tmp_z = input_data_xyz_3D[b].z;
				input_data_xyz_3D[b].z = input_data_xyz_3D[a].z;
				input_data_xyz_3D[a].z = tmp_z;



				//2D
				tmp_x_2D = input_data_xy_2D[b].x;
				input_data_xy_2D[b].x = input_data_xy_2D[a].x;
				input_data_xy_2D[a].x = tmp_x_2D;

				tmp_y_2D = input_data_xy_2D[b].y;
				input_data_xy_2D[b].y = input_data_xy_2D[a].y;
				input_data_xy_2D[a].y = tmp_y_2D;


			}

		}

	}



	//テキストファイルに出力
	//for (int a = 0; a < (int)input_data_distance.size(); a++)
	//{
	//	//C++
	//	outputfile << a << "\t" << input_data_distance[a] << "\t" << input_data_xyz_3D[a].x << "\t" << input_data_xyz_3D[a].y << "\t" << input_data_xyz_3D[a].z << "\n";
	//}


	////C++
	//outputfile.close();






	//2021_2_1 add　←　6フレーム目で終了してしまう．恐らく，Bounding box内に点がないのでは？
	//find the median value
	median(input_data_distance, input_data_xyz_3D, median_num);






	n_class++;



	return 0;

}






int median(vector<double> input_data_distance_3D, vector<Point3f> input_data_xyz_3D, double &median_num)
{


	double median_distance_container;
	double median_distance_person;
	struct point_cloud_after_conversion median_num_container;
	struct point_cloud_after_conversion median_num_person;





	if ((int)input_data_distance_3D.size() == 0)
	{

		median_distance_container = 0.0;
		median_num_container.x = 0.0;
		median_num_container.y = 0.0;
		median_num_container.z = 0.0;

		median_distance_person = 0.0;
		median_num_person.x = 0.0;
		median_num_person.y = 0.0;
		median_num_person.z = 0.0;

	}
	else if ((int)input_data_distance_3D.size() % 2 == 1)
	{
		median_num = input_data_distance_3D[(int)((int)input_data_distance_3D.size() / 2)];


		if (container)
		{
			median_distance_container = median_num;
			median_num_container.x = input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].x;
			median_num_container.y = input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].y;
			median_num_container.z = input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].z;


			in_object_container.x = median_num_container.x;
			in_object_container.y = median_num_container.y;
			in_object_container.z = median_num_container.z;

			IN_Objects_buf_container.push_back(in_object_container);


			in_obj_points_container.x = in_object_container.x;
			in_obj_points_container.y = in_object_container.y;
			in_obj_points_container.z = in_object_container.z;

			d_median_points_container.push_back(in_obj_points_container);





			median_distance_person = 0.0;
			median_num_person.x = 0.0;
			median_num_person.y = 0.0;
			median_num_person.z = 0.0;

		}
		else if (person)
		{
			median_distance_person = median_num;
			median_num_person.x = input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].x;
			median_num_person.y = input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].y;
			median_num_person.z = input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].z;


			in_object_person.x = median_num_person.x;
			in_object_person.y = median_num_person.y;
			in_object_person.z = median_num_person.z;


			IN_Objects_buf_person.push_back(in_object_person);



			in_obj_points_person.x = in_object_person.x;
			in_obj_points_person.y = in_object_person.y;
			in_obj_points_person.z = in_object_person.z;

			d_median_points_person.push_back(in_obj_points_person);






			median_distance_container = 0.0;
			median_num_container.x = 0.0;
			median_num_container.y = 0.0;
			median_num_container.z = 0.0;
		}


	}
	else if ((int)input_data_distance_3D.size() % 2 == 0)
	{
		median_num = (input_data_distance_3D[(int)((int)input_data_distance_3D.size() / 2) - 1] + input_data_distance_3D[(int)((int)input_data_distance_3D.size() / 2)]) / 2;


		if (container)
		{


			median_distance_container = median_num;
			median_num_container.x = (input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2) - 1].x + input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].x) / 2;
			median_num_container.y = (input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2) - 1].y + input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].y) / 2;
			median_num_container.z = (input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2) - 1].z + input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].z) / 2;


			//median_distance_container[int(NumFromMedian / 2)] = (input_data_distance_3D[(int)((int)input_data_distance_3D.size() / 2) - 1] + input_data_distance_3D[(int)((int)input_data_distance_3D.size() / 2)]) / 2;
			//median_num_container[int(NumFromMedian / 2)].x = (input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2) - 1].x + input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].x) / 2;


			in_object_container.x = median_num_container.x;
			in_object_container.y = median_num_container.y;
			in_object_container.z = median_num_container.z;

			IN_Objects_buf_container.push_back(in_object_container);


			in_obj_points_container.x = in_object_container.x;
			in_obj_points_container.y = in_object_container.y;
			in_obj_points_container.z = in_object_container.z;

			d_median_points_container.push_back(in_obj_points_container);




			median_distance_person = 0.0;
			median_num_person.x = 0.0;
			median_num_person.y = 0.0;
			median_num_person.z = 0.0;


		}
		else if (person)
		{

			median_distance_person = median_num;
			median_num_person.x = (input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2) - 1].x + input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].x) / 2;
			median_num_person.y = (input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2) - 1].y + input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].y) / 2;
			median_num_person.z = (input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2) - 1].z + input_data_xyz_3D[(int)((int)input_data_distance_3D.size() / 2)].z) / 2;


			in_object_person.x = median_num_person.x;
			in_object_person.y = median_num_person.y;
			in_object_person.z = median_num_person.z;


			IN_Objects_buf_person.push_back(in_object_person);



			in_obj_points_person.x = in_object_person.x;
			in_obj_points_person.y = in_object_person.y;
			in_obj_points_person.z = in_object_person.z;

			d_median_points_person.push_back(in_obj_points_person);





			median_distance_container = 0.0;
			median_num_container.x = 0.0;
			median_num_container.y = 0.0;
			median_num_container.z = 0.0;
		}


	}


	all_median_distance_container.push_back(median_distance_container);
	all_median_num_container.push_back(median_num_container);

	all_median_distance_person.push_back(median_distance_person);
	all_median_num_person.push_back(median_num_person);


	return 0;

}








//find the median value (←修正前)

/*
int median()
{


	double median_distance_container;
	double median_distance_person;
	Point3f median_num_container;
	Point3f median_num_person;





	if ((int)all_lidar_distance_in_boundingbox.size() == 0)
	{

		median_distance_container = 0.0;
		median_num_container.x = 0.0;
		median_num_container.y = 0.0;
		median_num_container.z = 0.0;

		median_distance_person = 0.0;
		median_num_person.x = 0.0;
		median_num_person.y = 0.0;
		median_num_person.z = 0.0;

	}
	else if ((int)all_lidar_distance_in_boundingbox.size() % 2 == 1)
	{
		median_distance = all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)];


		if (container)
		{
			median_distance_container = median_distance;
			median_num_container.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x;
			median_num_container.y = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].y;
			median_num_container.z = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].z;


			in_object_container.x = median_num_container.x;
			in_object_container.y = median_num_container.y;
			in_object_container.z = median_num_container.z;

			IN_Objects_buf_container.push_back(in_object_container);


			in_obj_points_container.x = in_object_container.x;
			in_obj_points_container.y = in_object_container.y;
			in_obj_points_container.z = in_object_container.z;

			d_median_points_container.push_back(in_obj_points_container);





			median_distance_person = 0.0;
			median_num_person.x = 0.0;
			median_num_person.y = 0.0;
			median_num_person.z = 0.0;

		}
		else if (person)
		{
			median_distance_person = median_distance;
			median_num_person.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x;
			median_num_person.y = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].y;
			median_num_person.z = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].z;


			in_object_person.x = median_num_person.x;
			in_object_person.y = median_num_person.y;
			in_object_person.z = median_num_person.z;


			IN_Objects_buf_person.push_back(in_object_person);



			in_obj_points_person.x = in_object_person.x;
			in_obj_points_person.y = in_object_person.y;
			in_obj_points_person.z = in_object_person.z;

			d_median_points_person.push_back(in_obj_points_person);






			median_distance_container = 0.0;
			median_num_container.x = 0.0;
			median_num_container.y = 0.0;
			median_num_container.z = 0.0;
		}


	}
	else if ((int)all_lidar_distance_in_boundingbox.size() % 2 == 0)
	{
		median_distance = (all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1] + all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)]) / 2;


		if (container)
		{


			median_distance_container = median_distance;
			median_num_container.x = (all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].x + all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x) / 2;
			median_num_container.y = (all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].y + all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].y) / 2;
			median_num_container.z = (all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].z + all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].z) / 2;


			//median_distance_container[int(NumFromMedian / 2)] = (all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1] + all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)]) / 2;
			//median_num_container[int(NumFromMedian / 2)].x = (all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].x + all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x) / 2;


			in_object_container.x = median_num_container.x;
			in_object_container.y = median_num_container.y;
			in_object_container.z = median_num_container.z;

			IN_Objects_buf_container.push_back(in_object_container);


			in_obj_points_container.x = in_object_container.x;
			in_obj_points_container.y = in_object_container.y;
			in_obj_points_container.z = in_object_container.z;

			d_median_points_container.push_back(in_obj_points_container);




			median_distance_person = 0.0;
			median_num_person.x = 0.0;
			median_num_person.y = 0.0;
			median_num_person.z = 0.0;


		}
		else if (person)
		{

			median_distance_person = median_distance;
			median_num_person.x = (all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].x + all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x) / 2;
			median_num_person.y = (all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].y + all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].y) / 2;
			median_num_person.z = (all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].z + all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].z) / 2;


			in_object_person.x = median_num_person.x;
			in_object_person.y = median_num_person.y;
			in_object_person.z = median_num_person.z;


			IN_Objects_buf_person.push_back(in_object_person);



			in_obj_points_person.x = in_object_person.x;
			in_obj_points_person.y = in_object_person.y;
			in_obj_points_person.z = in_object_person.z;

			d_median_points_person.push_back(in_obj_points_person);





			median_distance_container = 0.0;
			median_num_container.x = 0.0;
			median_num_container.y = 0.0;
			median_num_container.z = 0.0;
		}


	}


	all_median_distance_container.push_back(median_distance_container);
	all_median_num_container.push_back(median_num_container);

	all_median_distance_person.push_back(median_distance_person);
	all_median_num_person.push_back(median_num_person);


	return 0;

}

*/











//2021_1_20 add
//Moving Trajectory
int vector_test_sample(FILE *gid)
{


	// tracking //
	double d_dx_container;
	double d_dy_container;
	double d_dz_container;

	double d_dx_person;
	double d_dy_person;
	double d_dz_person;

	int n_object_tracked_count_person;
	int n_object_tracked_count_container;



	double d_object_distance_container;
	double d_object_distance_min_container;
	int n_object_distance_min_id_container;

	struct tracking_object_type input_vec_container;



	double d_object_distance_person;
	double d_object_distance_min_person;
	int n_object_distance_min_id_person;

	struct tracking_object_type input_vec_person;



	/////////////////////////////////////////////////

	double d_object_distance_thresh = 1.0;

	/////////////////////////////////////////////////




	//container//

	for (size_t num = 0; num < IN_Objects_buf_container.size(); num++)
	{
		IN_Objects_container.clear();    IN_Objects_container.shrink_to_fit();
		input_vec_container = IN_Objects_buf_container[num];
		IN_Objects_container.push_back(input_vec_container);



		//std::cout << "current container frame: " << to_string((int)num + 1) << " / " << IN_Objects_buf_container.size() << std::endl;



		for (size_t f = 0; f < IN_Objects_container.size(); f++)
		{//each frame

			in_object_container.x = IN_Objects_container[f].x;
			in_object_container.y = IN_Objects_container[f].y;
			in_object_container.z = IN_Objects_container[f].z;
			in_object_container.type = IN_Objects_container[f].type;




			if (Objects_container.size() == 0)
			{
				targeted_object_container.clear();     targeted_object_container.shrink_to_fit();

				targeted_object_container.push_back(in_object_container);

				Objects_container.push_back(targeted_object_container); //add the 1st object
			}
			else
			{
				// search the nearest object //
				d_object_distance_min_container = 100000;

				for (size_t k = 0; k < Objects_container.size(); k++)
				{

					n_object_tracked_count_container = (int)Objects_container[k].size();
					d_dx_container = (in_object_container.x - Objects_container[k][n_object_tracked_count_container - 1].x);
					d_dy_container = (in_object_container.y - Objects_container[k][n_object_tracked_count_container - 1].y);
					d_dz_container = (in_object_container.z - Objects_container[k][n_object_tracked_count_container - 1].z);

					d_object_distance_container = sqrt(d_dx_container*d_dx_container + d_dy_container*d_dy_container + d_dz_container*d_dz_container);

					if (d_object_distance_min_container > d_object_distance_container)
					{
						d_object_distance_min_container = d_object_distance_container;
						n_object_distance_min_id_container = k;
					}
				}


				// check the distance between the input object and the nearest object //
				if (d_object_distance_min_container < d_object_distance_thresh)
				{
					Objects_container[n_object_distance_min_id_container].push_back(in_object_container);
				}
				else
				{
					targeted_object_container.clear();     targeted_object_container.shrink_to_fit();
					targeted_object_container.push_back(in_object_container);

					Objects_container.push_back(targeted_object_container);
				}


			}

		}

	}









	//person//

	for (size_t num = 0; num < IN_Objects_buf_person.size(); num++)
	{
		IN_Objects_person.clear();    IN_Objects_person.shrink_to_fit();
		input_vec_person = IN_Objects_buf_person[num];
		IN_Objects_person.push_back(input_vec_person);



		//std::cout << "current person frame: " << to_string((int)num + 1) << " / " << IN_Objects_buf_person.size() << std::endl;



		for (size_t f = 0; f < IN_Objects_person.size(); f++)
		{//each frame

			in_object_person.x = IN_Objects_person[f].x;
			in_object_person.y = IN_Objects_person[f].y;
			in_object_person.z = IN_Objects_person[f].z;
			in_object_person.type = IN_Objects_person[f].type;




			if (Objects_person.size() == 0)
			{
				targeted_object_person.clear();     targeted_object_person.shrink_to_fit();

				targeted_object_person.push_back(in_object_person);

				Objects_person.push_back(targeted_object_person); //add the 1st object
			}
			else
			{
				// search the nearest object //
				d_object_distance_min_person = 100000;

				for (size_t k = 0; k < Objects_person.size(); k++)
				{

					n_object_tracked_count_person = (int)Objects_person[k].size();
					d_dx_person = (in_object_person.x - Objects_person[k][n_object_tracked_count_person - 1].x);
					d_dy_person = (in_object_person.y - Objects_person[k][n_object_tracked_count_person - 1].y);
					d_dz_person = (in_object_person.z - Objects_person[k][n_object_tracked_count_person - 1].z);

					d_object_distance_person = sqrt(d_dx_person*d_dx_person + d_dy_person*d_dy_person + d_dz_person*d_dz_person);

					if (d_object_distance_min_person > d_object_distance_person)
					{
						d_object_distance_min_person = d_object_distance_person;
						n_object_distance_min_id_person = k;
					}

				}


				// check the distance between the input object and the nearest object //
				if (d_object_distance_min_person < d_object_distance_thresh)
				{
					Objects_person[n_object_distance_min_id_person].push_back(in_object_person);
				}
				else
				{
					targeted_object_person.clear();     targeted_object_person.shrink_to_fit();
					targeted_object_person.push_back(in_object_person);

					Objects_person.push_back(targeted_object_person);
				}


			}

		}


	}







	//2021_1_25 addition
	//Display "Transporting" if the representative distance from the person to container is minimum. 









	//////////////////////////////////////////////////////////////////

	//show the result of moving trajectory using gnuplot.←これを使う
	show_objects_pose_via_gnuplot(gid);

	//////////////////////////////////////////////////////////////////






	// check the tracked object data //

	/*for (size_t j = 0; j < Objects_container.size(); j++)
	{

	cout << "Object container: " << j << endl;

	for (size_t i = 0; i < Objects_container[j].size(); i++)
	{

	cout << j << "\t" << i << "\t" << to_string(Objects_container[j][i].x) << "," << to_string(Objects_container[j][i].y) << "," << to_string(Objects_container[j][i].z) << endl;

	}

	cout << endl;

	}




	for (size_t j = 0; j < Objects_person.size(); j++)
	{

	cout << "Object person: " << j << endl;

	for (size_t i = 0; i < Objects_person[j].size(); i++)
	{

	cout << j << "\t" << i << "\t" << to_string(Objects_person[j][i].x) << "," << to_string(Objects_person[j][i].y) << "," << to_string(Objects_person[j][i].z) << endl;

	}

	cout << endl;

	}*/





	return 0;

}









//2021_1_25 add
//Display "Transporting" if the representative distance from the person to container is minimum. 
int display_transporting(double threshold, vector<struct tracking_object_type> in_object_buf_person, vector<struct tracking_object_type> in_objects_buf_container)
{

	double x, y, z;
	double distance;

	double distance_min = 100000;





	for (size_t i = 0; i < in_object_buf_person.size(); i++)
	{
		for (size_t j = 0; j < in_objects_buf_container.size(); j++)
		{
			x = in_object_buf_person[i].x - in_objects_buf_container[j].x;
			y = in_object_buf_person[i].y - in_objects_buf_container[j].y;
			z = in_object_buf_person[i].z - in_objects_buf_container[j].z;


			distance = sqrt(x*x + y*y + z*z);

			if (distance < distance_min)
			{
				distance_min = distance;
			}

		}
	}

}











//2021_1_24 add
int show_objects_pose_via_gnuplot(FILE *gid)
{


	fprintf(gid, "unset multiplot\n");

	fprintf(gid, "set multiplot\n");
	fprintf(gid, "set size ratio -1\n");
	fprintf(gid, "set xl 'X[m]'\n");
	fprintf(gid, "set yl 'Z[m]'\n");
	fprintf(gid, "set xr [-30:10]\n");
	fprintf(gid, "set yr [0:40]\n");

	fprintf(gid, "set key left outside\n");
	fflush(gid);


	fprintf(gid, "set lmargin screen 0.2\n");
	fprintf(gid, "set rmargin screen 0.8\n");
	fprintf(gid, "set tmargin screen 0.8\n");
	fprintf(gid, "set bmargin screen 0.2\n");







	//fprintf(gid, "plot for [i=0:*] '-' index i with lines title 'container', for [k=0:*] '-' index k with linespoints title 'person'\n");
	fprintf(gid, "plot for [i=0:*] '-' index i with lines title 'Container'\n");

	for (size_t i = 0; i < Objects_container.size(); i++)
	{
		for (size_t j = 0; j < Objects_container[i].size(); j++)
		{
			fprintf(gid, "%lf\t%lf\n", (double)Objects_container[i][j].x, (double)Objects_container[i][j].z);
		}

		fprintf(gid, "\n\n");
	}

	fprintf(gid, "e\n");
	fflush(gid);


	fprintf(gid, "set lmargin screen 0.2\n");
	fprintf(gid, "set rmargin screen 0.8\n");
	fprintf(gid, "set tmargin screen 0.8\n");
	fprintf(gid, "set bmargin screen 0.2\n");
	fprintf(gid, "set key right outside\n");

	fprintf(gid, "plot for [i=0:*] '-' index i with linespoints title 'Person'\n");

	for (size_t i = 0; i < Objects_person.size(); i++)
	{
		for (size_t j = 0; j < Objects_person[i].size(); j++)
		{
			fprintf(gid, "%lf\t%lf\n", (double)Objects_person[i][j].x, (double)Objects_person[i][j].z);
		}

		fprintf(gid, "\n\n");
	}

	fprintf(gid, "e\n");
	fflush(gid);






	/*fprintf(gid, "plot for [i=0:*] '-' index i with lines\n");

	for (size_t h = 0; h < Objects_all.size(); h++)
	{
	for (size_t i = 0; i < Objects_all[h].size(); i++)
	{
	for (size_t j = 0; j = Objects_all[h][i].size(); j++)
	{
	fprintf(gid, "%lf\t%lf\n", (double)Objects_all[h][i][j].x, (double)Objects_all[h][i][j].z);
	}


	fprintf(gid, "\n\n");

	}

	}*/



	fprintf(gid, "pause 0.5\n");
	fflush(gid);


	return 0;
}














//Bounding box内のLiDARの点群をマッピング（gnuplot）(http://nalab.mind.meiji.ac.jp/~mk/labo/howto/intro-gnuplot/node24.html)
int gnuplot_mapping()
{
	FILE *gid;
	gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");



	//fprintf(gid, "set size ratio -1\n");
	fprintf(gid, "set xrange[-20:20]\n");
	fprintf(gid, "set yrange[0:40]\n");

	fprintf(gid, "set xlabel 'x[m]'\n");
	fprintf(gid, "set ylabel 'z[m]'\n");




	//gnuplotの結果を保存（http://www.gnuplot-cmd.com/in-out/output.html）
	fprintf(gid, "set terminal png\n");







	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//都度，変更する！！

	//①Only median

	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\Gnuplot_result\\mapping_image_20201117_low\\image%d.png'\n", i_yolo_lidar);
	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\Gnuplot_result\\mapping_image_20201223112457\\image%d.png'\n", i_yolo_lidar);


	////reduction_ver
	fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\Gnuplot_result\\mapping_image_20201117_low_reduction_ver\\image%d.png'\n", i_yolo_lidar);
	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\Gnuplot_result\\mapping_image_20201223112457_reduction_ver\\image%d.png'\n", i_yolo_lidar);



	//②All point cloud in the bounding box

	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\Gnuplot_result\\mapping_image_20201117_low_all\\image%d.png'\n", i_yolo_lidar);
	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\Gnuplot_result\\mapping_image_20201223112457_all\\image%d.png'\n", i_yolo_lidar);


	////reduction_ver
	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\Gnuplot_result\\mapping_image_20201117_low_reduction_ver_all\\image%d.png'\n", i_yolo_lidar);
	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\Gnuplot_result\\mapping_image_20201223112457_reduction_ver_all\\image%d.png'\n", i_yolo_lidar);



	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




	fprintf_s(gid, "plot '-' with points ps 1 lc rgb 'red' title 'Container in bounding box', '-' with points ps 1 lc rgb 'blue' title 'Person in bounding box'\n");




	///////////////////////////////////////////////////////////////////////////////////////////////

	//①Only median

	for (size_t a = 0; a < all_median_num_container.size(); a++)
	{
		fprintf(gid, "%f\t%f\n", all_median_num_container[a].x, all_median_num_container[a].z);
	}


	fprintf(gid, "e\n");


	for (size_t a = 0; a < all_median_num_person.size(); a++)
	{
		fprintf(gid, "%f\t%f\n", all_median_num_person[a].x, all_median_num_person[a].z);
	}





	//②All point cloud in the bounding box

	/*for (size_t a = 0; a < all_point_cloud_in_a_boundingbox_3D_container_for_gnuplot.size(); a++)
	{
		fprintf(gid, "%f\t%f\n", all_point_cloud_in_a_boundingbox_3D_container_for_gnuplot[a].x, all_point_cloud_in_a_boundingbox_3D_container_for_gnuplot[a].z);
	}


	fprintf(gid, "e\n");


	for (size_t a = 0; a < all_point_cloud_in_a_boundingbox_3D_person_for_gnuplot.size(); a++)
	{
		fprintf(gid, "%f\t%f\n", all_point_cloud_in_a_boundingbox_3D_person_for_gnuplot[a].x, all_point_cloud_in_a_boundingbox_3D_person_for_gnuplot[a].z);
	}*/

	///////////////////////////////////////////////////////////////////////////////////////////////






	fprintf(gid, "set terminal windows\n");
	fprintf(gid, "set output\n");



	fprintf(gid, "e\n");
	fflush(gid);
	fprintf(gid, "pause mouse\n");
	fflush(gid);
	_pclose(gid);



	return 0;
}





//gnuplotでヒストグラムを表示
int gnuplot_histgram()
{

	struct hist_data per_hist_data;


	//open textfile
	std::cout << "Text file name = " << std::endl;
	std::string TextfileName;
	std::cin >> TextfileName;

	std::ifstream textfile;
	textfile.open(TextfileName);

	if (!textfile.is_open())
	{
		std::cerr << "Can not open " + TextfileName << std::endl;
		return -1;
	}


	FILE *gid;
	gid = _popen("c:/win64app/gnuplot/bin/gnuplot.exe", "w");

	//軸ラベル
	fprintf(gid, "set xlabel 'distance[m]' \n");
	fprintf(gid, "set ylabel 'count' \n");


	//目盛の間隔
	//fprintf(gid, "set xtics 5");
	fprintf(gid, "set ytics 1\n");


	//目盛の最大値
	fprintf(gid, "set xrange[0:50]\n");


	//凡例の位置
	fprintf(gid, "set key right top\n");

	//凡例のタイトルとグラフの形式
	fprintf(gid, "plot '-' title 'LiDAR point cloud in a bounding box' with boxes lw 2 \n");



	while (!textfile.eof())
	{
		textfile >> per_hist_data.i;
		textfile >> per_hist_data.count;

		std::cout << per_hist_data.i << "\t" << per_hist_data.count << "\n";


		fprintf(gid, "%d\t%d\n", per_hist_data.i, per_hist_data.count);
	}

	textfile.close();


	fprintf(gid, "e\n");
	fflush(gid);
	fprintf(gid, "pause mouse\n");
	fflush(gid);
	_pclose(gid);


	std::cout << "OK!" << std::endl;

	return 0;
}







//2021_1_26 add
//gnuplotで降順の結果を表示
int gnuplot_sort_result()
{
	vector<struct sort_data> all_sort_result;
	struct sort_data  sort_result;

	vector<struct sort_data> all_median_result;


	//open textfile
	std::cout << "Text file name = " << std::endl;
	std::string TextfileName;
	std::cin >> TextfileName;

	std::ifstream textfile;
	textfile.open(TextfileName);

	if (!textfile.is_open())
	{
		std::cerr << "Can not open " + TextfileName << std::endl;
		return -1;
	}


	while (!textfile.eof())
	{
		textfile >> sort_result.i;
		textfile >> sort_result.distance;
		textfile >> sort_result.x;
		textfile >> sort_result.y;
		textfile >> sort_result.z;


		std::cout << sort_result.i << "\t" << sort_result.distance << "\n";


		all_sort_result.push_back(sort_result);

	}



	for (size_t i = 0; i < all_sort_result.size(); i++)
	{
		if (all_sort_result.size() % 2 == 1)
		{

		}
	}



	textfile.close();






	FILE *gid;
	gid = _popen("c:/win64app/gnuplot/bin/gnuplot.exe", "w");

	//軸ラベル
	fprintf(gid, "set xlabel 'i' \n");
	fprintf(gid, "set ylabel 'distance[m]' \n");


	//目盛の間隔
	//fprintf(gid, "set xtics 5");
	//fprintf(gid, "set ytics 1\n");


	//目盛の最大値
	//fprintf(gid, "set xrange[0:50]\n");


	//凡例の位置
	fprintf(gid, "set key right top\n");





	//凡例のタイトルとグラフの形式
	fprintf(gid, "plot '-' title 'Point cloud in bounding box' with boxes\n");


	for (size_t i = 0; i < all_sort_result.size(); i++)
	{
		fprintf(gid, "%d\t%lf\n", all_sort_result[i].i, all_sort_result[i].distance);
	}




	fprintf(gid, "e\n");
	fflush(gid);
	fprintf(gid, "pause mouse\n");
	fflush(gid);
	_pclose(gid);


	std::cout << "OK!" << std::endl;

	return 0;
}

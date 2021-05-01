#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //テキストファイルを扱うために用意
#include <string>
#include <process.h>

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
void get_mediam(std::string outputfile_path);
void median();
void gnuplot_mapping();

int gnuplot_histgram();




//YOLOの認識結果のテキストファイル内の項目を読み込むために用意
struct yolo_bounfing_box
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




//ヒストグラムを表示するために用意（仮）
struct hist_data
{
	int i;
	int count;
};








//YOLO Variables

//yolo bounding box parameters
//vector<struct yolo_bounfing_box> yolo_BBox; //1まいの画像結果ごとに追加していくので，配列型（vector）にしておく
struct yolo_bounfing_box yolo_buf_bbox;

//LiDAR Variables
vector<struct point_cloud>point_cloud; //push_backするために用意
struct point_cloud point; //LiDARの出力結果(x,y,z,intensity)



std::vector<Point3f> point_cloud_LiDAR_yzx;
cv::Point3f buf_LiDAR_yzx;

std::vector<double>all_lidar_distance;
double lidar_distance;

//LiDAR 3D → 2D//
vector<Point2f> imagePoints_LiDAR2ZED;


std::vector<Point3f>all_point_cloud_in_a_boundingbox_3D;
cv::Point3f point_cloud_in_a_boundingbox_3D;


std::vector<double>all_lidar_distance_in_boundingbox;
double lidar_distance_in_boundingbox;



double median_num;

vector<double>all_median_distance_container;
double median_distance_container;

vector<double>all_median_distance_person;
double median_distance_person;




vector<Point3f>all_median_num_container;
Point3f median_num_container;
//Point3f median_num_container_prev;
//Point3f median_num_container_next;


vector<Point3f>all_median_num_person;
Point3f median_num_person;
//Point3f median_num_person_prev;
//Point3f median_num_person_next;







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


			gnuplot_histgram();

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







	VideoWriter writer(destination_file_path + "\\camera_lidar_calibration.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), n, Size(640, 480));


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


									//テキストファイル内の各1行目の文字を格納するために用意
	char yolo_bb_file_BUFFER[YOLO_FILE_LINE_MAX_BUFFER_SIZE]; //1024






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
	std::cout << "Folder for storing the text file of the histogram results = " << std::endl;
	std::string histgram_textfile_path;
	std::cin >> histgram_textfile_path;





	Mat img_in;
	Mat img_in_left;
	Mat img_out;
	std::string img_out_name;





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





			//初期化
			n_yolo_bb_file_stream_size = 0;



			//もし，テキストファイル内に文字があるなら
			if (fgets(yolo_bb_file_BUFFER, YOLO_FILE_LINE_MAX_BUFFER_SIZE, fp_yolo_bb) != NULL)
			{

				n_yolo_bb_file_stream_size = (int)strlen(yolo_bb_file_BUFFER);



				if (n_yolo_bb_file_stream_size == 2)
				{

					//memory clear
					//point_cloud.clear();                                  point_cloud.shrink_to_fit();
					//point_cloud_LiDAR_yzx.clear();                        point_cloud_LiDAR_yzx.shrink_to_fit();
					//all_lidar_distance.clear();                           all_lidar_distance.shrink_to_fit();
					//imagePoints_LiDAR2ZED.clear();                        imagePoints_LiDAR2ZED.shrink_to_fit();
					//all_point_cloud_in_a_boundingbox_3D.clear();          all_point_cloud_in_a_boundingbox_3D.shrink_to_fit();
					//all_lidar_distance_in_boundingbox.clear();            all_lidar_distance_in_boundingbox.shrink_to_fit();

					//all_point_cloud_in_a_boundingbox_container.clear();   all_point_cloud_in_a_boundingbox_container.shrink_to_fit();
					//all_point_cloud_in_a_boundingbox_person.clear();      all_point_cloud_in_a_boundingbox_person.shrink_to_fit();


					all_median_num_container.clear();                     all_median_num_container.shrink_to_fit();
					all_median_num_person.clear();                        all_median_num_person.shrink_to_fit();
					
					all_median_distance_container.clear();                all_median_distance_container.shrink_to_fit();
					all_median_distance_person.clear();                   all_median_distance_person.shrink_to_fit();




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


					if (transporting)
					{
						cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(255, 0, 0), 3);

						cv::putText(img_out, "transporting", cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

					}
					else
					{
						cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(0, 0, 255), 1);

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
						buf_LiDAR_yzx.x = (float)(-point_cloud[k].y);
						buf_LiDAR_yzx.y = (float)(-point_cloud[k].z);
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
						if ((imagePoints_LiDAR2ZED[j].x >= yolo_buf_bbox.x && imagePoints_LiDAR2ZED[j].x <= yolo_buf_bbox.x + yolo_buf_bbox.width) && (imagePoints_LiDAR2ZED[j].y >= yolo_buf_bbox.y && imagePoints_LiDAR2ZED[j].y <= yolo_buf_bbox.y + yolo_buf_bbox.height))
						{

							//YOLO内のLiDARの結果を書き込むファイルに『LiDARの結果』を書き込んでいく
							//fprintf_s(yolo_lidar_distance_result_data, "%lf\t%lf\t%lf\n", point_cloud[j].x, point_cloud[j].y, point_cloud[j].z);


							//Point cloud in the bounding box(3D)
							point_cloud_in_a_boundingbox_3D.x = point_cloud_LiDAR_yzx[j].x;
							point_cloud_in_a_boundingbox_3D.y = point_cloud_LiDAR_yzx[j].y;
							point_cloud_in_a_boundingbox_3D.z = point_cloud_LiDAR_yzx[j].z;


							all_point_cloud_in_a_boundingbox_3D.push_back(point_cloud_in_a_boundingbox_3D);





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




					//2021_1_12追記
					////////////////////////////////////////////////////////////////////////////
					//Calculate the median
					get_mediam(histgram_textfile_path);




					//Bounding box内のLiDAR点群をgnuplotに表示（2021_1_6追記） (2つ目のgnuplot_mapping)
					gnuplot_mapping();





					//memory clear
					point_cloud.clear();                                  point_cloud.shrink_to_fit();
					point_cloud_LiDAR_yzx.clear();                        point_cloud_LiDAR_yzx.shrink_to_fit();
					all_lidar_distance.clear();                           all_lidar_distance.shrink_to_fit();
					imagePoints_LiDAR2ZED.clear();                        imagePoints_LiDAR2ZED.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_3D.clear();          all_point_cloud_in_a_boundingbox_3D.shrink_to_fit();
					all_lidar_distance_in_boundingbox.clear();            all_lidar_distance_in_boundingbox.shrink_to_fit();






					//小数点以下の指定(https://mstn.hateblo.jp/entry/2018/01/25/160457)(http://koshinran.hateblo.jp/entry/2018/04/04/234111)
					int digit = 1; //小数第1位まで表示
					std::ostringstream oss;
					oss << std::fixed << std::setprecision(digit) << median_num;
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


	

	return 0;

}








//Calculate the median
void get_mediam(std::string outputfile_path)
{

	//テキストファイルを用意
	std::string outputfile_name = outputfile_path + "\\yolo_distance_histgram_image_count_" + std::to_string(i_yolo_lidar) + "_" + yolo_buf_bbox.name + "_" + std::to_string(n_class) + ".txt";


	double tmp;


	//C++
	//出力用
	std::ofstream outputfile;
	outputfile.open(outputfile_name);

	if (!outputfile.is_open())
	{
		std::cerr << "Can not open " + outputfile_name << std::endl;
		return;
	}



	//Sort in descending order
	for (int a = 0; a < (int)all_lidar_distance_in_boundingbox.size(); a++)
	{
		for (int b = a + 1; b < (int)all_lidar_distance_in_boundingbox.size(); b++)
		{
			if (all_lidar_distance_in_boundingbox[a] < all_lidar_distance_in_boundingbox[b])
			{
				tmp = all_lidar_distance_in_boundingbox[b];
				all_lidar_distance_in_boundingbox[b] = all_lidar_distance_in_boundingbox[a];
				all_lidar_distance_in_boundingbox[a] = tmp;
			}

		}

	}



	//テキストファイルに出力
	for (int a = 0; a < (int)all_lidar_distance_in_boundingbox.size(); a++)
	{
		//C++
		outputfile << a << "\t" << all_lidar_distance_in_boundingbox[a] << "\n";
	}


	//C++
	outputfile.close();


	//find the median value
	median();


	n_class++;


}







//find the median value
void median()
{

	if ((int)all_lidar_distance_in_boundingbox.size() == 0)
	{
		median_distance_container = 0.0;
		median_num_container.x = 0.0;
		median_distance_person = 0.0;
		median_num_person.x = 0.0;
	}
	else if ((int)all_lidar_distance_in_boundingbox.size() % 2 == 1)
	{
		median_num = all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)];


		if (container)
		{


			median_distance_container = all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)];
			median_num_container.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x;
			


			//median_num_container_prev.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2 - 1)].x;
			

			//median_num_container_next.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2 + 1)].x;
			




			median_distance_person = 0.0;
			median_num_person.x = 0.0;




		}
		else if (person)
		{


			median_distance_person = all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)];
			median_num_person.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x;


			//median_num_person_prev.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2 - 1)].x;

			//median_num_person_next.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2 + 1)].x;




			median_distance_container = 0.0;
			median_num_container.x = 0.0;


		}
		
	}
	else if((int)all_lidar_distance_in_boundingbox.size() % 2 == 0)
	{
		median_num = (all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1] + all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)]) / 2;


		if (container)
		{
			median_distance_container = (all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1] + all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)]) / 2;

			median_num_container.x = (all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].x + all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x) / 2;


			//2021_1_14追記
			//median_num_container_prev.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].x;

			//median_num_container_next.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x;



			median_distance_person = 0.0;
			median_num_person.x = 0.0;



		}
		else if (person)
		{

			median_distance_person = (all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1] + all_lidar_distance_in_boundingbox[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)]) / 2;
			median_num_person.x = (all_point_cloud_in_a_boundingbox_3D[((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].x + all_point_cloud_in_a_boundingbox_3D[(int)all_lidar_distance_in_boundingbox.size() / 2].x) / 2;


			//2021_1_14追記
			//median_num_person_prev.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2) - 1].x;

			//median_num_person_next.x = all_point_cloud_in_a_boundingbox_3D[(int)((int)all_lidar_distance_in_boundingbox.size() / 2)].x;



			median_distance_container = 0.0; //y axis
			median_num_container.x = 0.0; //x axis



		}
	

	}







	all_median_distance_container.push_back(median_distance_container);
	all_median_num_container.push_back(median_num_container);
	//2021_1_14追記
	//all_median_num_container.push_back(median_num_container_prev);
	//all_median_num_container.push_back(median_num_container_next);


	all_median_distance_person.push_back(median_distance_person);
	all_median_num_person.push_back(median_num_person);
	//2021_1_14追記
	//all_median_num_person.push_back(median_num_person_prev);
	//all_median_num_person.push_back(median_num_person_next);


}












//Bounding box内のLiDARの点群をマッピング（gnuplot）(http://nalab.mind.meiji.ac.jp/~mk/labo/howto/intro-gnuplot/node24.html)
void gnuplot_mapping()
{

	//bnuplotの作成(2021_1_13　追記)
	FILE *gid;

	gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");

	//fprintf(gid, "set size ratio -1\n");
	fprintf(gid, "set xrange[-20:20]\n");
	fprintf(gid, "set yrange[0:40]\n");

	fprintf(gid, "set xlabel 'x[m]'\n");
	fprintf(gid, "set ylabel 'distance[m]'\n");




	//gnuplotの結果を保存（http://www.gnuplot-cmd.com/in-out/output.html）
	fprintf(gid, "set terminal png\n");







	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//都度，変更する！！

	fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\mapping_image_20201117_low\\image%d.png'\n", i_yolo_lidar);
	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\mapping_image_20201223112457\\image%d.png'\n", i_yolo_lidar);
	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000_ver3\\mapping_image_20201223113541\\image%d.png'\n", i_yolo_lidar);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






	//fprintf_s(gid, "plot '-' index 0,'' index 1,'' index 2,'' index 3,'' index 4,'' index 5 with points ps 1\n");
	fprintf_s(gid, "plot '-' with points ps 1 lc rgb 'red' title 'Container in bounding box', '-' with points ps 1 lc rgb 'blue' title 'Person in bounding box'\n");




	for (int a = 0; a < (int)all_median_num_container.size(); a++)
	{
		fprintf(gid, "%f\t%lf\n", all_median_num_container[a].x, all_median_distance_container[a]);
	}


	fprintf(gid, "e\n");


	for (int a = 0; a < (int)all_median_num_person.size(); a++)
	{
		fprintf(gid, "%f\t%lf\n", all_median_num_person[a].x, all_median_distance_person[a]);
	}







	fprintf(gid, "set terminal windows\n");
	fprintf(gid, "set output\n");



	fprintf(gid, "e\n");
	fflush(gid);
	fprintf(gid, "pause mouse\n");
	fflush(gid);
	_pclose(gid);



	
}















//Bounding box内のLiDARの点群をマッピング（gnuplot）(スレッドver)
/*
unsigned _stdcall gnuplot_mapping(void *p)
{
FILE *gid;

gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");

//fprintf(gid, "set size ratio -1\n");
fprintf(gid, "set xrange[-10:10]\n");
fprintf(gid, "set yrange[0:20]\n");

fprintf(gid, "set xlabel 'x[m]'\n");
fprintf(gid, "set ylabel'z[m]'\n");


fflush(gid);


while (1)
{
//fprintf_s(gid, "plot '-' with points ps 1 title 'Trajectory of object movement'\n");


//gnuplotの結果を保存（http://www.gnuplot-cmd.com/in-out/output.html）
fprintf(gid, "set terminal png\n");


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//保存フォルダー名を変更する！//
//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201117_low\\image%d.png'\n", i_yolo_lidar);
//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201223112457\\image%d.png'\n", i_yolo_lidar);
//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201223113541\\image%d.png'\n", i_yolo_lidar);


//copyしたフォルダ（ヒストグラムのテスト用）
fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201117_low_copy\\image%d.png'\n", i_yolo_lidar);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




//fprintf_s(gid, "plot '-' with points ps 1 lc rgb 'red' title 'Container in bounding box'\n");
fprintf_s(gid, "plot '-' lc rgb 'red' title 'Container in bounding box'\n");


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//draw trajectory
for (int a = 0; a < (int)all_point_cloud_in_a_boundingbox_container.size(); a++)
{
fprintf(gid, "%f\t%f\n", all_point_cloud_in_a_boundingbox_container[a].x, all_point_cloud_in_a_boundingbox_container[a].z);

//fprintf(gid, "%lf\t%lf\n", all_point_cloud_in_a_boundingbox_person[a].x, all_point_cloud_in_a_boundingbox_person[a].z);
//fprintf(gid, "%lf\t%lf\n", all_point_cloud_in_a_boundingbox_transporting[a].x, all_point_cloud_in_a_boundingbox_transporting[a].z);
}



//copyしたフォルダ（ヒストグラムのテスト用）
//draw trajectory
//for (int a = 0; a < (int)all_hist_result_container.size(); a++)
//{
//fprintf(gid, "%lf\t%lf\n", all_hist_result_container[a].x, all_hist_result_container[a].z);

//fprintf(gid, "%lf\t%lf\n", all_hist_result_person[a].x, all_hist_result_person[a].z);
//fprintf(gid, "%lf\t%lf\n", all_hist_result_transporting[a].x, all_hist_result_transporting[a].z);
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





fprintf(gid, "set terminal windows\n");
fprintf(gid, "set output\n");




fprintf(gid, "e\n");
fflush(gid);


if (n_flag_thread_gnuplot_exit == 1)
{

break;
}

Sleep(10);
}


fclose(gid);

_endthreadex(0);

return 0;
}
*/








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













//動画内の動体の差分を求める　+　表示【途中】

/*

int differences_in_the_movie()
{
//動画の読み込み
std::cout << "Movie name = " << std::endl;
std::string movie_name;
std::cin >> movie_name;

VideoCapture cap(movie_name);

//動画の保存
std::cout << "Folder path for saving = " << std::endl;
std::string saving_folder_path;
std::cin >> saving_folder_path;

//保存する際のフレーム数を指定
std::cout << "Number of frames=" << std::endl;
int n;
std::cin >> n;

VideoWriter writer(saving_folder_path + "\\differences_in_the_movie.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), n, Size(672, 376));


// 背景差分計算用オブジェクトの生成


int max_frame = int(cap.get(cv::CAP_PROP_FRAME_COUNT));
int wait_secs = int(1000 / cap.get(cv::CAP_PROP_FPS));


for (int i = 0; i < max_frame; i++)
{
Mat frame, output;
cap >> frame;

if (frame.empty())
{
std::cerr << "Movie is None!" << std::endl;
return -1;
}

cv::imshow("output", output);

}

}

*/





//Histogramming YOLO point clouds in a bounding box（最初に作ったもの）←たぶん，使わない

/*
int histgram_Lidar_YOLO()
{
//One line length
int count_one_line;
//Prepare an array
vector<struct yolo_bounfing_box> yolo_BBox; //for YOLO
struct yolo_bounfing_box yolo_buf_bbox;
vector<struct point_cloud>point_cloud; //for LiDAR
struct point_cloud point;
FILE *text_file;
//Load a text file
std::cout << "Text file = " << std::endl;
std::string text_file_path;
std::cin >> text_file_path;
//open text file to read
fopen_s(&text_file, text_file_path.c_str(), "rt");
//Error Handling
if (text_file == NULL)
{
std::cerr << "Cannot this text file!" << std::endl;
return -1;
}
//Prepare a buffer to get one line at a time
char one_line_BUFFER[YOLO_FILE_LINE_MAX_BUFFER_SIZE];
//
while (1)
{
//
while (1)
{
//If not EOF, then
if (fgets(one_line_BUFFER, YOLO_FILE_LINE_MAX_BUFFER_SIZE, text_file) != NULL)
{
count_one_line = (int)strlen(one_line_BUFFER);

//object名
char *obj1, *obj2, *obj3;
obj1 = strstr(one_line_BUFFER, "person"); //『strstr』で，指定した文字列を検索
obj2 = strstr(one_line_BUFFER, "container");
obj3 = strstr(one_line_BUFFER, "cone");

// Per object  or  Per image
//if (count_one_line > 0 && count_one_line < 30) //Scanning the results of YOLO
//{
//	sscanf_s(one_line_BUFFER, "%s\t%d\t%d\t%d\t%d", &yolo_buf_bbox.name, YOLO_LABEL_STRING_MAX_LENGTH, &yolo_buf_bbox.x, &yolo_buf_bbox.y, &yolo_buf_bbox.width, &yolo_buf_bbox.height);
//	//display
//	printf_s("%s\t%d\t%d\t%d\t%d\n", yolo_buf_bbox.name, yolo_buf_bbox.x, yolo_buf_bbox.y, yolo_buf_bbox.width, yolo_buf_bbox.height);
//}

if (obj1 || obj2 || obj3)
{
sscanf_s(one_line_BUFFER, "%s\t%d\t%d\t%d\t%d", &yolo_buf_bbox.name, YOLO_LABEL_STRING_MAX_LENGTH, &yolo_buf_bbox.x, &yolo_buf_bbox.y, &yolo_buf_bbox.width, &yolo_buf_bbox.height);
//display
printf_s("%s\t%d\t%d\t%d\t%d\n", yolo_buf_bbox.name, yolo_buf_bbox.x, yolo_buf_bbox.y, yolo_buf_bbox.width, yolo_buf_bbox.height);
//yolo_BBox.push_back(yolo_buf_bbox);
}
else //Scanning the results of liDAR
{
sscanf_s(one_line_BUFFER, "%lf\t%lf\t%lf\n", &point.x, &point.y, &point.z);
//display
printf_s("%lf\t%lf\t%lf\n", point.x, point.y, point.z);
//point_cloud.push_back(point);
}
}
else
{
break;
}
} //second while
break;
}//first while
//for (int a = 0; a <= yolo_BBox.size(); a++)
//{
//	a++;
//	std::cout << a << std::endl;
//	sscanf_s(one_line_BUFFER, "%s\t%d\t%d\t%d\t%d", &yolo_BBox[a].name, YOLO_LABEL_STRING_MAX_LENGTH, &yolo_BBox[a].x, &yolo_BBox[a].y, &yolo_BBox[a].width, &yolo_BBox[a].height);
//
//	//display
//	printf_s("%s\t%d\t%d\t%d\t%d\n", yolo_BBox[a].name, yolo_BBox[a].x, yolo_BBox[a].y, yolo_BBox[a].width, yolo_BBox[a].height);
//	for (int b = 0; b <= point_cloud.size(); b++)
//	{
//		sscanf_s(one_line_BUFFER, "%lf\t%lf\t%lf\n", &point_cloud[b].x, &point_cloud[b].y, &point_cloud[b].z);
//		//display
//		printf_s("%lf\t%lf\t%lf\n", point_cloud[b].x, point_cloud[b].y, point_cloud[b].z);
//	}
//}
//
////clear
//yolo_BBox.clear();   yolo_BBox.shrink_to_fit();
//point_cloud.clear(); point_cloud.shrink_to_fit();
printf_s("Finish!\n");
return 0;
}
*/
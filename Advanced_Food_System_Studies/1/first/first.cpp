#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //テキストファイルを扱うために用意
#include <string>
#include <process.h>
#include <time.h>
#include <random>
#include <atltime.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/superres/optical_flow.hpp>
#include <opencv2/video/tracking.hpp>


#include <atltime.h> //処理時間を計算するために用意





//プロトタイプ宣言
int lecture();
int menu_screen(void);
int disply_movie(void);
int tracker_main(void);
int result_RGB_HSV(void);
int face_detection(void);
int diff_camera(void);
int template_matching();
void mouse_callback(int event, int x, int y, int flags, void* userdata);
int hough();
int hough_circle();
int HOG();
int template_matting();
int template_ssd();
int stereo_distance();
int stereo_gnuplot();
unsigned _stdcall thread_gnuplot(void *p);
int stereo_matching();
int photo_hunt();
int meanshift(int argc, char **argv);
int optical_flow();



using namespace cv;
using namespace std;


struct stereo {
	float x;
	float y;
	float z;
};


Rect2i rectangle_value;
int n_flag_thread_gnuplot_exit;
HANDLE hThread_gnuplot;
std::vector<struct stereo> all_xyz;





int main(int argc, char* argv[])
{
	int n = 0; //画像から動画を作成する際のフレーム数
	int nCommand;
	int nFlag;
	int track_code;

	nFlag = 1;

	while (nFlag)
	{
		nCommand=menu_screen();

		switch (nCommand)
		{

			case 0:
				lecture();

				break;



			case 1:
				//動画の保存
				disply_movie();

				break;



			// エラーが出て実行できない
			case 2:
				track_code = tracker_main();

				break;

			case 3:
				//RGB値とHSV値の書き出し
				result_RGB_HSV();

				break;


			case 4:
				//顔検出
				face_detection();


				break;


			case 5:
				diff_camera();

				break;

			case 6:
				template_matching();

				break;


			case 7:
				hough();

				break;

			case 8:
				hough_circle();

				break;

			case 9:
				HOG();

				break;

			case 10:
				template_matting();

				break;

			case 11:

				template_ssd();

				break;

			case 12:
				stereo_distance();

				break;

			case 13:
				stereo_gnuplot();


				break;

			case 14:
				photo_hunt();

				break;

			case 15:
				meanshift(argc, argv);

				break;

			case 16:
				optical_flow();

				break;


			case 99:
				nFlag = -1;
				exit(1);


		}

	}


}





//メニュー画面の関数
int menu_screen()
{
	int ncommand;

	printf("\n");
	printf("----------------------------------------------------\n");
	printf("<<0>>:授業用\n");
	printf("<<1>>:動画の保存\n");
	printf("<<2>>:物体追跡\n");
	printf("<<3>>:RGB値の書き出し\n");
	printf("<<4>>:顔検出\n");
	printf("<<5>>:差分検出\n");
	printf("<<6>>:テンプレートマッチング\n");
	printf("<<7>>:Hough変換\n");
	printf("<<8>>:Hough変換【円ver】\n");
	printf("<<9>>:HOG特徴量\n");
	printf("<<10>>:テンプレートマッチング\n");
	printf("<<11>>:テンプレートマッチング高速化\n");
	printf("<<12>>:ステレオカメラによる距離算出\n");
	printf("<<13>>:ステレオカメラによる距離算出 グラフに表示\n");
	printf("<<14>>:間違い探し\n");
	printf("<<15>>:MeanShiftを使った物体追跡\n");
	printf("<<16>>:オプティカルフローを用いた物体追跡\n");
	printf("<<17>>:○○\n");
	printf("<99>>:終了します．\n");
	printf("----------------------------------------------------\n");
	printf("\n");

	printf("command=");
	scanf_s("%d", &ncommand);

	return ncommand;

}




//授業用
int lecture(void)
{
	


	return 0;
}




//第2回授業(動画の表示+保存)
int disply_movie(void)
{
	int i = 0;
	std::string image_name;
	Mat frame;

	Mat frame_gray;

	VideoCapture cap(0, cv::CAP_DSHOW);

	int w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
	int h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);


	VideoWriter writer("D:\\M1\\Advanced_Food_System_Studies\\1\\movie\\recording.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 10, Size(w, h));

	if (!cap.isOpened()) return -1;


	namedWindow("image", WINDOW_AUTOSIZE);


	while(1)
	{

		cap >> frame;


		//グレイ化



		cv::imshow("image", frame);


		//動画から画像を取り出し，フォルダに保存
		image_name = "D:\\M1\\Advanced_Food_System_Studies\\1\\image\\image" + std::to_string(i) + ".png";
		cv::imwrite(image_name, frame);


		//動画を保存
		writer << frame;


		if (waitKey(30) >= 0) break;



		i++;

	}


	cap.release();//読み込み用のオブジェクトを解放
	writer.release();//書き込み用のオブジェクトを解放
	destroyAllWindows();//ウィンドウを破棄


	return 0;
}





int tracker_main(void)
{
	cv::Ptr<cv::Tracker> tracker = cv::TrackerTLD::create();




	cv::Rect2d roi;





	cv::Scalar color = cv::Scalar(0, 255, 0);
	cv::Mat frame;


	cv::VideoCapture cap(0);
	if (!cap.isOpened())
	{
		std::cout << "Can not open camera!" << std::endl;
		return -1;
	}


	cap >> frame;
	imshow("frame", frame);

	roi = cv::selectROI("frame", frame);
	tracker->cv::Tracker::init(frame, roi);

	cv::destroyWindow("frame");

	while (true) {

		cap >> frame;
		if (frame.empty())
		{
			break;
		}


		//更新
		tracker->cv::Tracker::update(frame, roi);

		//結果を表示
		cv::rectangle(frame, roi, color, 1, 1);

		cv::imshow("tracker", frame);


		if (cv::waitKey(30) >= 0) break;

	}

	cv::destroyWindow("tracker");
	return 0;
}




int result_RGB_HSV(void)
{
	//std::cout << "Hello" << std::endl;

	int i = 0;

	Mat img;
	std::string img_name;
	



	int r;
	int g;
	int b;


	while (1)
	{

		img_name = "D://M1//Advanced_Food_System_Studies//1//image//image" + std::to_string(i) + ".png";

		img = cv::imread(img_name);



		//保存先のテキストファイル
		std::string rgb_reslt = "D://M1//Advanced_Food_System_Studies//1//RGB_result//image" + std::to_string(i) + ".txt";
		std::ofstream of_rgb;
		of_rgb.open(rgb_reslt);


		for (int y = 0; y < img.rows; y++)
		{
			for (int x = 0; x < img.cols; x++)
			{
				b = img.at<cv::Vec3b>(y, x)[0];//B
				g = img.at<cv::Vec3b>(y, x)[1];//G
				r = img.at<cv::Vec3b>(y, x)[2];//R



				//printf("R=%d\tG=%d\tB=%d\n", r, g, b);

				
				of_rgb << "B =" << r << "\t" << "G=" << g << "\t" << "R=" << b << "\n";//テキストファイルに書き出す
			
			
			}
		}


		of_rgb.close();

		std::cout << std::to_string(i) + "番目完了" << std::endl;


		i++;


		if (img.empty())
		{
			break;
		}
	}


	return 0;
}




//第4回
//顔を検出して，ファイルに保存
//http://www.ail.cs.gunma-u.ac.jp/ailwiki/index.php?Haar-like%E7%89%B9%E5%BE%B4%E9%87%8F%E3%82%92%E7%94%A8%E3%81%84%E3%81%9F%E3%82%AB%E3%82%B9%E3%82%B1%E3%83%BC%E3%83%89%E5%88%86%E9%A1%9E%E5%99%A8%E3%81%AB%E3%82%88%E3%82%8B%E5%89%8D%E6%96%B9%E8%BB%8A%E4%B8%A1%E3%81%AE%E8%AD%98%E5%88%A5

int face_detection(void)
{
	int q = 0;
	std::string image_name;
	Mat frame;
	Mat frame_gray;
	Mat frame_rect;


	int x_start = 0;
	int y_start = 0;
	int x_end = 0;
	int y_end = 0;
	int width = 0;
	int height = 0;



	VideoCapture cap(0, cv::CAP_DSHOW);
	if (!cap.isOpened()) return -1;


	//顔認識用のカスケードを用意
	std::string cascade_path = "C:\\opencv-3.4.10\\opencv-3.4.10\\sources\\samples\\winrt\\FaceDetection\\FaceDetection\\Assets\\haarcascade_frontalface_alt.xml";
	cv::CascadeClassifier cascade;
	cascade.load(cascade_path);


	namedWindow("image", WINDOW_AUTOSIZE);
	//namedWindow("image_rect", WINDOW_AUTOSIZE);
	



	while (1)
	{

		cap >> frame;


		// グレイスケール
		cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);


		
		std::vector<cv::Rect> faces;

		// frame_grayから顔を探す
		cascade.detectMultiScale(frame_gray, faces);


		for (int i = 0; i < faces.size(); i++)
		{

			x_start = faces[i].x;
			y_start = faces[i].y;
			x_end = faces[i].x + faces[i].width;
			y_end = faces[i].y + faces[i].height;
			width = faces[i].width;;
			height = faces[i].height;

			// 赤い長方形を frame に描画
			cv::rectangle(frame, cv::Point(x_start, y_start), cv::Point(x_end, y_end), cv::Scalar(0, 0, 255), 2);


			// 顔のみを取り出す
			frame_rect = Mat(frame, Rect(x_start, y_start, width, height));

			
		}



		if (!frame.empty())
		{
			// frameを表示
			cv::imshow("image", frame);

			if (!frame_rect.empty())
			{
				//リサイズ
				cv::resize(frame_rect, frame_rect, Point(frame.cols, frame.rows));

				// frame_rectを表示
				//cv::imshow("image_rect", frame_rect);

				//動画から画像を取り出し，フォルダに保存
				image_name = "D:\\M1\\Advanced_Food_System_Studies\\1\\face_image\\image" + std::to_string(q) + ".png";
				cv::imwrite(image_name, frame_rect);
			}
		
		}
		else
		{
			continue;
		}


		if (waitKey(30) >= 0) break;



		q++;

	}


	cap.release();//読み込み用のオブジェクトを解放
	cv::destroyAllWindows();//ウィンドウを破棄

	return 0;
}






//第3回授業(物体追跡（フレーム間差分）)
//https://code-database.com/knowledges/116

int diff_camera(void)
{
	Mat frame;
	Mat gray_frame1, gray_frame2, gray_frame3;
	Mat diff1, diff2;
	Mat diff;
	Mat thres1, thres2, thres3;
	vector< vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	Point temp;
	int temp_x, temp_y;
	int x_min, x_max, y_min, y_max;


	cv::Scalar green = cv::Scalar(0, 255, 0);

	VideoCapture cap(0, cv::CAP_DSHOW);
	if (!cap.isOpened())return -1;


	//3フレーム分読み込む
	cap >> frame;
	cv::cvtColor(frame, gray_frame1, cv::COLOR_BGR2GRAY);//グレイスケール化
	
	cap >> frame;
	cv::cvtColor(frame, gray_frame2, cv::COLOR_BGR2GRAY);

	cap >> frame;
	cv::cvtColor(frame, gray_frame3, cv::COLOR_BGR2GRAY);
	



	cout << "start" << endl;

	while (1)
	{
		//フレームの差を求める
		cv::absdiff(gray_frame1, gray_frame2, diff1);
		cv::absdiff(gray_frame2, gray_frame3, diff2);

		//差分1と差分2の結果を比較（論理積）
		cv::bitwise_and(diff1, diff2, diff);

		cv::namedWindow("diff image", cv::WINDOW_AUTOSIZE);
		imshow("diff image", diff);


		//輪郭を抽出
		cv::findContours(diff, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		
		
		for (int i = 0; i < contours.size(); i++) {
			temp = contours[i][0];
			x_min = temp.x;
			x_max = temp.x;
			y_min = temp.y;
			y_max = temp.y;

			for (int j = 1; j < contours[i].size(); j++) {
				temp = contours[i][j];
				temp_x = temp.x;
				temp_y = temp.y;

				if (temp_x < x_min)x_min = temp_x;
				if (temp_x > x_max)x_max = temp_x;
				if (temp_x < y_min)y_min = temp_y;
				if (temp_x > y_max)y_max = temp_y;
			}

			//小さな差分は取り除く
			int w = x_max - x_min;
			int h = y_max - y_min;
			if (w > 30 && h > 30)
			{
				cv::rectangle(frame, Point(x_min, y_min), Point(x_max, y_max), green, 2); //四角で囲む
			}


		}


		//表示
		cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
		cv::imshow("result", frame);


		//画像を1フレームずらす
		gray_frame2.copyTo(gray_frame1, gray_frame2); //フレーム2をフレーム1へ
		gray_frame3.copyTo(gray_frame2, gray_frame3); //フレーム3をフレーム2へ
		cap >> frame;
		cv::cvtColor(frame, gray_frame3, cv::COLOR_BGR2GRAY);
		


		if (waitKey(30) >= 0) break;
	}


	cap.release();//読み込み用のオブジェクトを解放
	cv::destroyAllWindows();//ウィンドウを破棄


	return 0;
}



//https://code-database.com/knowledges/116

//int diff_camera(void)
//{
//	Mat frame;
//	Mat gray_frame1, gray_frame2, gray_frame3;
//	Mat diff1, diff2;
//	Mat diff;
//	Mat diff_out;
//	vector< vector<cv::Point> > contours;
//	vector<cv::Vec4i> hierarchy;
//	Point temp;
//	int temp_x, temp_y;
//	int x_min, x_max, y_min, y_max;
//
//	cv::Rect box;
//
//	cv::Scalar green = cv::Scalar(0, 255, 0);
//
//	VideoCapture cap(0, cv::CAP_DSHOW);
//	if (!cap.isOpened())return -1;
//
//
//	//3フレーム分読み込む
//	cap >> frame;
//	cv::cvtColor(frame, gray_frame1, cv::COLOR_BGR2GRAY);
//
//	waitKey(30);
//
//	cap >> frame;
//	cv::cvtColor(frame, gray_frame2, cv::COLOR_BGR2GRAY);
//	waitKey(30);
//
//	cap >> frame;
//	cv::cvtColor(frame, gray_frame3, cv::COLOR_BGR2GRAY);
//
//
//
//	cout << "start" << endl;
//
//	while (1)
//	{
//		//フレームの差を求める
//		cv::absdiff(gray_frame1, gray_frame2, diff1);
//		cv::absdiff(gray_frame2, gray_frame3, diff2);
//
//		//差分1と差分2の結果を比較（論理積）
//		cv::bitwise_and(diff1, diff2, diff);
//
//		cv::threshold(diff, diff_out, 4, 255, CV_THRESH_BINARY);
//		imshow("diff", diff);
//		imshow("diff_out", diff_out);
//
//		dilate(diff_out, diff_out, 3,Point(-1,-1),3);
//
//		//輪郭を抽出
//		cv::findContours(diff_out, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//
//		//差分があった点を画像に描写
//		cout << contours.size() << endl;
//		for (int i = 0; i < contours.size(); i++){
//			temp = contours[i][0];
//			x_min = temp.x;
//			x_max = temp.x;
//			y_min = temp.y;
//			y_max = temp.y;
//
//			for (int j = 1; j < contours[i].size(); j++) {
//				temp  = contours[i][j];
//				temp_x = temp.x;
//				temp_y = temp.y;
//
//				if (temp_x < x_min)x_min = temp_x;
//				if (temp_x > x_max)x_max = temp_x;
//				if (temp_x < y_min)y_min = temp_y;
//				if (temp_x > y_max)y_max = temp_y;
//
//				//	cv::rectangle(frame, cv::Point(contours[i].x, box.y), cv::Point(box.x+box.width, box.y+box.height), green, 2);
//				
//			}
//			cv::rectangle(frame, Point(x_min,y_min), Point(x_max,y_max), green, 2);
//		}
//		
//		//表示
//		cv::namedWindow("diff image", cv::WINDOW_NORMAL);
//		cv::imshow("diff image", frame);
//
//		//画像を1フレームずらす
//		gray_frame2.copyTo(gray_frame1, gray_frame2);
//		gray_frame3.copyTo(gray_frame2, gray_frame3);
//		cap >> frame;
//		cv::cvtColor(frame, gray_frame3, cv::COLOR_BGR2GRAY);
//
//
//		contours.clear();
//		contours.shrink_to_fit();
//		if (waitKey(30) >= 0) break;
//	}
//
//
//	cap.release();//読み込み用のオブジェクトを解放
//	cv::destroyAllWindows();//ウィンドウを破棄
//
//
//	return 0;
//}







//第5回授業×
//テンプレートマッチング(動画では上手くいかない)
//https://qiita.com/appin/items/0bb42ef108b49e9f72c3
//https://qiita.com/satsukiya/items/c8828f48be7673007007 
//http://imgprolab.sys.fit.ac.jp/~yama/imgproc/proc/Document_OpenCVforC_8_2017.pdf (←参考記事)

int template_matching()
{
	bool start_capture = false;


	vector<Rect2i> all_rectangle_value;

	Mat img;
	Mat result_img;
	
	VideoCapture cap(0);

	cap >> img;

	Mat draw_img = img.clone();
	string window_name = "example";
	bool isClick = false;
	int key;
	int key2;

	imshow(window_name, img);
	setMouseCallback(window_name, mouse_callback, &isClick);
	for (;;) {
		key = 0;

		// 左ボタンが押されたら描画開始
		if (isClick == true) {
			rectangle(draw_img, rectangle_value, Scalar(255, 0, 0), 3, CV_AA);
		}

		imshow(window_name, draw_img);
		draw_img = img.clone();

		// qキーが押されたら終了
		key = waitKey(30);
		if (key > 0)
			break;
	}

	destroyWindow("example");

	start_capture = true;

	

	if (start_capture == true) {
		Mat last_roi_image(img, Rect(rectangle_value.x, rectangle_value.y, rectangle_value.width, rectangle_value.height));


		while (1) {

			cap >> img;

			//テンプレートマッチング
			cv::matchTemplate(img, last_roi_image, result_img, CV_TM_CCOEFF_NORMED);
			Point max_pt;
			double maxVal;
			cv::minMaxLoc(result_img, NULL, &maxVal, NULL, &max_pt);

			//探索結果の場所に矩形を描画
			Rect roi_rect(0, 0, rectangle_value.width, rectangle_value.height);
			roi_rect.x = max_pt.x;
			roi_rect.y = max_pt.y;
			cv::rectangle(img, roi_rect, Scalar(0, 255, 255), 2);

			//次フレームでのテンプレートマッチング用のROIを保存
			Mat roi_image(img, Rect(rectangle_value.x, rectangle_value.y, rectangle_value.width, rectangle_value.height));
			last_roi_image = roi_image.clone();

			namedWindow("result", WINDOW_AUTOSIZE);
			cv::imshow("result", img);
			key2 = waitKey(30);
			if (key2 >= 0) {
				break;
			}
		}

	}


	cap.release();
	cv::destroyAllWindows();


	return 0;
}



//コールバック関数（マウスで操作）
void  mouse_callback(int event, int x, int y, int flags, void* userdata)
{
	bool *isClick = static_cast<bool *>(userdata);

	if (event == EVENT_LBUTTONDOWN) {
		*isClick = true;
		cout << "Draw rectangle\n"
			<< " start position (x, y) : " << x << ", " << y << endl;

		rectangle_value = Rect2i(x, y, 0, 0);
	}
	if (event == EVENT_LBUTTONUP) {
		*isClick = false;
		cout << " end   position (x, y) : " << x << ", " << y << endl;
		cout << " width and height : " << rectangle_value .width<< ", " << rectangle_value.height << endl;

		rectangle_value.width = x - rectangle_value.x;
		rectangle_value.height = y - rectangle_value.y;
	}
	if (event == EVENT_MOUSEMOVE) {
		if (*isClick) {
			rectangle_value.width = x - rectangle_value.x;
			rectangle_value.height = y - rectangle_value.y;
		}
	}

}







//第5回授業〇
//Hough変換
//http://maverickproj.web.fc2.com/OpenCV_45.html
//http://whitewell.sakura.ne.jp/OpenCV/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html
//

int hough() {

	int hr = -1;


	cv::Mat src, gray_src, edge;
	std::vector<cv::Vec4i>lines;

	cv::VideoCapture cap("D:\\M1\\Advanced_Food_System_Studies\\1\\road.mp4");


	try {

		while (1) {
			cap >> src;

			//グレイスケール
			cv::cvtColor(src, gray_src, COLOR_BGR2GRAY);

			cv::namedWindow("src", 1);
			cv::imshow("src", src);

			cv::namedWindow("gray", 1);
			cv::imshow("gray", gray_src);

			//輪郭抽出
			cv::Canny(src, edge, 320, 370);

			cv::namedWindow("edge", 1);
			cv::imshow("edge", edge);

			//線分検出
			cv::HoughLinesP(
				edge,// 8ビット，シングルチャンネルの2値入力画像
				lines, // 検出された線分が出力されるベクトル
				1,// ピクセル単位での距離分解能
				CV_PI / 180.0,// ラジアン単位での角度分解能
				80,// 閾値.thresholdを十分に超えている直線のみが出力対象
				100,// 最小の線分長
				10 // 2点が同一線分上にあると見なす場合に許容される最大距離
			);

			//線分描写
			for (int i = 0; i < lines.size(); i++) {
				cv::line(src, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 2);

				//printf("%d\t%d\t%d\t%d\n", lines[i][0], lines[i][1], lines[i][2], lines[i][3]);
			}


			cv::namedWindow("Hough", 1);
			cv::imshow("Hough", src);



			//4画面表示






			int key=waitKey(30);
			if (key > 0) {
				break;
			}



			hr = 0;
		}


	}
	catch (cv::Exception ex) {
		std::cout << ex.err << std::endl;
	}



	cap.release();
	cv::destroyAllWindows();

	return hr;
}







//第6回授業
//https://teratail.com/questions/130841
//https://so-zou.jp/software/tech/library/opencv/retrieval/hough-transform.htm
//https://tutorialmore.com/questions-1585722.htm
//http://beetreehitsuji.hatenablog.com/entry/2017/02/02/175838

int hough_circle() {

	Mat src, src_clone, gray, hsv;
	vector<Vec3f> circles;

	cv::VideoCapture cap("D:\\M1\\Advanced_Food_System_Studies\\1\\orange_image\\orange_movie.mp4");

	//src = imread("D:\\M1\\Advanced_Food_System_Studies\\1\\orange_image\\orange_input.jpg");



	while (1) {

		cap >> src;

		if (src.empty()) {
			break;
		}

		src_clone = src.clone();

		cvtColor(src_clone, hsv, CV_BGR2HSV); //hsvに変換

		//hsv分解
		std::vector<cv::Mat>mat_channels;
		cv::split(hsv, mat_channels);

		//取り出す色を決定 0 - 30 まで
		int hue_min = 0;
		int hue_max = hue_min + 30;

		int hsv_value;
		int cols = hsv.cols;
		int rows = hsv.rows;

		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				hsv_value = static_cast<int>(mat_channels[0].at<uchar>(row, col));

				//色相(Hue)
				if (!((hue_min <= hsv_value) && (hsv_value <= hue_max))) {
					//RGB値を0にする
					src_clone.at<cv::Vec3b>(row, col)[0] = 0; //B
					src_clone.at<cv::Vec3b>(row, col)[1] = 0; //G
					src_clone.at<cv::Vec3b>(row, col)[2] = 0; //R
				}
			}
		}



		cvtColor(src_clone, gray, CV_BGR2GRAY); //グレイスケール

		GaussianBlur(gray, gray, Size(9, 9), 2, 2); //ノイズ除去



		cv::HoughCircles(
			gray,                 // 入力画像
			circles,              // 検出された線を格納する領域
			CV_HOUGH_GRADIENT,    // ハフ変換の種類
			1,                    // 円の中心を求める計算の解像度
			60,                   // 中心座標間の最小間隔
			200,                  // 1番目のパラメータ
			20,                   // 2番目のパラメータ
			0,                    // 検出される円の最小半径
			60);                  //検出される円の最大半径


		for (int i = 0; i < circles.size(); i++) {

			Point center(cvRound(circles[i][0]), cvRound(circles[i][1])); //中心の座標
			int radius = cvRound(circles[i][2]);  //半径

			cv::circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);//中心
			cv::circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);//円周

			std::cout << "center: " << center << "\nradius: " << radius << endl;
		}



		/*namedWindow("src_clone", WINDOW_AUTOSIZE);
		imshow("src_clone", src_clone);*/
		namedWindow("Hough Circle", WINDOW_AUTOSIZE);
		imshow("Hough Circle", src);



		int key = waitKey(30);
		if (key >= 0) {
			break;
		}



	}


	cap.release();
	cv::destroyAllWindows();
	return 0;

}









//第7回授業
//HOG特徴量
//https://qiita.com/kenmaro/items/1b5f23187cc46b4f28c0
//https://algorithm.joho.info/image-processing/hog-feature-value/

int HOG() {
	
	cv::Mat img, gray_img;
	std::vector<cv::Rect> found;

	
	cv::VideoCapture cap("D:\\Experiments\\2020_LiDAR_ZED_FUSION\\output_data\\image_to_movie_20201106_Container_around_workshop\\out_cam1_remap.mov");
	//cv::VideoCapture cap("D:\\Experiments\\2020_LiDAR_ZED_FUSION\\output_data\\image_to_movie_20201223112457\\movie_stereo_cam1.mov");

	if (!cap.isOpened()) {
		std::cout << "There is no image" << std::endl;
		return -1;
	}


	while (1) {
		cap >> img;

		if (img.empty()) {
			break;
		}

		cv::cvtColor(img, gray_img, CV_BGR2GRAY);

		cv::HOGDescriptor hog;
		//hog.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
		hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
		hog.detectMultiScale(gray_img, found);



		for (int i = 0; i < found.size(); i++) {

			cv::Rect r = found[i];
			rectangle(img, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
		}


		namedWindow("HOG", WINDOW_AUTOSIZE);
		imshow("HOG", img);

		int k = cvWaitKey(30);
		if (k >= 0) {
			break;
		}

	}



	cap.release();
	destroyAllWindows();



	return 0;

}







double IoU(cv::Point2i a_tl, cv::Point2i a_br, cv::Point2i b_tl, cv::Point2i b_br) {

	int dx1, dy1, dx2, dy2;
	int dx, dy;
	int d;

	double iou;


	if (a_tl.x <= b_tl.x) {
		dx1 = b_tl.x;
	}
	else {
		dx1 = a_tl.x;
	}

	if (a_tl.x + a_br.x <= b_tl.x + b_br.x) {
		dx2 = a_tl.x + a_br.x;
	}
	else {
		dx2 = b_tl.x + b_br.x;
	}


	if (a_tl.y <= b_tl.y) {
		dy1 = b_tl.y;
	}
	else {
		dy1 = a_tl.y;
	}


	if (a_tl.y + a_br.y <= b_tl.y + b_br.y) {
		dx2 = a_tl.y + a_br.y;
	}
	else {
		dx2 = b_tl.y + b_br.y;
	}



	dx = dx2 - dx1;
	dy = dy2 - dy1;

	d = dx*dy;


	iou = (double)d / (double)((a_br.x*a_br.y + b_br.x*b_br.y - d));


	return iou;

}








//第8回授業
//http://imgprolab.sys.fit.ac.jp/~yama/imgproc/proc/Document_OpenCVforC_8_2017.pdf


int template_matting() {
	CFileTime cTimeStart, cTimeEnd;
	CFileTimeSpan cTimeSpan;
	cTimeStart = CFileTime::GetCurrentTime();           // 現在時刻

	double r;
	Point pos;
	Rect roi;

	cv::VideoCapture cap("D:\\M1\\Advanced_Food_System_Studies\\1\\image_6_16.mp4");
	

	Mat image;
	string image_folder = "D:\\M1\\Advanced_Food_System_Studies\\1\\";
	Mat template_image;
	string template_image_name = "template_image.png";

	template_image = cv::imread(image_folder + template_image_name);

	Mat limage;
	Mat dst;

	

	while (1) {

		cap >> image;
		if (image.empty()) {
			break;
		}

		double fMin = 20000.0;

		int rows = image.rows - template_image.rows + 1;
		int cols = image.cols - template_image.cols + 1;


		for (int y = 0; y < rows; y++) {
			for (int x = 0; x < cols; x++) {

				roi = Rect(x, y, template_image.cols, template_image.rows);
				limage = Mat(image, roi); //元の画像から，テンプレート画像の範囲のみを取り出す

				cv::absdiff(limage, template_image, dst);//テンプレート画像と比較する
				r = sum(dst)[0];//差分画像の1つの色のみの合計を求める


				if (fMin > r) {
					fMin = r;
					pos.x = x;
					pos.y = y;
				}
			}
		}


		//printf("%lf\n", fMin);

		if (fMin < 20000) {
			cv::rectangle(image, Rect(pos.x, pos.y, template_image.cols, template_image.rows), Scalar(255, 255, 255), 2);
		}

		cv::namedWindow("result", WINDOW_AUTOSIZE);
		cv::imshow("result", image);

		cv::imshow("template_image", template_image);
		cv::namedWindow("template_image", WINDOW_AUTOSIZE);

		if (cv::waitKey(30) >= 0) {
			break;
		}

		

	}

	cap.release();
	cv::destroyAllWindows();



	cTimeEnd = CFileTime::GetCurrentTime();           // 現在時刻
	cTimeSpan = cTimeEnd - cTimeStart;
	std::cout << "処理時間:" << cTimeSpan.GetTimeSpan() / 10000000 << "[s]" << std::endl;

	return 0;

}




//int template_matting() {
//
//	double r;
//	Point pos;
//	Rect roi;
//
//	Mat image;
//	string image_folder = "D:\\M1\\Advanced_Food_System_Studies\\1\\";
//	string image_name = "image669.png";
//
//	Mat template_image;
//	string template_image_name = "template_image.png";
//
//	image = cv::imread(image_folder + image_name);
//	template_image = cv::imread(image_folder + template_image_name);
//
//	Mat limage;
//	Mat dst;
//
//	double rMin = 10000.0;
//
//	int rows = image.rows - template_image.rows + 1;
//	int cols = image.cols - template_image.cols + 1;
//
//
//	for (int y = 0; y < rows; y++) {
//		for (int x = 0; x < cols; x++) {
//
//			roi = Rect(x, y, template_image.cols, template_image.rows);
//			limage = Mat(image, roi); //元の画像から，テンプレート画像の範囲のみを取り出す
//
//			cv::absdiff(limage, template_image, dst);//テンプレート画像と比較する
//			r = sum(dst)[0];
//
//			if (rMin > r) {
//				pos.x = x;
//				pos.y = y;
//			}
//		}
//	}
//
//
//	cv::rectangle(image, Rect(pos.x, pos.y, template_image.cols, template_image.rows), Scalar(255, 255, 255), 2);
//
//	cv::namedWindow("result", WINDOW_AUTOSIZE);
//	cv::namedWindow("template_image", WINDOW_AUTOSIZE);
//
//	cv::imshow("result", image);
//	cv::imshow("template_image", template_image);
//
//
//	cv::waitKey(0);
//
//	return 0;
//
//}







//第9回
//https://www.robonchu.info/entry/2018/05/12/155455
//http://imgprolab.sys.fit.ac.jp/~yama/imgproc/lec/imgproc_patternrecog_2.pdf


//テンプレートマッチング高速化
int template_ssd() {
	CFileTime cTimeStart, cTimeEnd;
	CFileTimeSpan cTimeSpan;
	cTimeStart = CFileTime::GetCurrentTime();           // 現在時刻
	
	double r = 0;
	Point pos;
	Rect roi;

	cv::VideoCapture cap("D:\\M1\\Advanced_Food_System_Studies\\1\\image_6_16.mp4");

	int box_num = 0;
	int frame_count = 0;

	Mat image;
	string image_folder = "D:\\M1\\Advanced_Food_System_Studies\\1\\";
	Mat template_image;
	string template_image_name = "template_image.png";

	template_image = cv::imread(image_folder + template_image_name);

	Mat limage;
	Mat dst;




	while (1) {

		cap >> image;
		if (image.empty()) {
			break;
		}

		int break_frag = 0;
		//double fMin = 20000.0;
		double fMin;

		int rows = image.rows - template_image.rows + 1;
		int cols = image.cols - template_image.cols + 1;

		if (frame_count > 0) {
			fMin = fMin + 1500.0;
		}

		for (int y = 0; y < rows; y++) {

			for (int x = 0; x < cols; x++) {

				roi = Rect(x, y, template_image.cols, template_image.rows);
				limage = Mat(image, roi); //元の画像から，テンプレート画像の範囲のみを取り出す

				cv::absdiff(limage, template_image, dst); //テンプレート画像と比較する

				if (box_num == 0) {
					r = sum(dst)[0];
					fMin = r;
				}
				

				if (box_num > 0) {

					for (int j = 0; j < dst.rows; j++) {
						for (int i = 0; i < dst.cols; i++) {

							r += dst.at<Vec3b>(j, i)[0];
							//printf("%lf\n", r);

							if (r > fMin) {
								//printf("%lf\n",fMin);
								break_frag = 1;
								break;
							}
						}

						if (break_frag == 1) {
							//printf("Break\n");
							break;
						}

					}

				}
				

				//printf("%lf\n", break_frag);


				break_frag = 0;
				//printf("%lf\n",r);


				if (fMin >= r) {
					fMin = r;
					pos.x = x;
					pos.y = y;
				}

				r = 0.0;
				box_num++;
			}
		}


		//printf("%lf\n", fMin);

		if (fMin < 12000.0) {
			cv::rectangle(image, Rect(pos.x, pos.y, template_image.cols, template_image.rows), Scalar(255, 255, 255), 2);
		}


		cv::namedWindow("result", WINDOW_AUTOSIZE);
		cv::imshow("result", image);

		cv::imshow("template_image", template_image);
		cv::namedWindow("template_image", WINDOW_AUTOSIZE);

		if (cv::waitKey(30) >= 0) {
			break;
		}


		frame_count++;

	}

	cap.release();
	cv::destroyAllWindows();



	cTimeEnd = CFileTime::GetCurrentTime();           // 現在時刻
	cTimeSpan = cTimeEnd - cTimeStart;
	std::cout << "処理時間:" << cTimeSpan.GetTimeSpan() / 10000000 << "[s]" << std::endl;

	return 0;
}







//第10回
//https://toragi.cqpub.co.jp/Portals/0/backnumber/2019/03/p047.pdf
//http://whitewell.sakura.ne.jp/OpenCV/py_tutorials/py_calib3d/py_depthmap/py_depthmap.html
//https://cvtech.cc/centroid/
//https://plant-raspberrypi3.hatenablog.com/entry/2018/11/13/185057

int stereo_distance() {
	
	float T = 12; //カメラ間の距離
	float F = 255; //焦点距離
	//float Z = 60; //物体距離
	float D;//ピクセル誤差

	//推定後
	float f;
	float z;


	cv::VideoCapture cap(0);

	if (!cap.isOpened()) {
		return -1;
	}

	cv::Mat image;
	cv::Mat left_image, right_image;
	cv::Mat left_image_gray, right_image_gray;
	cv::Mat mask1, mask2;
	


	//顔認識用のカスケードを用意
	std::string cascade_path = "C:\\opencv-3.4.10\\opencv-3.4.10\\sources\\samples\\winrt\\FaceDetection\\FaceDetection\\Assets\\haarcascade_frontalface_alt.xml";
	cv::CascadeClassifier cascade;
	cascade.load(cascade_path);

	std::vector<cv::Rect> left_faces;
	std::vector<cv::Rect> right_faces;

	int x_start_left = 0;
	int y_start_left = 0;
	int x_end_left = 0;
	int y_end_left = 0;
	int width_left = 0;
	int height_left = 0;

	int x_start_right = 0;
	int y_start_right = 0;
	int x_end_right = 0;
	int y_end_right = 0;
	int width_right = 0;
	int height_right = 0;

	//重心
	cv::Point2f pt1, pt2;


	while(1) {
		cap >> image;

		left_image = image(cv::Rect(0, 0, image.cols / 2, image.rows));
		right_image = image(cv::Rect(image.cols / 2, 0, image.cols / 2, image.rows));

		// グレイスケール
		cv::cvtColor(left_image, left_image_gray, cv::COLOR_BGR2GRAY);
		cv::cvtColor(right_image, right_image_gray, cv::COLOR_BGR2GRAY);


		cascade.detectMultiScale(left_image_gray, left_faces);
		cascade.detectMultiScale(right_image_gray, right_faces);



		if (left_faces.size() > 0 && right_faces.size() > 0) {

			for (int i = 0; i < left_faces.size(); i++) {

				x_start_left = left_faces[i].x;
				y_start_left = left_faces[i].y;
				x_end_left = left_faces[i].x + left_faces[i].width;
				y_end_left = left_faces[i].y + left_faces[i].height;
				width_left = left_faces[i].width;;
				height_left = left_faces[i].height;


				x_start_right = right_faces[i].x;
				y_start_right = right_faces[i].y;
				x_end_right = right_faces[i].x + right_faces[i].width;
				y_end_right = right_faces[i].y + right_faces[i].height;
				width_right = right_faces[i].width;;
				height_right = right_faces[i].height;

				// 赤い長方形を frame に描画
				cv::rectangle(left_image, cv::Point(x_start_left, y_start_left), cv::Point(x_end_left, y_end_left), cv::Scalar(0, 0, 255), 2);
				cv::rectangle(right_image, cv::Point(x_start_right, y_start_right), cv::Point(x_end_right, y_end_right), cv::Scalar(0, 0, 255), 2);


				//重心を求める
				//左
				pt1 = cv::Point2f(x_start_left + (1.0 / 2.0)*width_left, y_start_left + (1.0 / 2.0)*height_left);
				pt2 = cv::Point2f(x_start_right + (1.0 / 2.0)*width_right, y_start_right + (1.0 / 2.0)*height_right);

				//重心を描写
				cv::circle(left_image, pt1, 5, cv::Scalar(100), 2, 4);
				cv::circle(right_image, pt2, 5, cv::Scalar(100), 2, 4);

				//計算
				D = pt1.x - pt2.x;//ピクセル誤差

				//焦点距離
				//f = Z*D / T;

				//推定距離
				z = F*T/D;

				//std::cout << D << std::endl; //ピクセル誤差
				//std::cout << f << std::endl; //標準出力
				std::cout << int(z) << " cm"<<  std::endl; //標準出力
			}

		}
		

		if (!image.empty()) {
			cv::namedWindow("left_image", WINDOW_AUTOSIZE);
			cv::imshow("left_image", left_image);
			cv::namedWindow("right_image", WINDOW_AUTOSIZE);
			cv::imshow("right_image", right_image);

		}
		
		


		int k = cv::waitKey(30);
		if (k >= 0) {
			break;
		}
	}



	cap.release();
	cv::destroyAllWindows();

	return 0;
}






//第11回
//マルチスレッド（gnuplot）
//https://vislab.jp/hiura/lec/iip/geometry.pdf

int stereo_gnuplot() {

	float T = 12; //カメラ間の距離
	float F = 255; //焦点距離
	//float Z = 60; //物体距離
	float D;//ピクセル誤差

	//推定後
	float f;

	struct stereo xyz;


	cv::VideoCapture cap(0);

	if (!cap.isOpened()) {
		return -1;
	}

	cv::Mat image;
	cv::Mat left_image, right_image;
	cv::Mat left_image_gray, right_image_gray;
	cv::Mat mask1, mask2;



	//顔認識用のカスケードを用意
	std::string cascade_path = "C:\\opencv-3.4.10\\opencv-3.4.10\\sources\\samples\\winrt\\FaceDetection\\FaceDetection\\Assets\\haarcascade_frontalface_alt.xml";
	cv::CascadeClassifier cascade;
	cascade.load(cascade_path);

	std::vector<cv::Rect> left_faces;
	std::vector<cv::Rect> right_faces;

	int x_start_left = 0;
	int y_start_left = 0;
	int x_end_left = 0;
	int y_end_left = 0;
	int width_left = 0;
	int height_left = 0;

	int x_start_right = 0;
	int y_start_right = 0;
	int x_end_right = 0;
	int y_end_right = 0;
	int width_right = 0;
	int height_right = 0;

	//重心
	cv::Point2f pt1, pt2;


	n_flag_thread_gnuplot_exit = 0;
	hThread_gnuplot = (HANDLE)_beginthreadex(NULL, 0, thread_gnuplot, NULL, 0, NULL);
	//hThread_gnuplot = (HANDLE)_beginthreadex(NULL, 0, thread_gnuplot, "gnuplot", 0, NULL);



	while (1) {
		cap >> image;

		left_image = image(cv::Rect(0, 0, image.cols / 2, image.rows));
		right_image = image(cv::Rect(image.cols / 2, 0, image.cols / 2, image.rows));

		// グレイスケール
		cv::cvtColor(left_image, left_image_gray, cv::COLOR_BGR2GRAY);
		cv::cvtColor(right_image, right_image_gray, cv::COLOR_BGR2GRAY);


		cascade.detectMultiScale(left_image_gray, left_faces);
		cascade.detectMultiScale(right_image_gray, right_faces);



		if (left_faces.size() > 0 && right_faces.size() > 0) {

			for (int i = 0; i < left_faces.size(); i++) {

				x_start_left = left_faces[i].x;
				y_start_left = left_faces[i].y;
				x_end_left = left_faces[i].x + left_faces[i].width;
				y_end_left = left_faces[i].y + left_faces[i].height;
				width_left = left_faces[i].width;
				height_left = left_faces[i].height;


				x_start_right = right_faces[i].x;
				y_start_right = right_faces[i].y;
				x_end_right = right_faces[i].x + right_faces[i].width;
				y_end_right = right_faces[i].y + right_faces[i].height;
				width_right = right_faces[i].width;;
				height_right = right_faces[i].height;

				// 赤い長方形を frame に描画
				cv::rectangle(left_image, cv::Point(x_start_left, y_start_left), cv::Point(x_end_left, y_end_left), cv::Scalar(0, 0, 255), 2);
				cv::rectangle(right_image, cv::Point(x_start_right, y_start_right), cv::Point(x_end_right, y_end_right), cv::Scalar(0, 0, 255), 2);


				//重心を求める
				//左
				pt1 = cv::Point2f(x_start_left + (1.0 / 2.0)*width_left, y_start_left + (1.0 / 2.0)*height_left);
				pt2 = cv::Point2f(x_start_right + (1.0 / 2.0)*width_right, y_start_right + (1.0 / 2.0)*height_right);

				//重心を描写
				cv::circle(left_image, pt1, 5, cv::Scalar(100), 2, 4);
				cv::circle(right_image, pt2, 5, cv::Scalar(100), 2, 4);

				//計算
				D = pt1.x - pt2.x;//ピクセル誤差

				//焦点距離
				//f = Z*D / T;

				//推定距離
				xyz.z = F*T / D;

				xyz.x = ((pt1.x + pt2.x) / 2.0)*(T / D);
				xyz.y = (pt1.y)*(T / D);

				all_xyz.push_back(xyz);


				//std::cout << D << std::endl; //ピクセル誤差
				//std::cout << f << std::endl; //標準出力
				std::cout << "z = " << int(xyz.z) << " cm" << std::endl; //標準出力
				std::cout << "x = "<< int(xyz.x) << " cm" << std::endl; //標準出力
				//std::cout << int(xyz.y) << " cm" << std::endl; //標準出力
			}

		}


		if (!image.empty()) {
			cv::namedWindow("left_image", WINDOW_AUTOSIZE);
			cv::imshow("left_image", left_image);
			cv::namedWindow("right_image", WINDOW_AUTOSIZE);
			cv::imshow("right_image", right_image);

		}




		int k = cv::waitKey(30);
		if (k >= 0) {
			break;
		}


		if (n_flag_thread_gnuplot_exit == 1) break;

	}


	n_flag_thread_gnuplot_exit = 1; //gnuplot 終了
	CloseHandle(hThread_gnuplot);

	cap.release();
	cv::destroyAllWindows();

	return 0;
}




unsigned _stdcall thread_gnuplot(void *p)
{


	FILE *gid;


	gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");




	fprintf_s(gid, "set size ratio -1\n");
	fprintf(gid, "set xrange[0:100]\n");
	fprintf(gid, "set yrange[0:100]\n");
	fprintf(gid, "set xlabel 'x[cm]'\n");
	fprintf(gid, "set ylabel'z[cm]'\n");


	fflush(gid);




	while (1) {

		fprintf_s(gid, "plot '-' with points ps 1 title 'Trajectory of object movement'\n");

		for (int k = 0; k < (int)all_xyz.size(); k++) {

			fprintf(gid, "%lf\t%lf\n", all_xyz[k].x, all_xyz[k].z);

		}

		fprintf(gid, "e\n");
		fflush(gid);
	

		fflush(gid);


		if (n_flag_thread_gnuplot_exit == 1) break;



		Sleep(10);

	}


	fclose(gid);



	_endthreadex(0);



	return 0;

}







//ステレオマッチング
//https://www.kkaneko.jp/db/pointcloud/stereomatching.html
//https://ichi.pro/fukasa-ii-burokku-matchingu-234857019418386
//https://rest-term.com/archives/1072/

int stereo_matching() {

	cv::VideoCapture cap(0);

	if (!cap.isOpened()) {
		return -1;
	}

	cv::Mat image;
	cv::Mat left_image, right_image;
	cv::Mat left_image_gray, right_image_gray;

	int left_rows;
	int left_cols; 
	int right_rows;
	int right_cols;




	while (1) {
		cap >> image;

		left_image = image(cv::Rect(0, 0, image.cols / 2, image.rows));
		right_image = image(cv::Rect(image.cols / 2, 0, image.cols / 2, image.rows));


		// グレイスケール
		cv::cvtColor(left_image, left_image_gray, cv::COLOR_BGR2GRAY);
		cv::cvtColor(right_image, right_image_gray, cv::COLOR_BGR2GRAY);


		left_rows = left_image.rows;
		left_cols = left_image.cols;
		right_rows = right_image.rows;
		right_rows = right_image.cols;






		cv::namedWindow("left_image", WINDOW_AUTOSIZE);
		cv::imshow("left_image", left_image);
		cv::namedWindow("right_image", WINDOW_AUTOSIZE);
		cv::imshow("right_image", right_image);



		int k = cv::waitKey(30);
		if (k >= 0) {
			break;
		}

	}



	cap.release();
	cv::destroyAllWindows();

	return 0;
}








int photo_hunt() {

	
	string image_path;
	Mat image, left_image, right_image, left_image_clone;
	Mat left_image_gray;
	Mat black;


	cv::Rect rect;

	int rows, cols;
	int b_diff, g_diff, r_diff;
	std::vector<std::vector<cv::Point> > contours;
	

	int threshold = 50;


	image_path = "D:\\M1\\Advanced_Food_System_Studies\\1\\matigai.png";
	image =  cv::imread(image_path, cv::IMREAD_COLOR);
	left_image = image(cv::Rect(0, 0, image.cols / 2, image.rows));
	left_image_clone = left_image.clone();
	right_image = image(cv::Rect(image.cols / 2, 0, image.cols / 2, image.rows));


	black = left_image.clone();

	cvtColor(left_image, left_image_gray, CV_BGR2GRAY); //グレースケールに変換


	rows = left_image.rows;
	cols = left_image.cols;


	for (int j = 0; j < rows; j++) {
		for (int i = 0; i < cols; i++) {
			black.at<cv::Vec3b>(j, i)[0] = 0;
			black.at<cv::Vec3b>(j, i)[1] = 0;
			black.at<cv::Vec3b>(j, i)[2] = 0;
		}
	}
	

	for (int j = 0; j < rows; j++) {
		for (int i = 0; i < cols; i++) {

			b_diff = abs(left_image.at<cv::Vec3b>(j, i)[0] - right_image.at<cv::Vec3b>(j, i)[0]);
			g_diff = abs(left_image.at<cv::Vec3b>(j, i)[1] - right_image.at<cv::Vec3b>(j, i)[1]);
			r_diff = abs(left_image.at<cv::Vec3b>(j, i)[2] - right_image.at<cv::Vec3b>(j, i)[2]);


			if (b_diff > threshold && g_diff > threshold && r_diff > threshold) {

				black.at<cv::Vec3b>(j, i)[0] = 255;
				black.at<cv::Vec3b>(j, i)[1] = 255;
				black.at<cv::Vec3b>(j, i)[2] = 255;


				left_image_clone.at<cv::Vec3b>(j, i)[0] = 0;
				left_image_clone.at<cv::Vec3b>(j, i)[1] = 0;
				left_image_clone.at<cv::Vec3b>(j, i)[2] = 255;

			}
			else {
				continue;
			}
			
		}
	}
	

	cv::cvtColor(black, black, cv::COLOR_BGR2GRAY);

	cv::findContours(black, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);


	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());


	for (int i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}


	for (int i = 0; i < contours.size(); i++) {
		drawContours(left_image_clone, contours_poly, i, Scalar(0,0,255), 1, 1, vector<Vec4i>(), 0, Point());
		rectangle(left_image_clone, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2, 8, 0);
	}




	cv::namedWindow("photo hunt", WINDOW_AUTOSIZE);
	cv::imshow("photo hunt", left_image_clone);
	cv::namedWindow("photo hunt black", WINDOW_AUTOSIZE);
	cv::imshow("photo hunt black", black);
	cv::namedWindow("left image", WINDOW_AUTOSIZE);
	cv::imshow("left image", left_image); 
	cv::namedWindow("right image", WINDOW_AUTOSIZE);
	cv::imshow("right image", right_image);
	cv::waitKey(0);

	return 0;
}








//第12回
//https://www.robonchu.info/entry/2017/08/20/130631
//http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_video/py_meanshift/py_meanshift.html
//https://github.com/gishi523/simple-meanshift-traker
//https://docs.opencv.org/master/d7/d00/tutorial_meanshift.html
//↑Meanshift と Camshift

//http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html
//↑オプティカルフロー


int meanshift(int argc, char **argv) {


	//const string about =
	//	"This sample demonstrates the meanshift algorithm.\n"
	//	"The example file can be downloaded from:\n"
	//	"  https://www.bogotobogo.com/python/OpenCV_Python/images/mean_shift_tracking/slow_traffic_small.mp4";
	//const string keys =
	//	"{ h help |      | print this help message }"
	//	"{ @image |<none>| path to image file }";
	//CommandLineParser parser(argc, argv, keys);
	//parser.about(about);
	//if (parser.has("help"))
	//{
	//	parser.printMessage();
	//	return 0;
	//}
	//string filename = parser.get<string>("@image");
	//if (!parser.check())
	//{
	//	parser.printErrors();
	//	return 0;
	//}


	//VideoCapture capture(filename);
	VideoCapture capture(0);

	if (!capture.isOpened()) {
		//error in opening the video input
		cerr << "Unable to open file!" << endl;
		return 0;
	}
	Mat frame, roi, hsv_roi, mask;
	// take first frame of the video
	capture >> frame;
	// setup initial location of window
	Rect track_window(300, 200, 100, 50); // simply hardcoded the values
	

	// set up the ROI for tracking
	roi = frame(track_window);
	cvtColor(roi, hsv_roi, COLOR_BGR2HSV);
	inRange(hsv_roi, Scalar(0, 60, 32), Scalar(180, 255, 255), mask);
	float range_[] = { 0, 180 };
	const float* range[] = { range_ };
	Mat roi_hist;
	int histSize[] = { 180 };
	int channels[] = { 0 };
	calcHist(&hsv_roi, 1, channels, mask, roi_hist, 1, histSize, range);
	normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX); //正規化
	// Setup the termination criteria, either 10 iteration or move by atleast 1 pt
	TermCriteria term_crit(TermCriteria::EPS | TermCriteria::COUNT, 10, 1);


	while (true) 
	{
		Mat hsv, dst;
		capture >> frame;
		if (frame.empty())
			break;
		cvtColor(frame, hsv, COLOR_BGR2HSV);
		calcBackProject(&hsv, 1, channels, roi_hist, dst, range);
		// apply meanshift to get the new location
		cv::meanShift(dst, track_window, term_crit);
		// Draw it on image
		rectangle(frame, track_window, 255, 2);

		
		imshow("img2", frame);
		imshow("img3", dst);


		int keyboard = waitKey(30);
		if (keyboard >= 0)
		{
			break;
		}
			
	}



	return 0;

}






//オプティカルフローを用いた物体追跡
//https://qiita.com/icoxfog417/items/357e6e495b7a40da14d8
//https://whoopsidaisies.hatenablog.com/entry/2013/12/15/020420
//
int optical_flow() {

	// 動画ファイルの読み込み
	VideoCapture capture = VideoCapture("D:\\M1\\Advanced_Food_System_Studies\\1\\Car_2165.mp4");

	// 前のフレームを保存しておく
	Mat prev;
	capture >> prev;

	// 特徴点格納用
	vector<Point2f> prevCorners;
	vector<Point2f> currCorners;
	vector<uchar> featuresFound;
	vector<float> featuresErrors;

	// 追跡する特徴点を求める
	Mat prevGray;
	cv::cvtColor(prev, prevGray, CV_RGB2GRAY);
	cv::goodFeaturesToTrack(prevGray, prevCorners, 1000, 0.3, 7);
	cv::cornerSubPix(prevGray, prevCorners, Size(10, 10), Size(-1, -1), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 0.03));

	while (1) 
	{

		// 現在のフレームを保存
		Mat curr;
		capture >> curr;

		Mat currGray;
		cv::cvtColor(curr, currGray, CV_RGB2GRAY);

		cv::calcOpticalFlowPyrLK(
			prevGray,
			currGray,
			prevCorners,
			currCorners,
			featuresFound,
			featuresErrors);

		for (int i = 0; i < featuresFound.size(); i++) {
			if (featuresFound[0] == 0 || featuresErrors[i] > 550) {
				continue;
			}

			Point p1 = Point((int)prevCorners[i].x, (int)prevCorners[i].y);
			Point p2 = Point((int)currCorners[i].x, (int)currCorners[i].y);
			cv::line(curr, p1, p2, Scalar(255, 0, 0), 2);
		}

		// 表示
		cv::imshow("input", curr);

		prev = curr;


		int key = cv::waitKey(30);

		if (key >= 0) break;

	}

	return 0;
}







//最終講義
//パーティクルフィルタ
//https://algorithm.joho.info/image-processing/particle-filter-tracking/
//file:///C:/Users/MASUDA~1/AppData/Local/Temp/MicrosoftEdgeDownloads/06de909c-3dc1-46f9-9e74-dbcacdb3cda2/OS10041000011.pdf
//https://qiita.com/chimamedia/items/e6498ee6bcd976cbc5c2
int particle_main()
{

}



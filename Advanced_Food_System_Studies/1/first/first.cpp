#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //テキストファイルを扱うために用意
#include <string>
#include <process.h>
#include <time.h>
#include <random>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>



#define IMG_XSIZE 640;
#define IMG_YSIZE 480;


//プロトタイプ宣言
int lecture();
int menu_screen(void);
int disply_movie(void);
int tracker_main(void);
int result_RGB_HSV(void);
int face_detection(void);
int diff_camera(void);




using namespace cv;
using namespace std;

int main(int argc, const char* argv[])
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
	printf("<<6>>:○○\n");
	printf("<99>>:終了します．\n");
	printf("----------------------------------------------------\n");
	printf("\n");

	printf("command=");
	scanf_s("%d", &ncommand);

	return ncommand;

}





int lecture(void)
{
	//std::cout << "Hello" << std::endl;
	
	int x, y;

	Mat img;
	std::string img_name = "D://M1//Advanced_Food_System_Studies//1//image//image27.png";
	img = cv::imread(img_name);



	int r;
	int g;
	int b;


	for (y = 0; y < img.rows; y++)
	{
		for (x = 0; x < img.cols; x++)
		{
			r = img.at<cv::Vec3b>(y, x)[0];//R
			g = img.at<cv::Vec3b>(y, x)[1];//G
			b = img.at<cv::Vec3b>(y, x)[2];//B

			printf("R=%d\tG=%d\tB=%d\n", r, g, b);
		}
	}



	


	return 0;
}





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

	roi = selectROI("frame", frame);
	tracker->init(frame, roi);

	cv::destroyWindow("frame");

	while (true) {
		cap >> frame;
		if (frame.empty())
		{
			break;
		}



		//更新
		tracker->update(frame, roi);

		//結果を表示
		cv::rectangle(frame, roi, color, 1, 1);

		cv::imshow("tracker", frame);


		if (waitKey(30) >= 0) break;

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
				r = img.at<cv::Vec3b>(y, x)[0];//R
				g = img.at<cv::Vec3b>(y, x)[1];//G
				b = img.at<cv::Vec3b>(y, x)[2];//B



				//printf("R=%d\tG=%d\tB=%d\n", r, g, b);

				
				of_rgb << "R =" << r << "\t" << "G=" << g << "\t" << "B=" << b << "\n";//テキストファイルに書き出す
			
			
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





int face_detection(void)
{
	int i = 0;
	std::string image_name;
	Mat frame;

	Mat frame_gray;



	VideoCapture cap(0, cv::CAP_DSHOW);

	int w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
	int h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

	//VideoWriter writer("D:\\M1\\Advanced_Food_System_Studies\\1\\movie\\recording.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 10, Size(w, h));

	if (!cap.isOpened()) return -1;


	//顔認識用のカスケードを用意
	std::string cascade_path = "C:\\opencv-3.4.10\\opencv-3.4.10\\sources\\samples\\winrt\\FaceDetection\\FaceDetection\\Assets\\haarcascade_frontalface_alt.xml";
	cv::CascadeClassifier cascade;
	cascade.load(cascade_path);


	namedWindow("image", WINDOW_AUTOSIZE);


	while (1)
	{

		cap >> frame;


		//グレイ化
		cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);


		// gray から顔を探して赤い長方形を img に描画
		std::vector<cv::Rect> faces;
		cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

		for (auto face : faces)
		{
			cv::rectangle(frame, face, cv::Scalar(0, 0, 255), 2);
		}


		cv::imshow("image", frame);


		//動画から画像を取り出し，フォルダに保存
		//image_name = "D:\\M1\\Advanced_Food_System_Studies\\1\\image\\image" + std::to_string(i) + ".png";
		//cv::imwrite(image_name, frame);


		//動画を保存
		//writer << frame;


		if (waitKey(30) >= 0) break;



		i++;

	}


	cap.release();//読み込み用のオブジェクトを解放
	//writer.release();//書き込み用のオブジェクトを解放
	cv::destroyAllWindows();//ウィンドウを破棄

	return 0;
}






//https://code-database.com/knowledges/116

int diff_camera(void)
{
	Mat frame;
	Mat gray_frame1, gray_frame2, gray_frame3;
	Mat diff1, diff2;
	Mat diff;
	vector< vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	cv::Rect box;

	cv::Scalar green = cv::Scalar(0, 255, 0);

	VideoCapture cap(0, cv::CAP_DSHOW);
	if (!cap.isOpened())return -1;


	//3フレーム分読み込む
	cap >> frame;
	cv::cvtColor(frame, gray_frame1, cv::COLOR_BGR2GRAY);

	cap >> frame;
	cv::cvtColor(frame, gray_frame2, cv::COLOR_BGR2GRAY);

	cap >> frame;
	cv::cvtColor(frame, gray_frame3, cv::COLOR_BGR2GRAY);





	while (1)
	{
		//フレームの差を求める
		cv::absdiff(gray_frame1, gray_frame2, diff1);
		cv::absdiff(gray_frame2, gray_frame3, diff2);

		//差分1と差分2の結果を比較（論理積）
		cv::bitwise_and(diff1, diff2, diff);

		//輪郭を抽出
		cv::findContours(diff, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		
		//差分があった点を画像に描写
		for (int i = 0; i < contours.size(); i++);
		{
			box = cv::boundingRect(i);

			cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(box.x+box.width, box.y+box.height), green, 2);
		}


		//表示
		cv::namedWindow("diff image", cv::WINDOW_NORMAL);
		cv::imshow("diff image", frame);

		//画像を1フレームずらす
		gray_frame2.copyTo(gray_frame1, gray_frame2);
		gray_frame3.copyTo(gray_frame2, gray_frame3);
		cap >> frame;
		cv::cvtColor(frame, gray_frame3, cv::COLOR_BGR2GRAY);



		if (waitKey(30) >= 0) break;
	}


	cap.release();//読み込み用のオブジェクトを解放
	cv::destroyAllWindows();//ウィンドウを破棄


	return 0;
}




#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //テキストファイルを扱うために用意
#include <string>
#include <process.h>
#include <time.h>
#include <random>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>
#include <opencv2/imgproc.hpp>






//プロトタイプ宣言
void menu_screen(int &ncommand);
int disply_movie();
int tracker_main();
int tracker_init(cv::Ptr<cv::TrackerMedianFlow> &tracker, cv::Rect2d &rect);





using namespace cv;

int main(int argc, const char* argv[])
{
	int n = 0; //画像から動画を作成する際のフレーム数
	int nCommand;
	int nFlag;
	nFlag = 1;

	while (nFlag)
	{
		menu_screen(nCommand);

		switch (nCommand)
		{
			case 1:
				disply_movie();

				break;


			case 2:
				tracker_main();

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
	printf("<<1>>:動画の保存\n");
	printf("<<2>>:物体追跡\n");
	printf("<<3>>:〇〇\n");
	printf("<<0>>:終了します．\n");
	printf("----------------------------------------------------\n");
	printf("\n");

	printf("command=");
	scanf_s("%d", &ncommand);

}





int disply_movie()
{
	int i = 0;
	std::string image_name;
	Mat frame;

	VideoCapture cap(0, cv::CAP_DSHOW);

	int max_frame = (int)cap.get(CAP_PROP_FRAME_COUNT);
	double fps = cap.get(cv::CAP_PROP_FPS);
	int w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
	int h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

	VideoWriter writer("F:\\M1\\Advanced_Food_System_Studies\\1\\movie\\recording.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), fps, Size(w, h));

	if (!cap.isOpened()) return -1;


	namedWindow("image", WINDOW_AUTOSIZE);


	while(1)
	{

		cap >> frame;

		cv::imshow("image", frame);


		//動画から画像を取り出し，フォルダに保存
		//image_name = "F:\\M1\\Advanced_Food_System_Studies\\1\\image\\image" + std::to_string(i) + ".png";
		//cv::imwrite(image_name, frame);


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






int tracker_main()
{
	cv::Ptr<cv::TrackerMedianFlow> tracker = cv::TrackerMedianFlow::create();
	cv::Rect2d roi;


	cv::Scalar color = cv::Scalar(0, 255, 0);
	cv::Mat frame;

	cv::VideoCapture cap(0);
	if (!cap.isOpened())
	{
		std::cout << "Can not open camera!" << std::endl;
		return -1;
	}



	while (cv::waitKey(1) != 'q')
	{
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
	}


	return 0;

}




//MedianFlowによる物体追跡（https://qiita.com/atsisy/items/af0670207535b7cd145f）
int tracker_init(cv::Ptr<cv::TrackerMedianFlow> &tracker, cv::Rect2d &rect)
{
	cv::VideoCapture cap(0);
	if (!cap.isOpened())
	{
		std::cout << "Can not open camera!" << std::endl;
		return -1;
	}

	cv::Mat frame;
	cv::namedWindow("image", WINDOW_AUTOSIZE);

	while (1)
	{
		cap >> frame;
		if (frame.empty())
		{
			std::cout << "Faild to read a frame!" << std::endl;
			cv::destroyAllWindows();
			break;
		}


		cv::imshow("image", frame);




		switch (cv::waitKey(1))
		{
			case 'q':
				return -1;

			case 't':

				rect = cv::selectROI("tracker", frame);
				tracker->init(frame, rect);

				cv::destroyAllWindows();

				return 1;


			default:
				break;
		}

	}

	return -1;
}
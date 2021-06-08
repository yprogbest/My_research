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



//using宣言
using namespace cv;
using namespace std;


//プロトタイプ宣言
void menu_screen();
int yolo_pixcel_count();
int seg_pixcel_count();
int seg_pixcel_count();



int nCommand;


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
	printf("<<3>>:○○\n");
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
	int box_height = 100;
	int box_width = 100;

	//box内のピクセル数
	int box_pixcel_count = box_width * box_height;


	//背景の割合
	double back_ground;



	string folder_path = "D:\\M1\\";
	string image_name = folder_path + "image48.png";

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

	printf("count = %d\n", count);

	back_ground = (((double)box_pixcel_count - (double)count) / (double)box_pixcel_count);


	printf("back_ground = %lf", back_ground);


	cv::namedWindow("result", WINDOW_AUTOSIZE);
	cv::imshow("result", mask_img);

	waitKey();

	return 0;

}





int seg_pixcel_count() {

	int mask_back_count = 0;
	int seg_back_count = 0;

	string folder_path = "D:\\M1\\";
	string image_name = folder_path + "image48.png";
	Mat mask_img = cv::imread(image_name);

	string image_name_seg = folder_path + "image48_seg_test.png";
	Mat seg_img = cv::imread(image_name_seg);



	int img_h = mask_img.rows;
	int img_w = mask_img.cols;

	//背景の割合
	double back_ground;




	for (int j = 0; j < img_h; j++) {
		for (int i = 0; i < img_w; i++) {

			if (mask_img.at<cv::Vec3b>(j, i)[0] <= 127 && mask_img.at<cv::Vec3b>(j, i)[1] <= 127 && mask_img.at<cv::Vec3b>(j, i)[2] <= 127) {

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
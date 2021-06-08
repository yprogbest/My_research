#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //�e�L�X�g�t�@�C�����������߂ɗp��
#include <string>
#include <process.h>
#include <time.h>
#include <random>

#include <opencv2/opencv.hpp>//OpenCV�̃C���N���[�h
#include "opencv2/highgui/highgui.hpp"



//using�錾
using namespace cv;
using namespace std;


//�v���g�^�C�v�錾
void menu_screen();
int yolo_pixcel_count();
int seg_pixcel_count();
int seg_pixcel_count();



int nCommand;


void main(int argc, const char* argv[])
{


	int n = 0; //�摜���瓮����쐬����ۂ̃t���[����

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



//���j���[��ʂ̊֐�
void menu_screen()
{

	printf("\n");
	printf("----------------------------------------------------\n");
	printf("<<1>>:YOLO�̔w�i�̉�f�̊���\n");
	printf("<<2>>:segmentation�̔w�i�̉�f�̊���\n");
	printf("<<3>>:����\n");
	printf("<<0>>:�I�����܂��D\n");
	printf("----------------------------------------------------\n");
	printf("\n");

	printf("command=");
	scanf_s("%d", &nCommand);


}








int yolo_pixcel_count() {

	//�s�N�Z���̃J�E���g��
	int count = 0;

	//Bounding box�̕��ƍ���
	int box_height = 100;
	int box_width = 100;

	//box���̃s�N�Z����
	int box_pixcel_count = box_width * box_height;


	//�w�i�̊���
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

	//�w�i�̊���
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

			//�����C�}�X�N�摜�����i�w�i�j�ŃZ�O�����e�[�V�����̉摜�����i���̏�j�Ȃ�C�w�i�Ȃ̂ŃJ�E���g����
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
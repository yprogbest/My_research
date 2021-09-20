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


#define IMG_XSIZE 672
#define IMG_YSIZE 376


//�e�L�X�g�t�@�C�����̊e�s�̉\�ő啶�����Ƃ��Ē�`
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



//LiDAR��x,y,z�Ɣ��ˋ��x�̃e�L�X�g�t�@�C������ǂݍ��ނ��߂ɗp��
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




//using�錾
using namespace cv;
using namespace std;


//�v���g�^�C�v�錾
void menu_screen();
int yolo_pixcel_count();
int seg_pixcel_count();
int seg_pixcel_count();
int yolo_box_coordinate();
int object_tracking();
int distance_hist(std::vector<cv::Point3f> obj_lidar);
int max_distance_hist(int hist[]);
std::tuple<float, Point3f> get_median(std::vector<cv::Point3f> obj_lidar);



int nCommand;
int i_yolo_lidar = 0;



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



//���j���[��ʂ̊֐�
void menu_screen()
{

	printf("\n");
	printf("----------------------------------------------------\n");
	printf("<<1>>:YOLO�̔w�i�̉�f�̊���\n");
	printf("<<2>>:segmentation�̔w�i�̉�f�̊���\n");
	printf("<<3>>:YOLO�̔F�����ʂ̃t�@�C����1�t���[�����ƕۑ�\n");
	printf("<<4>>:�y���C���z���̒ǐ�\n");
	printf("<<5>>:����\n");
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
	int box_height = 203;
	int box_width = 79;

	//box���̃s�N�Z����
	int box_pixcel_count = box_width * box_height;


	//�w�i�̊���
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

	//�w�i�̊���
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





int yolo_box_coordinate() {


	int count = 0;

	struct yolo_bounding_box yolo_buf_bbox;


	//YOLO�̔F�����ʃt�@�C���̓ǂݍ���
	std::cout << "File path : Yolo Bounding Box Info = " << std::endl;
	string sFilePath_YOLO_BoundingBox;
	cin >> sFilePath_YOLO_BoundingBox;


	//�������ޗp�̃t�H���_�[�w��
	std::cout << "folder for saving box coodinate = " << std::endl;
	string output_folder;
	cin >> output_folder;


	FILE *fp_yolo_bb = NULL;

	//errno_t�E�E�E����I��0�C�ُ�I��0�ȊO
	errno_t err_yolo_bb;

	err_yolo_bb = fopen_s(&fp_yolo_bb, sFilePath_YOLO_BoundingBox.c_str(), "rt");

	//�����C�ُ�I���Ȃ�
	if (err_yolo_bb != 0)
	{
		std::cout << "can not open!" << std::endl;
		std::cout << sFilePath_YOLO_BoundingBox << std::endl;
		return -1;
	}



	int n_yolo_bb_file_stream_size; //�e�s�̕����̐�





	while (1) {

		std::string filename = "output" + std::to_string(count) + ".txt";
		std::string output_data = output_folder + "\\" + filename;
		std::ofstream file_output;
		file_output.open(output_data);


		std::cout << filename << std::endl;

		while (1) {
			//�e�L�X�g�t�@�C�����̊e1�s�ڂ̕������i�[���邽�߂ɗp��
			char yolo_bb_file_BUFFER[YOLO_FILE_LINE_MAX_BUFFER_SIZE]; //1024

																	  //������
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

		//�����CYOLO�̃e�L�X�g�t�@�C���̍s�̕�������0�Ȃ�c(���e�L�X�g�t�@�C������S�ĊJ���I������c)
		if (n_yolo_bb_file_stream_size == 0)
		{
			break; //while�����甲����
		}
	}


	return 0;
}





//����ŏ������s��
int object_tracking() {


	vector<struct point_cloud>point_cloud; //push_back���邽�߂ɗp��
	struct point_cloud point; //LiDAR�̏o�͌���(x,y,z,intensity)

	std::vector<Point3f> point_cloud_LiDAR_yzx; //LiDAR ���W�ϊ�
	Point3f buf_LiDAR_yzx;

	vector<Point2f> imagePoints_LiDAR2ZED; //LiDAR 3D �� 2D
	//float fraction_x; //�_�Q�̏�������
	//float fraction_y; //�_�Q�̏�������
	struct lidar_int imagePoints_LiDAR2ZED_int;//LiDAR�̓_�Q�̐�����
	vector<struct lidar_int>all_imagePoints_LiDAR2ZED_int;;
	vector<Point2f> imagePoints_LiDAR2ZED_in_region_person;
	vector<Point2f> imagePoints_LiDAR2ZED_in_region_container;

	//�F���͈͂�LiDAR�̓_�Q�����邽�߂ɔz���p��
	vector<Point3f> person_lidar1;
	vector<Point3f> person_lidar2;
	vector<Point3f> person_lidar3;
	vector<Point3f> person_lidar4;
	vector<Point3f> person_lidar5;

	vector<Point3f> container_lidar1;
	vector<Point3f> container_lidar2;
	vector<Point3f> container_lidar3;
	vector<Point3f> container_lidar4;
	vector<Point3f> container_lidar5;

	int color_diff = 30;
	int object_color1 = 255;
	int object_color2 = object_color1 - color_diff;
	int object_color3 = object_color2 - color_diff;
	int object_color4 = object_color3 - color_diff;
	int object_color5 = object_color4 - color_diff;
	int object_color6 = object_color5 - color_diff;
	
	//����
	float person_distance1;
	float person_distance2;

	float container_distance1;
	float container_distance2;
	float container_distance3;
	float container_distance4;
	float container_distance5;

	//�����̍ő�l
	float max_distance_person1;
	float max_distance_person2;
	float max_distance_container1;
	float max_distance_container2;
	float max_distance_container3;
	float max_distance_container4;
	float max_distance_container5;

	// �ő�̋����̎���x,y,z���W
	Point3f max_coordinate_person1;
	Point3f max_coordinate_person2;
	Point3f max_coordinate_container1;
	Point3f max_coordinate_container2;
	Point3f max_coordinate_container3;
	Point3f max_coordinate_container4;
	Point3f max_coordinate_container5;






	//mask�摜�̐F�iperson���ԁCcontainer���j
	int b, g, r;
	int b_lidar, g_lidar, r_lidar;
	

	string object_name;

	
	//yaml�t�@�C���̓ǂݍ���(rvec��tvec���g��)

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
	fs_rvec_tvec["mat_T_LiDAR2ZED"] >> mat_T_LiDAR2ZED; //�L�����u���[�V�����ŋ��߂����s�s��
	fs_rvec_tvec["mat_R_LiDAR2ZED"] >> mat_R_LiDAR2ZED; //�L�����u���[�V�����ŋ��߂���]�s��



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



	//LiDAR�̓_�Q�f�[�^�̃e�L�X�g�t�@�C���̃p�X��W�����͂��邽�߂ɗp��
	std::cout << "File path : LiDAR Point Cloud Info = " << std::endl;
	string sFilePath_LiDAR_PointCloud;
	cin >> sFilePath_LiDAR_PointCloud;



	//MaskRCNN�̉摜����͂��邽�߂Ƀt�@�C���p�X���w��
	std::cout << "input color movie = " << std::endl;
	std::string in_put_image_file_path;
	std::cin >> in_put_image_file_path;


	//MaskRCNN�̉摜����͂��邽�߂Ƀt�@�C���p�X���w��
	std::cout << "input mask movie = " << std::endl;
	std::string in_put_image_file_path_mask;
	std::cin >> in_put_image_file_path_mask;


	//�摜���o�͂��邽�߂Ƀt�@�C���p�X���w��
	std::cout << "Files for storing movie = " << std::endl;
	std::string out_put_image_file_path;
	std::cin >> out_put_image_file_path;






	Mat color_image;
	Mat mask_image;
	Mat out_image;
	std::string img_out_name;

	//NN�ŋ��߂�3�����_�Q��2�����ɕϊ�
	vector<Point3f> NN_3D;
	Point3f NN_3D_buf;
	vector<Point2f> NN_2D;



	//maskrcnn�̔F���̍ŏ���f�ƍő��f���`
	Point mask_min, mask_max;

	Point person_xy, container_xy;



	//����iVideocapture�j
	cv::VideoCapture frame_color;
	frame_color.open(in_put_image_file_path);

	int w = (int)frame_color.get(cv::CAP_PROP_FRAME_WIDTH);
	int h = (int)frame_color.get(cv::CAP_PROP_FRAME_HEIGHT);

	cv::VideoCapture frame_mask;
	frame_mask.open(in_put_image_file_path_mask);


	cv::VideoWriter out_frame(out_put_image_file_path+"/output_movie.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 10, Size(w, h));




	while (1)
	{
		//�J���[����̓ǂݍ���
		frame_color >> color_image;

		int color_rows = color_image.rows;
		int color_cols = color_image.cols;


		//mask����̓ǂݍ���
		frame_mask >> mask_image;

		int mask_rows = mask_image.rows;
		int mask_cols = mask_image.cols;


		out_image = mask_image.clone();




		//LiDAR�̃e�L�X�g�t�@�C����ǂݍ��ނ��߂̃t�@�C����p��
		std::string sFilePath_point_cloud_on_image;
		sFilePath_point_cloud_on_image = sFilePath_LiDAR_PointCloud + "/point_cloud" + std::to_string(i_yolo_lidar) + ".txt_NaN_data_removed.txt";

		//�t�@�C�����J��
		std::ifstream lidar_text_file;
		lidar_text_file.open(sFilePath_point_cloud_on_image);

		if (!lidar_text_file.is_open())
		{
			std::cerr << "Can not open " + sFilePath_point_cloud_on_image << std::endl;

			return -1;

		}



		//Maskrcnn�̔F���͈͓��́wLiDAR�x�̓_�Q���擾
		while (1)
		{


			if (!lidar_text_file.eof())
			{


				//LiDAR�̃t�@�C�������X�L�������Ă����i�S�Ă̍s��ǂݍ��ށj
				lidar_text_file >> point.x;
				lidar_text_file >> point.y;
				lidar_text_file >> point.z;
				lidar_text_file >> point.intensity;


				point_cloud.push_back(point); //push_back




			} //�����CLiDAR�̃e�L�X�g�t�@�C���ɕ���������Ȃ�E�E�E�̏I���
			else
			{
				lidar_text_file.close();
				break;
			}

		} //�wMaskrcnn�̔F���͈͓��́wLiDAR�x�̓_�Q���擾�x�I��(while��)



		// change the coordinates of xyz -> yzx (LiDAR)
		for (int k = 0; k < (int)point_cloud.size(); k++)
		{
			buf_LiDAR_yzx.x = (float)-point_cloud[k].y;
			buf_LiDAR_yzx.y = (float)-point_cloud[k].z;
			buf_LiDAR_yzx.z = (float)point_cloud[k].x;

			point_cloud_LiDAR_yzx.push_back(buf_LiDAR_yzx);
		}

		cv::projectPoints(point_cloud_LiDAR_yzx, rvec, tvec, K1, D1, imagePoints_LiDAR2ZED);

		
		for (int k = 0; k < (int)imagePoints_LiDAR2ZED.size(); k++)
		{
			if (point_cloud_LiDAR_yzx[k].z < 0) continue; 
			if (imagePoints_LiDAR2ZED[k].x < 0 || imagePoints_LiDAR2ZED[k].x >= IMG_XSIZE || imagePoints_LiDAR2ZED[k].y < 0 || imagePoints_LiDAR2ZED[k].y >= IMG_YSIZE) continue;
			

			//LiDAR�_�Q�̐�����
			imagePoints_LiDAR2ZED[k].x = int(imagePoints_LiDAR2ZED[k].x + 0.5);
			imagePoints_LiDAR2ZED[k].y = int(imagePoints_LiDAR2ZED[k].y + 0.5);

			//std::cout << imagePoints_LiDAR2ZED[k].x << "\t" << imagePoints_LiDAR2ZED[k].y << std::endl;


			//�F�����ʂ͈͓̔���LiDAR�_�Q�����݂����f�l
			b_lidar = mask_image.at<cv::Vec3b>(imagePoints_LiDAR2ZED[k].y, imagePoints_LiDAR2ZED[k].x)[0];
			g_lidar = mask_image.at<cv::Vec3b>(imagePoints_LiDAR2ZED[k].y, imagePoints_LiDAR2ZED[k].x)[1];
			r_lidar = mask_image.at<cv::Vec3b>(imagePoints_LiDAR2ZED[k].y, imagePoints_LiDAR2ZED[k].x)[2];


			//person
			//1�l��
			if (b_lidar < 10 && g_lidar < 10 && (object_color2 < r_lidar &&r_lidar <= object_color1))
			{
				person_lidar1.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);

			}
			//2�l��
			else if (b_lidar < 10 && g_lidar < 10 && (object_color3 < r_lidar &&r_lidar <= object_color2))
			{
				person_lidar2.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);
				
			}
			/*else if (b_lidar < 5 && g_lidar < 5 && (object_color4 < r_lidar &&r_lidar <= object_color3))
			{
				person_lidar3.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);

			}
			else if (b_lidar < 5 && g_lidar < 5 && (object_color5 < r_lidar &&r_lidar <= object_color4))
			{
				person_lidar4.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);

			}
			else if (b_lidar < 5 && g_lidar < 5 && (object_color6 < r_lidar &&r_lidar <= object_color5))
			{
				person_lidar5.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);

			}*/


			//container
			//1��
			if ((object_color2 < b_lidar &&b_lidar <= object_color1) && g_lidar < 10 && r_lidar < 10)
			{
				container_lidar1.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);

			}
			//2��
			else if ((object_color3 < b_lidar &&b_lidar <= object_color2) && g_lidar < 10 && r_lidar < 10)
			{
				container_lidar2.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);

			}
			//3��
			else if ((object_color4 < b_lidar &&b_lidar <= object_color3) && g_lidar < 10 && r_lidar < 10)
			{
				container_lidar3.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);

			}
			//4��
			else if ((object_color5 < b_lidar &&b_lidar <= object_color4) && g_lidar < 10 && r_lidar < 10)
			{
				container_lidar4.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);

			}
			//5��
			else if ((object_color6 < b_lidar &&b_lidar <= object_color5) && g_lidar < 10 && r_lidar < 10)
			{
				container_lidar5.push_back(point_cloud_LiDAR_yzx[k]);
				cv::circle(out_image, cv::Point((int)imagePoints_LiDAR2ZED[k].x, (int)imagePoints_LiDAR2ZED[k].y), 3, cv::Scalar(0, 255, 0), 0.5, cv::LINE_8);
			}

		}


		// find distance //
		// use fistgram /////////////////////////////////////////

		////person1
		//max_distance_person1 = distance_hist(person_lidar1);
		////printf("max_distance = %d m\n", max_distance_person1);

		////person2
		//max_distance_person2 = distance_hist(person_lidar2);

		////container1
		//max_distance_container1 = distance_hist(container_lidar1);

		////container2
		//max_distance_container2 = distance_hist(container_lidar2);

		////container3
		//max_distance_container3 = distance_hist(container_lidar3);

		////container4
		//max_distance_container4 = distance_hist(container_lidar4);

		////container5
		//max_distance_container5 = distance_hist(container_lidar5);

		///////////////////////////////////////////////////////////



		// use median/////////////////////////////////////////////
		std::tie(max_distance_person1, max_coordinate_person1) = get_median(person_lidar1);
		std::tie(max_distance_person2, max_coordinate_person1) = get_median(person_lidar2);
		std::tie(max_distance_container1, max_coordinate_container1) = get_median(container_lidar1);
		std::tie(max_distance_container2, max_coordinate_container2) = get_median(container_lidar2);
		std::tie(max_distance_container3, max_coordinate_container3) = get_median(container_lidar3);
		std::tie(max_distance_container4, max_coordinate_container4) = get_median(container_lidar4);
		std::tie(max_distance_container5, max_coordinate_container5) = get_median(container_lidar5);

		printf("person_median_x = %lf\n", max_coordinate_person1.x);
		///////////////////////////////////////////////////////////

		// ��find distance//





		// plot onto Gnuplot //



		// //




		//�摜�̏o��
		img_out_name = out_put_image_file_path + "/image" + std::to_string(i_yolo_lidar) + ".png";
		cv::imwrite(img_out_name, out_image);


		//����̏o��
		out_frame << out_image;



		std::cout << i_yolo_lidar << std::endl;


		i_yolo_lidar++;


		//std::cout << "person1_num = " << person_lidar1.size() << "\t" << "person2_num = " << person_lidar2.size() << "\t" << "person3_num = " << person_lidar3.size() << "\t" << "person4_num = " << person_lidar4.size() << "\t" << "person5_num = " << person_lidar5.size() << std::endl;
		//std::cout << "container1_num = " << container_lidar1.size() << "\t" << "container2_num = " << container_lidar2.size() << "\t" << "container3_num = " << container_lidar3.size() << "\t" << "container4_num = " << container_lidar4.size() << "\t" << "container5_num = " << container_lidar5.size() << std::endl;


		person_lidar1.clear();
		person_lidar1.shrink_to_fit();
		person_lidar2.clear();
		person_lidar2.shrink_to_fit();
		/*person_lidar3.clear();
		person_lidar3.shrink_to_fit();
		person_lidar4.clear();
		person_lidar4.shrink_to_fit();
		person_lidar5.clear();
		person_lidar5.shrink_to_fit();*/

		container_lidar1.clear();
		container_lidar1.shrink_to_fit();
		container_lidar2.clear();
		container_lidar2.shrink_to_fit();
		container_lidar3.clear();
		container_lidar3.shrink_to_fit();
		container_lidar4.clear();
		container_lidar4.shrink_to_fit();
		container_lidar5.clear();
		container_lidar5.shrink_to_fit();

		point_cloud.clear();
		point_cloud.shrink_to_fit();

		point_cloud_LiDAR_yzx.clear();
		point_cloud_LiDAR_yzx.shrink_to_fit();

		imagePoints_LiDAR2ZED.clear();
		imagePoints_LiDAR2ZED.shrink_to_fit();

	}


	frame_color.release();
	frame_mask.release();
	out_frame.release();
	cv::destroyAllWindows();


	return 0;
}


//����i�摜�ŏ������s���j
/*
int object_tracking() {


	vector<struct point_cloud>point_cloud; //push_back���邽�߂ɗp��
	struct point_cloud point; //LiDAR�̏o�͌���(x,y,z,intensity)
	
	std::vector<Point3f> point_cloud_LiDAR_yzx; //LiDAR ���W�ϊ�
	Point3f buf_LiDAR_yzx;
	
	vector<Point2f> imagePoints_LiDAR2ZED; //LiDAR 3D �� 2D
	float fraction_x; //�_�Q�̏�������
	float fraction_y; //�_�Q�̏�������
	Point imagePoints_LiDAR2ZED_int;//LiDAR�̓_�Q�̐�����
	vector<Point> all_imagePoints_LiDAR2ZED_int; 
	vector<Point2f> imagePoints_LiDAR2ZED_in_region_person;
	vector<Point2f> imagePoints_LiDAR2ZED_in_region_container;

	//mask�摜�̐F�iperson���ԁCcontainer���j
	int b, g, r;

	string object_name;




	//yaml�t�@�C���̓ǂݍ���(rvec��tvec���g��)

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
	fs_rvec_tvec["mat_T_LiDAR2ZED"] >> mat_T_LiDAR2ZED; //�L�����u���[�V�����ŋ��߂����s�s��
	fs_rvec_tvec["mat_R_LiDAR2ZED"] >> mat_R_LiDAR2ZED; //�L�����u���[�V�����ŋ��߂���]�s��



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



	//LiDAR�̓_�Q�f�[�^�̃e�L�X�g�t�@�C���̃p�X��W�����͂��邽�߂ɗp��
	std::cout << "File path : LiDAR Point Cloud Info = " << std::endl;
	string sFilePath_LiDAR_PointCloud;
	cin >> sFilePath_LiDAR_PointCloud;



	//MaskRCNN�̉摜����͂��邽�߂Ƀt�@�C���p�X���w��
	std::cout << "input color images = " << std::endl;
	std::string in_put_image_file_path;
	std::cin >> in_put_image_file_path;


	//MaskRCNN�̉摜����͂��邽�߂Ƀt�@�C���p�X���w��
	std::cout << "input mask images = " << std::endl;
	std::string in_put_image_file_path_mask;
	std::cin >> in_put_image_file_path_mask;


	//�摜���o�͂��邽�߂Ƀt�@�C���p�X���w��
	std::cout << "Files for storing images = " << std::endl;
	std::string out_put_image_file_path;
	std::cin >> out_put_image_file_path;






	Mat color_image;
	Mat mask_image;
	Mat out_image;
	std::string img_out_name;

	//NN�ŋ��߂�3�����_�Q��2�����ɕϊ�
	vector<Point3f> NN_3D;
	Point3f NN_3D_buf;
	vector<Point2f> NN_2D;



	//maskrcnn�̔F���̍ŏ���f�ƍő��f���`
	Point mask_min, mask_max;

	Point person_xy, container_xy;




	while (1) 
	{
	
		//�J���[�摜�̓ǂݍ���
		std::string color_img_in_name;
		color_img_in_name = in_put_image_file_path + "/image" + std::to_string(i_yolo_lidar) + ".png";
		color_image = cv::imread(color_img_in_name);

		int color_rows = color_image.rows;
		int color_cols = color_image.cols;


		//mask�摜�̓ǂݍ���
		std::string mask_img_in_name;
		mask_img_in_name = in_put_image_file_path_mask + "/image" + std::to_string(i_yolo_lidar) + ".png";
		mask_image = cv::imread(mask_img_in_name);

		int mask_rows = mask_image.rows;
		int mask_cols = mask_image.cols;


		out_image = mask_image.clone();


		

		//LiDAR�̃e�L�X�g�t�@�C����ǂݍ��ނ��߂̃t�@�C����p��
		std::string sFilePath_point_cloud_on_image;
		sFilePath_point_cloud_on_image = sFilePath_LiDAR_PointCloud + "/point_cloud" + std::to_string(i_yolo_lidar) + ".txt_NaN_data_removed.txt";

		//�t�@�C�����J��
		std::ifstream lidar_text_file;
		lidar_text_file.open(sFilePath_point_cloud_on_image);

		if (!lidar_text_file.is_open())
		{
			std::cerr << "Can not open " + sFilePath_point_cloud_on_image << std::endl;

			return -1;

		}



		//Maskrcnn�̔F���͈͓��́wLiDAR�x�̓_�Q���擾
		while (1)
		{


			if (!lidar_text_file.eof())
			{


				//LiDAR�̃t�@�C�������X�L�������Ă����i�S�Ă̍s��ǂݍ��ށj
				lidar_text_file >> point.x;
				lidar_text_file >> point.y;
				lidar_text_file >> point.z;
				lidar_text_file >> point.intensity;


				point_cloud.push_back(point); //push_back




			} //�����CLiDAR�̃e�L�X�g�t�@�C���ɕ���������Ȃ�E�E�E�̏I���
			else
			{
				lidar_text_file.close();
				break;
			}

		} //�wMaskrcnn�̔F���͈͓��́wLiDAR�x�̓_�Q���擾�x�I��(while��)



		// change the coordinates of xyz -> yzx (LiDAR)
		for (int k = 0; k < (int)point_cloud.size(); k++)
		{
			buf_LiDAR_yzx.x = (float)-point_cloud[k].y;
			buf_LiDAR_yzx.y = (float)-point_cloud[k].z;
			buf_LiDAR_yzx.z = (float)point_cloud[k].x;

			point_cloud_LiDAR_yzx.push_back(buf_LiDAR_yzx);
		}
		
		cv::projectPoints(point_cloud_LiDAR_yzx, rvec, tvec, K1, D1, imagePoints_LiDAR2ZED);


		//LiDAR�̓_�Q�̐�����
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
									//�F���͈͓���LiDAR�_�Q���v�b�V���o�b�N
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
									//�F���͈͓���LiDAR�_�Q���v�b�V���o�b�N
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

	

		//����̏o��
		img_out_name = out_put_image_file_path + "/image" + std::to_string(i_yolo_lidar) + ".png";
		cv::imwrite(img_out_name, out_image);

		std::cout << i_yolo_lidar << std::endl;


		i_yolo_lidar++;
	
	}




	return 0;
}

*/





//��\�_�̎Z�o��@1(�q�X�g�O����)���_�Q��x,y,z�l���o�͏o���Ȃ��Ȃ�
int distance_hist(std::vector<cv::Point3f> obj_lidar)
{
	//0m����150m
	const int hist_array = 150;
	int hist[150];
	float distance;
	int z_class = 0;
	float fraction;
	int max_distance;


	//hist�̏�����
	for (int a = 0; a < hist_array; a++)
	{
		hist[a] = 0;
	}

	for (int i = 0; i < obj_lidar.size(); i++)
	{
		// find a distance from the sensor to the object
		distance = obj_lidar[i].x * obj_lidar[i].x + obj_lidar[i].y * obj_lidar[i].y + obj_lidar[i].z * obj_lidar[i].z;
		distance = sqrt(distance);


		//��������
		fraction = distance - int(distance);

		if (fraction >= 0.5)
		{
			z_class = int(distance + 0.5);
		}
		else
		{
			z_class = int(distance);
		}

		hist[z_class]++;

		//printf("z_class = %d m\n", z_class);
	}

	max_distance = max_distance_hist(hist);


	return max_distance;
}


int max_distance_hist(int hist[])
{
	int max = -1;
	int a_max = -1;
	const int hist_array = 150;


	for (int a = 0; a < hist_array; a++)
	{
		if (max < hist[a])
		{
			max = hist[a];
			a_max = a;
		}
	}

	return a_max;
}



//��\�_�̎Z�o��@2(Median)
std::tuple<float, Point3f> get_median(std::vector<cv::Point3f> obj_lidar)
{
	float distance;
	std::vector<float> distance_vec;

	float tmp_distance;
	float tmp_x;
	float tmp_y;
	float tmp_z;

	float median_distance;
	Point3f median_lidar_point;


	// find a distance from the sensor to the object
	for (int i = 0; i < obj_lidar.size(); i++)
	{
		distance = obj_lidar[i].x * obj_lidar[i].x + obj_lidar[i].y * obj_lidar[i].y + obj_lidar[i].z * obj_lidar[i].z;
		distance = sqrt(distance);

		distance_vec.push_back(distance);
	}

	//Sort in descending order
	for (int i = 0; i < distance_vec.size(); i++)
	{
		for (int h = i + 1; h < distance_vec.size(); h++)
		{
			if (distance_vec[i] < distance_vec[h])
			{
				// distance
				tmp_distance = distance_vec[h];
				distance_vec[h] = distance_vec[i];
				distance_vec[i] = tmp_distance;

				// x
				tmp_x = obj_lidar[h].x;
				obj_lidar[h].x = obj_lidar[i].x;
				obj_lidar[i].x = tmp_x;

				// y
				tmp_y = obj_lidar[h].y;
				obj_lidar[h].y = obj_lidar[i].y;
				obj_lidar[i].y = tmp_y;

				// z
				tmp_z = obj_lidar[h].z;
				obj_lidar[h].z = obj_lidar[i].z;
				obj_lidar[i].z = tmp_z;
			}
		}
	}


	// find median value
	if (distance_vec.size() == 0)
	{
		median_distance = 0.0;
		median_lidar_point.x = 0.0;
		median_lidar_point.y = 0.0;
		median_lidar_point.z = 0.0;
	}
	else if (distance_vec.size() % 2 == 1)
	{
		median_distance = distance_vec[(int)((int)distance_vec.size() / 2)];
		median_lidar_point.x = obj_lidar[(int)((int)obj_lidar.size() / 2)].x;
		median_lidar_point.y = obj_lidar[(int)((int)obj_lidar.size() / 2)].y;
		median_lidar_point.z = obj_lidar[(int)((int)obj_lidar.size() / 2)].z;
	}
	else if (distance_vec.size() % 2 == 0)
	{
		median_distance = (distance_vec[(int)((int)distance_vec.size() / 2) - 1] + distance_vec[(int)((int)distance_vec.size() / 2)]) / 2.0;
		median_lidar_point.x = (obj_lidar[(int)((int)obj_lidar.size() / 2) - 1].x + obj_lidar[(int)((int)obj_lidar.size() / 2)].x) / 2.0;
		median_lidar_point.y = (obj_lidar[(int)((int)obj_lidar.size() / 2) - 1].y + obj_lidar[(int)((int)obj_lidar.size() / 2)].y) / 2.0;
		median_lidar_point.z = (obj_lidar[(int)((int)obj_lidar.size() / 2) - 1].z + obj_lidar[(int)((int)obj_lidar.size() / 2)].z) / 2.0;
	}

	return{ median_distance, median_lidar_point };
}


// Gnuplot�ɑ�\�_���v���b�g����
int gnuplot_mapping()
{

}


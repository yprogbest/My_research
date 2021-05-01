#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //�e�L�X�g�t�@�C�����������߂ɗp��
#include <string>
#include <process.h>

#include <opencv2/opencv.hpp>//OpenCV�̃C���N���[�h
#include "opencv2/highgui/highgui.hpp"



#define IMG_XSIZE 672
#define IMG_YSIZE 376


//�e�L�X�g�t�@�C�����̊e�s�̉\�ő啶�����Ƃ��Ē�`
#define YOLO_FILE_LINE_MAX_BUFFER_SIZE	1024
#define LiDAR_FILE_LINE_MAX_BUFFER_SIZE 1024

//YOLO�Ŋw�K�����镨�̖̂��̂�����t�@�C���\�ő�̈���`
#define YOLO_LABEL_STRING_MAX_LENGTH  100


//using�錾
using namespace cv;
using namespace std;





//�o�E���f�B���O�{�b�N�X���k������ۂɎg��
//////////////////////////////////////////
//���ꁩ�����ύX����
//double denominator = 4.5;

//���q
//double numerator = denominator - 1.0;
//////////////////////////////////////////




char *not_transporting;
char *transporting;
char *container;
char *person;




//histgram
double a_median;


int n_class = 0;


int nCommand;


int i_yolo_lidar = 0;




//HANDLE hThread_gnuplot;
//int n_flag_thread_gnuplot_exit;





//�v���g�^�C�v�錾
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
double make_histgram(std::string outputfile_path);
double median_distance(int hist[]);
int gnuplot_mapping();

int gnuplot_histgram();




//YOLO�̔F�����ʂ̃e�L�X�g�t�@�C�����̍��ڂ�ǂݍ��ނ��߂ɗp��
struct yolo_bounfing_box
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




//�q�X�g�O������\�����邽�߂ɗp�Ӂi���j
struct hist_data
{
	int i;
	int count;
};








//YOLO Variables

//yolo bounding box parameters
vector<struct yolo_bounfing_box> yolo_BBox; //1�܂��̉摜���ʂ��Ƃɒǉ����Ă����̂ŁC�z��^�ivector�j�ɂ��Ă���
struct yolo_bounfing_box yolo_buf_bbox;

//LiDAR Variables
vector<struct point_cloud>point_cloud; //push_back���邽�߂ɗp��
struct point_cloud point; //LiDAR�̏o�͌���(x,y,z,intensity)

vector<Point2f> yolo_lidar_inlier_pointcloud;
cv::Point2f yolo_lidar_inlier_point;

std::vector<Point3f> point_cloud_LiDAR_yzx;
cv::Point3f buf_LiDAR_yzx;

std::vector<double>all_lidar_distance;
double lidar_distance;

//LiDAR 3D �� 2D//
vector<Point2f> imagePoints_LiDAR2ZED;


std::vector<Point3f>all_point_cloud_in_a_boundingbox_3D;
cv::Point3f point_cloud_in_a_boundingbox_3D;


std::vector<double>all_lidar_distance_in_boundingbox;
double lidar_distance_in_boundingbox;



std::vector<Point3f>all_point_cloud_in_a_boundingbox_container;
cv::Point3f point_cloud_in_a_boundingbox_container;

std::vector<Point3f>all_point_cloud_in_a_boundingbox_person;
cv::Point3f point_cloud_in_a_boundingbox_person;


//Number of votes in histogram
const int hist_array = 151; //��ɂ���imedian�����߂�ۂɁC�������ƁC�z��̒����̏��Ԃ������ɂȂ�j
std::vector<int> all_hist;
int hist[hist_array];







//argc�͈����̌��Aargv�ɂ͈����̎��ۂ̒l������
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
			//����̓��o��
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


//���j���[��ʂ̊֐�
void menu_screen()
{

	printf("\n");
	printf("----------------------------------------------------\n");
	printf("<<1>>:����̓��o��\n");
	printf("<<2>>:�摜�̃��x�����O\n");
	printf("<<3>>:���摜�t�@�C�����瓮��𐶐�\n");
	printf("<<4>>:���摜(��������)�ȊO�̉摜���瓮��𐶐�\n");
	printf("<<5>>:���悩��摜���쐬\n");
	printf("<<6>>:�ʂ̃t�H���_�[�ɉ摜��ۑ�\n");
	printf("<<7>>:�e�L�X�g�t�@�C����ʂ̃t�H���_�[�փR�s�[���܂��D\n");
	//printf("<<8>>:LiDAR�̓_�Q�̎ʑ��摜�̏����ƃe�L�X�g�t�@�C���̏���\n");
	printf("<<9>>:YOLO�̃o�E���f�B���O�{�b�N�X���ɂ���LiDAR�_�Q�̋��������߂�\n");
	printf("<<10>>:�o�E���f�B���O�{�b�N�X����LiDAR�_�Q���q�X�g�O�����ŕ\������\n");
	printf("<<11>>:�Z�Z\n");
	printf("<<0>>:�I�����܂��D\n");
	printf("----------------------------------------------------\n");
	printf("\n");

	printf("command=");
	scanf_s("%d", &nCommand);


}



//����̓��o��
int OpencvVideocaputure(int argc, const char* argv[]) {

	Mat img;
	Mat gray_img;
	Mat faussian_img;
	Mat bin_img;
	Mat canny_img;


	//����t�@�C�����w��
	std::cout << "FileName=" << std::flush;


	//�����^�̃t�@�C����p��
	std::string VideoFile = argv[argc - 1];



	//�R�}���h�v�����v����t�@�C���������
	std::cin >> VideoFile;



	//https://cvtech.cc/opencv/2/
	//����ǂݍ��ݗp�̃I�u�W�F�N�g�𐶐�
	VideoCapture cap(VideoFile);


	//http://shibafu3.hatenablog.com/entry/2016/11/13/151118
	//���揑�����ݗp�̃I�u�W�F�N�g�𐶐�
	VideoWriter writer("D:\\latter_graduation_research\\recording.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 100, Size(672, 376));




	// cv::VideoCapture������ɓ��삵�Ă��邩���`�F�b�N�I
	if (!cap.isOpened())
	{

		std::cerr << "cannot open this movie file!" << std::endl;

		return -1;
	}

	// cv::VideoWriter������ɓ��삵�Ă��邩���`�F�b�N�I
	if (!writer.isOpened())
	{

		std::cerr << "failed to record" << std::endl;

		return -1;
	}



	double max_frame = cap.get(CAP_PROP_FRAME_COUNT);//�t���[�����@���icast�������������j�icast�Ƃ́Fhttps://www.sejuku.net/blog/25737�j


	for (int i = 0; i < int(max_frame); i++)
	{


		cap >> img;//1�t���[�������o����img�ɕۑ�������



				   //�O���C�X�P�[����
		cvtColor(img, gray_img, CV_BGR2GRAY);



		//������
		GaussianBlur(gray_img, faussian_img, Size(7, 7), 0.0);




		//2�l��
		threshold(faussian_img, bin_img, 50, 255, THRESH_BINARY);



		//�֊s���o
		Canny(img, canny_img, 60.0, 150.0);




		imshow("Video1", img);

		imshow("Video2", bin_img);

		imshow("Video3", canny_img);



		//�^��(����t�@�C���ɉ摜���o��)
		writer << bin_img;


		//�����CEnter�L�[�������ꂽ�狭���I��
		if (waitKey(1) == 13)
		{
			break;
		}

	}



	cap.release();//�ǂݍ��ݗp�̃I�u�W�F�N�g�����
	writer.release();//�������ݗp�̃I�u�W�F�N�g�����
	destroyAllWindows();//�E�B���h�E��j��


	return 0;
}





//���x�����O
//http://imgprolab.sys.fit.ac.jp/~yama/imgproc/proc/Document_OpenCVforC_5_2019.pdf
int Labeling(int argc, const char* argv[]) {

	Mat img;
	Mat gray_img;
	Mat bin_img;
	Mat label_img;


	//�摜�t�@�C�����w��
	std::cout << "FileName=" << std::flush;



	//�����^�̃t�@�C����p��
	std::string ImageFile = argv[argc - 1];



	//�R�}���h�v�����v����t�@�C���������
	std::cin >> ImageFile;



	img = imread(ImageFile);




	imshow("Original Image", img);



	//�O���C�X�P�[����
	cvtColor(img, gray_img, COLOR_BGR2GRAY);



	//2�l��
	threshold(gray_img, bin_img, 50, 255, THRESH_BINARY);



	//2�l�������摜��2��Dilate����(�c������)
	dilate(bin_img, bin_img, noArray(), Point(-1, -1), 2);//Q.�Ȃ��CnoArray()���g���Ă���̂��H & Point(-1, -1)�͉���\���Ă���̂��H


	imshow("Binary-Dilate Image", bin_img);




	//���x�����O(8�ߖT)�@�o�́F���x�����O���ꂽ�}�`�����̌�
	int nLabels = connectedComponents(bin_img, label_img, 8, CV_32S);





	//���x�����O���ʂ̕`��F�����肷��

	std::vector<Vec3b>colors(nLabels);//Vec3b�E�E�E0����255�̐��l���i�[�ł��锠��3�p�ӂ���ϐ�



	colors[0] = Vec3b(0, 0, 0);



	for (int label = 1; label < nLabels; label++)
	{


		//���x���ԍ��ɑ΂��āC�����_���ɐF�����蓖�Ă�(&255�E�E�E0����255)
		colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));

	}





	//���x�����O���ʂ̕`��(CV_8UC3�E�E�Eunsigned char [3])
	Mat dst(img.size(), CV_8UC3);


	for (int y = 0; y < dst.rows; y++)
	{
		for (int x = 0; x < dst.cols; x++)
		{
			//���x�����O�摜��(x,y)��̃��x���ԍ��𒊏o
			int label = label_img.at<int>(y, x);


			//���x���ԍ��Ɋ��蓖�Ă�ꂽ�F�i��f�l�j�����ʉ摜��(x,y)�Ɋi�[����@�i���Q��(reference)�j
			cv::Vec3b &pixel = dst.at<cv::Vec3b>(y, x);


			pixel = colors[label];


		}

	}


	imshow("Labeling Image", dst);



	waitKey();


	return 0;
}






//���摜�t�@�C�����瓮��𐶐�(https://kisqragi.hatenablog.com/entry/2019/11/02/142240)
int ImageToMovie(int n) {

	int start, goal;


	//�ǂݍ��މ摜�̃t�@�C���p�X���w��
	std::cout << "Path name of the image file to be loaded = " << std::endl;


	std::string image_file_path;
	std::cin >> image_file_path;



	//�t�@�C���̎n�߂ƏI�����w��
	std::cout << "File Start Number = " << std::endl;
	std::cin >> start;

	std::cout << "File Goal Number = " << std::endl;
	std::cin >> goal;




	//�ۑ���̃p�X�����w��
	std::cout << "Destination path name = " << std::endl;

	std::string destination_file_path;
	std::cin >> destination_file_path;




	std::cout << "Number of frames=" << std::endl;
	std::cin >> n;





	//�e�L�X�g�t�@�C���Ƀt���[��������������(https://qiita.com/fantm21/items/8489b944698f9d3818ea)
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




		//�X�e���I�摜���E�ƍ��ŕ���
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





//LiDAR�̓_�Q���ʑ������摜���瓮��𐶐�
int LidarImageToMovie(int n) {

	int start, goal;


	//�ǂݍ��މ摜�̃t�@�C���p�X���w��
	std::cout << "Path name of the image file to be loaded = " << std::endl;


	std::string image_file_path;
	std::cin >> image_file_path;



	//�t�@�C���̎n�߂ƏI�����w��
	std::cout << "File Start Number = " << std::endl;
	std::cin >> start;

	std::cout << "File Goal Number = " << std::endl;
	std::cin >> goal;


	//�摜�p�X�̔ԍ������̕�����W������
	std::cout << "Characters after numbers" << std::endl;
	std::string characters_after_numbers;
	std::cin >> characters_after_numbers;



	//�ۑ���̃p�X�����w��
	std::cout << "Destination path name = " << std::endl;

	std::string destination_file_path;
	std::cin >> destination_file_path;



	std::cout << "Number of frames=" << std::endl;
	std::cin >> n;







	//�e�L�X�g�t�@�C���Ƀt���[��������������(https://qiita.com/fantm21/items/8489b944698f9d3818ea)
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


	//�摜�p
	//Size(672, 376)

	//gnuplot�p
	//Size(640, 480)



	if (!writer.isOpened())
	{
		std::cerr << "This File can not be opend!" << std::endl;

		return -1;
	}




	for (int i = start; i <= goal; i++)
	{

		Mat img = imread(image_file_path + "\\image" + std::to_string(i) + characters_after_numbers);



		//�����CLiDAR�̓_�Q���ʂ̉摜�������Ȃ�D�D�D(�r���̔ԍ�����ł��g����悤�ɗp�ӂ��Ă���)
		if (img.empty())
		{
			//���̉摜�ɍs��
			continue;

		}




		writer << img;


		std::cout << i << std::endl;
	}


	std::cout << "success\n" << std::endl;


	writer.release();




	return 0;
}







//���悩��摜���쐬
int Movie_to_image()
{
	Mat img;


	std::wcout << "Movie Path =" << std::endl;
	std::string movie_file_path;
	std::cin >> movie_file_path;

	//������L���v�`��
	VideoCapture cap(movie_file_path); //Windows�̏ꍇ�@�p�X����?�͏d�˂�??�Ƃ���
									   //VideoCapture cap("videos/sample.mp4"); //Mac�̏ꍇ

									   //����̍ő�t���[����
	int max_frame = (int)cap.get(CAP_PROP_FRAME_COUNT);
	// ����̍���
	int img_h = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
	// ����̕�
	int img_w = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
	// �����fps
	double fps = cap.get(CAP_PROP_FPS);


	//�ۑ��p�t�@�C���p�X�̎w��
	std::cout << "File path for storage=" << std::endl;
	std::string file_path_for_storage;
	std::cin >> file_path_for_storage;


	for (int i = 0; i < max_frame; i++) {

		//1�t���[�������o����img�ɕێ�������
		cap >> img;

		//���݂̃t���[���ԍ���\��
		std::cout << "�t���[���ԍ� " << i << std::endl;


		//�擾�����摜��A�ԉ摜�ŕۑ�

		std::string image_name;
		image_name = file_path_for_storage + "/image" + std::to_string(i) + ".png";
		cv::imwrite(image_name, img);

	}
	//������̏o��
	std::cout << "�t���[���ԍ� " << max_frame - 1 << std::endl;
	std::cout << "����̍��� " << img_h << std::endl;
	std::cout << "����̕� " << img_w << std::endl;
	std::cout << "�����fps " << fps << std::endl;



	return 0;

}






//�t�H���_�[���̉摜�����o���āC���̃t�H���_�[�ɕۑ�����
int ImageDownloadFile() {

	int start, goal;


	//�ǂݍ��މ摜�̃t�@�C���p�X���w��
	std::cout << "Path name of the image file to be loaded = " << std::endl;


	std::string image_file_path;
	std::cin >> image_file_path;



	//�t�@�C���̎n�߂ƏI�����w��
	std::cout << "File Start Number = " << std::endl;
	std::cin >> start;

	std::cout << "File Goal Number = " << std::endl;
	std::cin >> goal;




	//�ۑ���̃p�X�����w��
	std::cout << "Destination path name = " << std::endl;

	std::string destination_file_path;
	std::cin >> destination_file_path;



	//�ۑ����鎞�̉摜�̊J�n�ԍ����w��i��Fimage50.jpg��50�̕��������R�Ɍ��߂�D�j
	std::cout << "Starting number when saving the image = " << std::endl;
	int j;
	std::cin >> j;






	for (int i = start; i <= goal; i++)
	{

		//�ǂݍ��މ摜
		Mat img = imread(image_file_path + "\\image" + std::to_string(i) + ".png");




		//�X�e���I�摜���E�ƍ��ŕ���
		Mat img_cam1 = img(cv::Rect(0, 0, img.cols / 2, img.rows));
		Mat img_cam2 = img(cv::Rect(img.cols / 2, 0, img.cols / 2, img.rows));



		if (img.empty())
		{
			std::cerr << "Error!!" << std::endl;

			return -1;
		}


		//�ʂ̃t�H���_�[�ɕۑ�����

		std::string img_name;
		img_name = destination_file_path + "\\image" + std::to_string(j + i) + ".jpg";

		cv::imwrite(img_name, img_cam1);

		std::cout << img_name << std::endl;
	}

	std::cout << "OK!" << std::endl;


	return 0;

}




//�e�L�X�g�t�@�C����ʂ̃t�H���_�[�ɃR�s�[(https://programming-place.net/ppp/contents/c/rev_res/file001.html)
int textfile_to_another_folder()
{

	//�ǂݍ��ރe�L�X�g�t�@�C�����ۑ�����Ă���t�H���_�[��W������
	std::cout << "input_folder = " << std::endl;
	std::string input_foder_path;
	std::cin >> input_foder_path;


	//�������ރe�L�X�g�t�@�C�����ۑ�����Ă���t�H���_�[��W������
	std::cout << "output_folder = " << std::endl;
	std::string output_foder_path;
	std::cin >> output_foder_path;


	//�e�L�X�g�t�@�C������W������
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




//LiDAR�̓_�Q�̎ʑ��摜�̏����ƃe�L�X�g�t�@�C���̏���
int remove_image_text() {

	int start, goal;

	//�t�H���_�[���̎w��
	std::cout << "Filepath name = " << std::endl;

	std::string filepath;
	std::cin >> filepath;


	//�t�@�C���̎n�߂ƏI�����w��
	std::cout << "File Start Number = " << std::endl;
	std::cin >> start;

	std::cout << "File Goal Number = " << std::endl;
	std::cin >> goal;



	for (int i = start; i <= goal; i++)
	{

		std::string img_filename = filepath + "\\image" + std::to_string(i) + ".png_PnP_projected_points.png";
		std::string text_filename = filepath + "\\image" + std::to_string(i) + ".png_ZED_calculated_stereo_point_cloud.txt";


		//���̃t�@�C��
		std::string next_img_filename = filepath + "\\image" + std::to_string(i + 1) + ".png_PnP_projected_points.png";
		std::string next_text_filename = filepath + "\\image" + std::to_string(i + 1) + ".png_ZED_calculated_stereo_point_cloud.txt";



		//�����Ci�Ԗڂ̃t�@�C�����Ȃ��Ȃ�C�X���[����D
		if ((img_filename.empty()) || (text_filename.empty()))
		{
			continue;
		}



		//�G���[����(����(=0)�łȂ���)
		//�摜�̏���
		//�e�L�X�g�t�@�C���̏���
		if (remove(img_filename.c_str()) != 0 && remove(text_filename.c_str()) != 0)
		{
			std::cout << "Could not remove file No" + std::to_string(i) << std::endl;
		}
		else
		{

			std::cout << "Remove image" + std::to_string(i) + ".png_PnP_projected_points.png" << std::endl;

			std::cout << "Remove image" + std::to_string(i) + ".png_ZED_calculated_stereo_point_cloud.txt" << std::endl;

		}






		//�t�@�C�����I���������̏���
		if (next_img_filename.empty() || next_text_filename.empty())
		{
			std::cerr << "Finish!!" << std::endl;

			return -1;

		}


	}

}











//YOLO�̃o�E���f�B���O�{�b�N�X���ɂ���LiDAR�_�Q�̋��������߂�(����)     int yolo_lidar()



int yolo_lidar() {

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








	//YOLO�̔F�����ʃt�@�C���̓ǂݍ���
	std::cout << "File path : Yolo Bounding Box Info = " << std::endl;
	string sFilePath_YOLO_BoundingBox;
	cin >> sFilePath_YOLO_BoundingBox;


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






	//LiDAR�̓_�Q�f�[�^�̃e�L�X�g�t�@�C���̃p�X��W�����͂��邽�߂ɗp��
	std::cout << "File path : LiDAR Point Cloud Info = " << std::endl;
	string sFilePath_LiDAR_PointCloud;
	cin >> sFilePath_LiDAR_PointCloud;








	int n_yolo_bb_file_stream_size; //�e�s�̕����̐�


									//�e�L�X�g�t�@�C�����̊e1�s�ڂ̕������i�[���邽�߂ɗp��
	char yolo_bb_file_BUFFER[YOLO_FILE_LINE_MAX_BUFFER_SIZE]; //1024






	std::cout << "Files for storing text file�iper_sheet...�j = " << std::endl;

	std::string per_yolo_lidar_distance_result_data_path;
	std::cin >> per_yolo_lidar_distance_result_data_path;

	////////////////////////////////////////////////////////////////////////////



	//�摜����͂��邽�߂Ƀt�@�C���p�X���w��
	std::cout << "input images = " << std::endl;
	std::string in_put_image_file_path;
	std::cin >> in_put_image_file_path;

	//�摜���o�͂��邽�߂Ƀt�@�C���p�X���w��
	std::cout << "Files for storing images = " << std::endl;
	std::string out_put_image_file_path;
	std::cin >> out_put_image_file_path;



	//�q�X�g�O�����̌��ʁi�e�L�X�g�t�@�C���j
	std::cout << "Folder for storing the text file of the histogram results = " << std::endl;
	std::string histgram_textfile_path;
	std::cin >> histgram_textfile_path;





	Mat img_in;
	Mat img_in_left;
	Mat img_out;
	std::string img_out_name;





	//�S�̂̉摜�ɑ΂��Ă̏���
	while (1)
	{
		//fprintf_s(yolo_lidar_distance_result_data, "%d\n", i); //�����ڂ̉摜�Ȃ̂���������悤�ɁC�o�͗p�̃e�L�X�g�t�@�C���ɖ����������Ă���
		//fprintf_s(yolo_result_textfile_path, "%d\n", i); //YOLO�̌��ʂ̃e�L�X�g�t�@�C���ɔԍ��������Ă���




		////////////////////////////////////////////////////////////////////////////

		//addition date is 2020_11_27 
		//1��LiDAR�̃f�[�^���ƂɃe�L�X�g�t�@�C���ɕۑ����邽�߂Ƀt�@�C����p��
		std::string per_yolo_lidar_distance_result_data_path_name;
		per_yolo_lidar_distance_result_data_path_name = per_yolo_lidar_distance_result_data_path + "/YOLO_LiDAR_distance_result_data" + std::to_string(i_yolo_lidar) + ".txt";


		//�t�@�C�����J��
		std::ofstream per_yolo_lidar_distance_result_data;
		per_yolo_lidar_distance_result_data.open(per_yolo_lidar_distance_result_data_path_name);

		if (!per_yolo_lidar_distance_result_data.is_open())
		{
			std::cerr << "Can not open " + per_yolo_lidar_distance_result_data_path_name << std::endl;
			return -1;
		}

		//fopen_s(&per_yolo_lidar_distance_result_data, per_yolo_lidar_distance_result_data_path_name.c_str(), "wt");

		////////////////////////////////////////////////////////////////////////////




		//12_1
		//���͉摜��p��
		////////////////////////////////////////////////////////////////////////////
		std::string img_in_name;
		img_in_name = in_put_image_file_path + "/image" + std::to_string(i_yolo_lidar) + ".png";
		img_in = cv::imread(img_in_name);

		//�X�e���I�摜�̔��������g��������
		//img_in_left = img_in(cv::Rect(0, 0, img_in.cols / 2, img_in.rows));
		//img_out = img_in_left.clone();


		//�X�e���I�摜�̏������I�������̉摜���g��������
		img_out = img_in.clone();
		////////////////////////////////////////////////////////////////////////////







		//1�����̉摜�ɑ΂��Ă̏���
		while (1)
		{

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













			//������
			n_yolo_bb_file_stream_size = 0;



			//�����C�e�L�X�g�t�@�C�����ɕ���������Ȃ�
			if (fgets(yolo_bb_file_BUFFER, YOLO_FILE_LINE_MAX_BUFFER_SIZE, fp_yolo_bb) != NULL)
			{

				n_yolo_bb_file_stream_size = (int)strlen(yolo_bb_file_BUFFER);



				if (n_yolo_bb_file_stream_size == 2)
				{

					//memory clear
					point_cloud.clear();                                  point_cloud.shrink_to_fit();
					yolo_lidar_inlier_pointcloud.clear();                 yolo_lidar_inlier_pointcloud.shrink_to_fit();
					point_cloud_LiDAR_yzx.clear();                        point_cloud_LiDAR_yzx.shrink_to_fit();
					all_lidar_distance.clear();                           all_lidar_distance.shrink_to_fit();
					imagePoints_LiDAR2ZED.clear();                        imagePoints_LiDAR2ZED.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_3D.clear();          all_point_cloud_in_a_boundingbox_3D.shrink_to_fit();
					all_lidar_distance_in_boundingbox.clear();            all_lidar_distance_in_boundingbox.shrink_to_fit();

					all_point_cloud_in_a_boundingbox_container.clear();   all_point_cloud_in_a_boundingbox_container.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_person.clear();      all_point_cloud_in_a_boundingbox_person.shrink_to_fit();

					all_hist.clear();                                     all_hist.shrink_to_fit(); //Clear the histogram data




					i_yolo_lidar++; //����LiDAR�̓_�Q�̍��W�������ꂽ�e�L�X�g�t�@�C����ǂݍ���

					std::cout << i_yolo_lidar << std::endl; //�����ڂ���\�����Ă���


					break; //while�����甲����

				}
				else
				{


					sscanf_s(yolo_bb_file_BUFFER, "%s\t%lf\t%d\t%d\t%d\t%d", &yolo_buf_bbox.name, YOLO_LABEL_STRING_MAX_LENGTH, &yolo_buf_bbox.likelihood, &yolo_buf_bbox.x, &yolo_buf_bbox.y, &yolo_buf_bbox.width, &yolo_buf_bbox.height);






					//�����o����YOLO�̃f�[�^���e�L�X�g�t�@�C���ɕۑ�����D
					//fprintf_s(yolo_result_textfile_path, "%s\t%d\t%d\t%d\t%d\n", yolo_buf_bbox.name, yolo_buf_bbox.x, yolo_buf_bbox.y, yolo_buf_bbox.width, yolo_buf_bbox.height);


					//YOLO����LiDAR�̌��ʂ��������ރt�@�C���ɁwYOLO�̌��ʁx����������ł���
					//fprintf_s(yolo_lidar_distance_result_data, "%s\t%d\t%d\t%d\t%d\n", yolo_buf_bbox.name, yolo_buf_bbox.x, yolo_buf_bbox.y, yolo_buf_bbox.width, yolo_buf_bbox.height);





					////////////////////////////////////////////////////////////////////////////

					//addition date is 2020_11_27 ���@2021_1_6 �R�����g�A�E�g
					//�t�@�C�����ƂɃo�E���f�B���O�{�b�N�X����LiDAR�̌��ʂ�ۑ�
					//per_yolo_lidar_distance_result_data << yolo_buf_bbox.name << "\t" << yolo_buf_bbox.x << "\t" << yolo_buf_bbox.y << "\t" << yolo_buf_bbox.width << "\t" << yolo_buf_bbox.height << "\n";






					//fprintf_s(per_yolo_lidar_distance_result_data, "%s\t%d\t%d\t%d\t%d\n", yolo_buf_bbox.name, yolo_buf_bbox.x, yolo_buf_bbox.y, yolo_buf_bbox.width, yolo_buf_bbox.height);

					////////////////////////////////////////////////////////////////////////////




					//transporting��not_transporting�ŐF��ς���
					not_transporting = strstr(yolo_bb_file_BUFFER, "not");
					transporting = strstr(yolo_bb_file_BUFFER, "transporting");
					container = strstr(yolo_bb_file_BUFFER, "container");
					person = strstr(yolo_bb_file_BUFFER, "person");




					if (not_transporting)
					{
						cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(0, 255, 255), 3);

						//�摜�ɕ�����`��(https://swallow-incubate.com/archives/blog/20190118/)
						cv::putText(img_out, "not_transporting", cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
					}
					else if (transporting)
					{
						cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(255, 0, 0), 3);

						cv::putText(img_out, "transporting", cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

					}
					else
					{
						cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(0, 0, 255), 1);

					}



					//12_17�ǋL
					//�k�������o�E���f�B���O�{�b�N�X��`��
					//cv::rectangle(img_out, cv::Point((int)((double)yolo_buf_bbox.x + (double)yolo_buf_bbox.width / denominator), (int)((double)yolo_buf_bbox.y + (double)yolo_buf_bbox.height / denominator)), cv::Point((int)((double)yolo_buf_bbox.x + numerator * (double)yolo_buf_bbox.width / denominator), (int)((double)yolo_buf_bbox.y + numerator * (double)yolo_buf_bbox.height / denominator)), cv::Scalar(0, 0, 0), 1);





					//12_1
					////////////////////////////////////////////////////////////////////////////
					//�摜����YOLO�̌��ʂ𔽉f������

					//cv::rectangle(img_out, cv::Point(int(yolo_buf_bbox.x), int(yolo_buf_bbox.y)), cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width), int(yolo_buf_bbox.y + yolo_buf_bbox.height)), cv::Scalar(0, 0, 255), 1);

					////////////////////////////////////////////////////////////////////////////








					//LiDAR






					//1�s���i�[���߂̃o�b�t�@�t�@�C����p��
					std::string lidar_textfile_line_BUFFER;




					//YOLO�̃o�E���f�B���O�{�b�N�X���́wLiDAR�x�̓_�Q���擾
					while (1)
					{

						//	if (getline(lidar_text_file, lidar_textfile_line_BUFFER))
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

					} //�wYOLO�̃o�E���f�B���O�{�b�N�X���́wLiDAR�x�̓_�Q���擾�x�I��(while��)







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










					//�v�b�V���o�b�N����point_cloud�̃f�[�^���e�L�X�g�t�@�C���ɏ�������ł���
					for (int j = 0; j < (int)imagePoints_LiDAR2ZED.size(); j++)
					{

						if (point_cloud_LiDAR_yzx[j].z < 0) continue; //
						if (imagePoints_LiDAR2ZED[j].x < 0 || imagePoints_LiDAR2ZED[j].x >= IMG_XSIZE || imagePoints_LiDAR2ZED[j].y < 0 || imagePoints_LiDAR2ZED[j].y >= IMG_YSIZE) continue;



						//YOLO�̃o�E���f�B���O�{�b�N�X���ł̏���

						//LiDAR�̓_�Q��YOLO�̃o�E���f�B���O�{�b�N�X���̎�
						if ((imagePoints_LiDAR2ZED[j].x >= yolo_buf_bbox.x && imagePoints_LiDAR2ZED[j].x <= yolo_buf_bbox.x + yolo_buf_bbox.width) && (imagePoints_LiDAR2ZED[j].y >= yolo_buf_bbox.y && imagePoints_LiDAR2ZED[j].y <= yolo_buf_bbox.y + yolo_buf_bbox.height))
						{

							//YOLO����LiDAR�̌��ʂ��������ރt�@�C���ɁwLiDAR�̌��ʁx����������ł���
							//fprintf_s(yolo_lidar_distance_result_data, "%lf\t%lf\t%lf\n", point_cloud[j].x, point_cloud[j].y, point_cloud[j].z);



							//�o�E���f�B���O�{�b�N�X����LiDAR�̓_�Q(2����)
							yolo_lidar_inlier_point.x = imagePoints_LiDAR2ZED[j].x;
							yolo_lidar_inlier_point.y = imagePoints_LiDAR2ZED[j].y;

							yolo_lidar_inlier_pointcloud.push_back(yolo_lidar_inlier_point);




							////////////////////////////////////////////////////////////////////////////////////////////////
							if (container)
							{
								point_cloud_in_a_boundingbox_container.x = point_cloud_LiDAR_yzx[j].x;
								point_cloud_in_a_boundingbox_container.y = point_cloud_LiDAR_yzx[j].y;
								point_cloud_in_a_boundingbox_container.z = point_cloud_LiDAR_yzx[j].z;

								all_point_cloud_in_a_boundingbox_container.push_back(point_cloud_in_a_boundingbox_container);



								point_cloud_in_a_boundingbox_person.x = 0.0;
								point_cloud_in_a_boundingbox_person.y = 0.0;
								point_cloud_in_a_boundingbox_person.z = 0.0;

								all_point_cloud_in_a_boundingbox_person.push_back(point_cloud_in_a_boundingbox_person);

							}
							else if (person)
							{
								point_cloud_in_a_boundingbox_person.x = point_cloud_LiDAR_yzx[j].x;
								point_cloud_in_a_boundingbox_person.y = point_cloud_LiDAR_yzx[j].y;
								point_cloud_in_a_boundingbox_person.z = point_cloud_LiDAR_yzx[j].z;

								all_point_cloud_in_a_boundingbox_person.push_back(point_cloud_in_a_boundingbox_person);



								point_cloud_in_a_boundingbox_container.x = 0.0;
								point_cloud_in_a_boundingbox_container.y = 0.0;
								point_cloud_in_a_boundingbox_container.z = 0.0;


								all_point_cloud_in_a_boundingbox_container.push_back(point_cloud_in_a_boundingbox_container);

							}
							else
							{
								point_cloud_in_a_boundingbox_3D.x = point_cloud_LiDAR_yzx[j].x;
								point_cloud_in_a_boundingbox_3D.y = point_cloud_LiDAR_yzx[j].y;
								point_cloud_in_a_boundingbox_3D.z = point_cloud_LiDAR_yzx[j].z;


								all_point_cloud_in_a_boundingbox_3D.push_back(point_cloud_in_a_boundingbox_3D);








								point_cloud_in_a_boundingbox_person.x = 0.0;
								point_cloud_in_a_boundingbox_person.y = 0.0;
								point_cloud_in_a_boundingbox_person.z = 0.0;

								all_point_cloud_in_a_boundingbox_person.push_back(point_cloud_in_a_boundingbox_person);


								point_cloud_in_a_boundingbox_container.x = 0.0;
								point_cloud_in_a_boundingbox_container.y = 0.0;
								point_cloud_in_a_boundingbox_container.z = 0.0;

								all_point_cloud_in_a_boundingbox_container.push_back(point_cloud_in_a_boundingbox_container);
							}
							////////////////////////////////////////////////////////////////////////////////////////////////








							lidar_distance_in_boundingbox = all_lidar_distance[j];

							all_lidar_distance_in_boundingbox.push_back(lidar_distance_in_boundingbox);


							////////////////////////////////////////////////////////////////////////////

							//�t�@�C�����ƂɃo�E���f�B���O�{�b�N�X����LiDAR�̌��ʂ�ۑ�
							//addition date is 2020_11_27 
							per_yolo_lidar_distance_result_data << point_cloud_LiDAR_yzx[j].x << "\t" << point_cloud_LiDAR_yzx[j].y << "\t" << point_cloud_LiDAR_yzx[j].z << "\n";


							//fprintf_s(per_yolo_lidar_distance_result_data, "%lf\t%lf\t%lf\n", point_cloud[j].x, point_cloud[j].y, point_cloud[j].z);

							////////////////////////////////////////////////////////////////////////////








							//2020_12_1
							////////////////////////////////////////////////////////////////////////////


							//LiDAR�̓_�Q���摜�ɏo��
							cv::circle(img_out, cv::Point((int)imagePoints_LiDAR2ZED[j].x, (int)imagePoints_LiDAR2ZED[j].y), 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);


							////////////////////////////////////////////////////////////////////////////



						}


					}



					//2021_1_6�ǋL
					//YOLO_LiDAR_distance_result_data�ɉ��s��2�s�ǉ�
					per_yolo_lidar_distance_result_data << "\n" << "\n";




					//2021_1_12�ǋL
					////////////////////////////////////////////////////////////////////////////
					//Calculate the median
					make_histgram(histgram_textfile_path);





					//Bounding box����LiDAR�_�Q��gnuplot�ɕ\���i2021_1_6�ǋL�j (2�ڂ�gnuplot_mapping)
					gnuplot_mapping();



					//���̂܂ł̋�����\��
					if (not_transporting)
					{
						//cv::putText(img_out, std::to_string(int(a_median)) + "m", cv::Point(int(yolo_buf_bbox.x - 40), int(yolo_buf_bbox.y)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
						continue;
					}
					else if (transporting)
					{
						//cv::putText(img_out, std::to_string(int(a_median)) + "m", cv::Point(int(yolo_buf_bbox.x - 40), int(yolo_buf_bbox.y)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
						continue;
					}
					else
					{
						cv::putText(img_out, std::to_string(int(a_median)) + "m", cv::Point(int(yolo_buf_bbox.x + yolo_buf_bbox.width + 5), int(yolo_buf_bbox.y)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

					}


					////////////////////////////////////////////////////////////////////////////




					//memory clear
					//point_cloud.clear();                                  point_cloud.shrink_to_fit();
					//yolo_lidar_inlier_pointcloud.clear();                 yolo_lidar_inlier_pointcloud.shrink_to_fit();
					//point_cloud_LiDAR_yzx.clear();                        point_cloud_LiDAR_yzx.shrink_to_fit();
					//all_lidar_distance.clear();                           all_lidar_distance.shrink_to_fit();
					//imagePoints_LiDAR2ZED.clear();                        imagePoints_LiDAR2ZED.shrink_to_fit();
					//all_point_cloud_in_a_boundingbox_3D.clear();          all_point_cloud_in_a_boundingbox_3D.shrink_to_fit();
					//all_lidar_distance_in_boundingbox.clear();            all_lidar_distance_in_boundingbox.shrink_to_fit();

					//all_point_cloud_in_a_boundingbox_container.clear();   all_point_cloud_in_a_boundingbox_container.shrink_to_fit();


				} //else ���s�̕�������2�ȊO�Ȃ�

			} //�w�����C�e�L�X�g�t�@�C�����ɕ���������Ȃ�x�I��(if��)

			else //�����C�e�L�X�g�t�@�C���̍Ō�܂ł����΁c
			{
				printf_s("Detected the end of file!!\n");


				fclose(fp_yolo_bb); //YOLO�̃e�L�X�g�t�@�C�������


				break;
			}




			//fprintf_s(yolo_lidar_distance_result_data, "-\n"); //�F���������̂��Ƃɉ��s����(�o�E���f�B���O�{�b�N�X����LiDAR�̓_�Q�̌��ʂ��o�͂����t�@�C��)



		} //1���̉摜�ɑ΂���YOLO�̔F�����ʂ̏I���



		//����̏o��
		img_out_name = out_put_image_file_path + "/image" + std::to_string(i_yolo_lidar - 1) + ".png";
		cv::imwrite(img_out_name, img_out);


		per_yolo_lidar_distance_result_data.close();
		//fclose(per_yolo_lidar_distance_result_data);



		//�����CYOLO�̃e�L�X�g�t�@�C���̍s�̕�������0�Ȃ�c(���e�L�X�g�t�@�C������S�ĊJ���I������c)
		if (n_yolo_bb_file_stream_size == 0)
		{

			//fclose(fp_yolo_bb); //YOLO�̃e�L�X�g�t�@�C�������


			break; //while�����甲����

		}




		//Bounding box����LiDAR�_�Q��gnuplot�ɕ\���i2021_1_6�ǋL�j (2�ڂ�gnuplot_mapping)�����Ԃ�g��Ȃ�
		//gnuplot_mapping();





		//fprintf_s(yolo_result_textfile_path, "--\n"); //YOLO�̌��ʂɑ΂��ĉ��s����


		//fprintf_s(yolo_lidar_distance_result_data, "\n"); //�o�E���f�B���O�{�b�N�X����liDAR�_�Q�̉摜1�����Ƃɉ��s����






	} //�S�Ẳ摜�ɑ΂���YOLO�̔F�����ʂ̏I���



	  //fclose(yolo_result_textfile_path);
	  //fclose(yolo_lidar_distance_result_data);
	  //fclose(fp_yolo_bb);





	std::cout << "\n" << std::endl;
	//printf_s("\n");



	std::cout << "OK!!" << std::endl;


	//n_flag_thread_gnuplot_exit = 1; //gnuplot �I��
	//CloseHandle(hThread_gnuplot);

	return 0;

}








//Calculate the median
double make_histgram(std::string outputfile_path)
{

	//�e�L�X�g�t�@�C����p��
	std::string outputfile_name = outputfile_path + "\\yolo_distance_histgram_image_count_" + std::to_string(i_yolo_lidar) + "_" + yolo_buf_bbox.name + "_" + std::to_string(n_class) + ".txt";



	//C++
	//�o�͗p
	std::ofstream outputfile;
	outputfile.open(outputfile_name);

	if (!outputfile.is_open())
	{
		std::cerr << "Can not open " + outputfile_name << std::endl;
		return -1;
	}



	int z_class = 0;




	//initialization
	for (int a = 0; a < hist_array; a++)
	{
		hist[a] = 0;
	}


	//make a histgram
	for (int a = 0; a < (int)all_lidar_distance_in_boundingbox.size(); a++)
	{
		//fractional part
		double z_fractional_part = all_lidar_distance_in_boundingbox[a] - int(all_lidar_distance_in_boundingbox[a]);


		//classification(Rounding)
		if (z_fractional_part >= 0.5)
		{
			z_class = int(all_lidar_distance_in_boundingbox[a] + 1.0); //��)8.8��9
		}
		else
		{
			z_class = int(all_lidar_distance_in_boundingbox[a]);  //��)8.4 ���@8
		}

		hist[z_class]++;

	}


	all_hist.push_back(hist[z_class]);




	//�e�L�X�g�t�@�C���ɏo��
	for (int a = 0; a < hist_array; a++)
	{
		//C++
		outputfile << a << "\t" << hist[a] << "\n";
	}


	//C++
	outputfile.close();





	//calculate the maximum value
	median_distance(hist);


	//�q�X�g�O�����̍ő�l��\������(i_max�̓��[���h��ԂŒ�`)
	//std::cout << "i max = " << i_max << std::endl;




	n_class++;



	return a_median;
}




//�q�X�g�O�����̍ő�̋��������߂�
double median_distance(int hist[])
{
	int tmp;

	int median_hist;


	a_median = 0.0;


	//Sort in descending order
	for (int a = 0; a < hist_array; a++)
	{
		for (int b = a + 1; b < hist_array; b++)
		{
			if (hist[a] < hist[b])
			{
				tmp = hist[b];
				hist[b] = hist[a];
				hist[a] = hist[b];
			}

		}

	}




	if (hist_array % 2 == 1)
	{
		median_hist = hist[all_hist.size() / 2];
		a_median = (double)all_hist.size() / 2.0;

		return a_median;
	}
	else
	{
		median_hist = (hist[((int)all_hist.size() / 2) - 1] + hist[(int)all_hist.size() / 2]) / 2;
		a_median = (double)(((double)all_hist.size() / 2.0 - 1.0) + ((double)all_hist.size() / 2.0)) / 2.0;

		return a_median;
	}


}







//Bounding box����LiDAR�̓_�Q���}�b�s���O�ignuplot�j(http://nalab.mind.meiji.ac.jp/~mk/labo/howto/intro-gnuplot/node24.html)

int gnuplot_mapping()
{

	FILE *gid;

	gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");

	//fprintf(gid, "set size ratio -1\n");
	fprintf(gid, "set xrange[-20:20]\n");
	fprintf(gid, "set yrange[0:40]\n");

	fprintf(gid, "set xlabel 'x[m]'\n");
	fprintf(gid, "set ylabel 'z[m]'\n");




	//gnuplot�̌��ʂ�ۑ��ihttp://www.gnuplot-cmd.com/in-out/output.html�j
	fprintf(gid, "set terminal png\n");







	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//�s�x�C�ύX����I�I

	fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!result_!YOLO_MASUDA3_20000_ver3\\mapping_image_20201117_low\\image%d.png'\n", i_yolo_lidar);
	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!result_!YOLO_MASUDA3_20000_ver3\\mapping_image_20201223112457\\image%d.png'\n", i_yolo_lidar);
	//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!result_!YOLO_MASUDA3_20000_ver3\\mapping_image_20201223113541\\image%d.png'\n", i_yolo_lidar);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






	//fprintf_s(gid, "plot '-' index 0,'' index 1,'' index 2,'' index 3,'' index 4,'' index 5 with points ps 1\n");
	fprintf_s(gid, "plot '-' with points ps 1 lc rgb 'red' title 'Container in bounding box', '-' with points ps 1 lc rgb 'blue' title 'Person in bounding box'\n");



	for (int a = 0; a < (int)all_point_cloud_in_a_boundingbox_container.size(); a++)
	{

		fprintf(gid, "%f\t%f\n", all_point_cloud_in_a_boundingbox_container[a].x, all_point_cloud_in_a_boundingbox_container[a].z);
	}


	fprintf(gid, "e\n");


	for (int a = 0; a < (int)all_point_cloud_in_a_boundingbox_person.size(); a++)
	{
		fprintf(gid, "%f\t%f\n", all_point_cloud_in_a_boundingbox_person[a].x, all_point_cloud_in_a_boundingbox_person[a].z);

	}


	fprintf(gid, "set terminal windows\n");
	fprintf(gid, "set output\n");



	fprintf(gid, "e\n");
	fflush(gid);
	fprintf(gid, "pause mouse\n");
	fflush(gid);
	_pclose(gid);


	return 0;
}















//Bounding box����LiDAR�̓_�Q���}�b�s���O�ignuplot�j(�X���b�hver)
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
//gnuplot�̌��ʂ�ۑ��ihttp://www.gnuplot-cmd.com/in-out/output.html�j
fprintf(gid, "set terminal png\n");
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//�ۑ��t�H���_�[����ύX����I//
//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201117_low\\image%d.png'\n", i_yolo_lidar);
//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201223112457\\image%d.png'\n", i_yolo_lidar);
//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201223113541\\image%d.png'\n", i_yolo_lidar);
//copy�����t�H���_�i�q�X�g�O�����̃e�X�g�p�j
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
//copy�����t�H���_�i�q�X�g�O�����̃e�X�g�p�j
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








//gnuplot�Ńq�X�g�O������\��
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

	//�����x��
	fprintf(gid, "set xlabel 'distance[m]' \n");
	fprintf(gid, "set ylabel 'count' \n");


	//�ڐ��̊Ԋu
	//fprintf(gid, "set xtics 5");
	fprintf(gid, "set ytics 1\n");


	//�ڐ��̍ő�l
	fprintf(gid, "set xrange[0:50]\n");


	//�}��̈ʒu
	fprintf(gid, "set key right top\n");

	//�}��̃^�C�g���ƃO���t�̌`��
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













//������̓��̂̍��������߂�@+�@�\���y�r���z

/*
int differences_in_the_movie()
{
//����̓ǂݍ���
std::cout << "Movie name = " << std::endl;
std::string movie_name;
std::cin >> movie_name;
VideoCapture cap(movie_name);
//����̕ۑ�
std::cout << "Folder path for saving = " << std::endl;
std::string saving_folder_path;
std::cin >> saving_folder_path;
//�ۑ�����ۂ̃t���[�������w��
std::cout << "Number of frames=" << std::endl;
int n;
std::cin >> n;
VideoWriter writer(saving_folder_path + "\\differences_in_the_movie.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), n, Size(672, 376));
// �w�i�����v�Z�p�I�u�W�F�N�g�̐���
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





//Histogramming YOLO point clouds in a bounding box�i�ŏ��ɍ�������́j�����Ԃ�C�g��Ȃ�

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
//object��
char *obj1, *obj2, *obj3;
obj1 = strstr(one_line_BUFFER, "person"); //�wstrstr�x�ŁC�w�肵�������������
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
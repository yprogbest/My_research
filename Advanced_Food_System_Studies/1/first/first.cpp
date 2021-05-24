#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //�e�L�X�g�t�@�C�����������߂ɗp��
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


#include <atltime.h> //�������Ԃ��v�Z���邽�߂ɗp��





//�v���g�^�C�v�錾
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



using namespace cv;
using namespace std;


Rect2i rectangle_value;


int main(int argc, const char* argv[])
{
	int n = 0; //�摜���瓮����쐬����ۂ̃t���[����
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
				//����̕ۑ�
				disply_movie();

				break;



			// �G���[���o�Ď��s�ł��Ȃ�
			case 2:
				track_code = tracker_main();

				break;

			case 3:
				//RGB�l��HSV�l�̏����o��
				result_RGB_HSV();

				break;


			case 4:
				//�猟�o
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


			case 99:
				nFlag = -1;
				exit(1);


		}

	}


}





//���j���[��ʂ̊֐�
int menu_screen()
{
	int ncommand;

	printf("\n");
	printf("----------------------------------------------------\n");
	printf("<<0>>:���Ɨp\n");
	printf("<<1>>:����̕ۑ�\n");
	printf("<<2>>:���̒ǐ�\n");
	printf("<<3>>:RGB�l�̏����o��\n");
	printf("<<4>>:�猟�o\n");
	printf("<<5>>:�������o\n");
	printf("<<6>>:�e���v���[�g�}�b�`���O\n");
	printf("<<7>>:Hough�ϊ�\n");
	printf("<<8>>:Hough�ϊ��y�~ver�z\n");
	printf("<<9>>:����\n");
	printf("<99>>:�I�����܂��D\n");
	printf("----------------------------------------------------\n");
	printf("\n");

	printf("command=");
	scanf_s("%d", &ncommand);

	return ncommand;

}




//���Ɨp
int lecture(void)
{



	return 0;
}




//��2�����(����̕\��+�ۑ�)
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


		//�O���C��



		cv::imshow("image", frame);


		//���悩��摜�����o���C�t�H���_�ɕۑ�
		image_name = "D:\\M1\\Advanced_Food_System_Studies\\1\\image\\image" + std::to_string(i) + ".png";
		cv::imwrite(image_name, frame);


		//�����ۑ�
		writer << frame;


		if (waitKey(30) >= 0) break;



		i++;

	}


	cap.release();//�ǂݍ��ݗp�̃I�u�W�F�N�g�����
	writer.release();//�������ݗp�̃I�u�W�F�N�g�����
	destroyAllWindows();//�E�B���h�E��j��


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



		//�X�V
		tracker->update(frame, roi);

		//���ʂ�\��
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



		//�ۑ���̃e�L�X�g�t�@�C��
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

				
				of_rgb << "B =" << r << "\t" << "G=" << g << "\t" << "R=" << b << "\n";//�e�L�X�g�t�@�C���ɏ����o��
			
			
			}
		}


		of_rgb.close();

		std::cout << std::to_string(i) + "�Ԗڊ���" << std::endl;


		i++;


		if (img.empty())
		{
			break;
		}
	}


	return 0;
}




//��4��
//������o���āC�t�@�C���ɕۑ�
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


	//��F���p�̃J�X�P�[�h��p��
	std::string cascade_path = "C:\\opencv-3.4.10\\opencv-3.4.10\\sources\\samples\\winrt\\FaceDetection\\FaceDetection\\Assets\\haarcascade_frontalface_alt.xml";
	cv::CascadeClassifier cascade;
	cascade.load(cascade_path);


	namedWindow("image", WINDOW_AUTOSIZE);
	//namedWindow("image_rect", WINDOW_AUTOSIZE);
	



	while (1)
	{

		cap >> frame;


		// �O���C�X�P�[��
		cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);


		
		std::vector<cv::Rect> faces;

		// frame_gray������T��
		cascade.detectMultiScale(frame_gray, faces);


		for (int i = 0; i < faces.size(); i++)
		{

			x_start = faces[i].x;
			y_start = faces[i].y;
			x_end = faces[i].x + faces[i].width;
			y_end = faces[i].y + faces[i].height;
			width = faces[i].width;;
			height = faces[i].height;

			// �Ԃ������`�� frame �ɕ`��
			cv::rectangle(frame, cv::Point(x_start, y_start), cv::Point(x_end, y_end), cv::Scalar(0, 0, 255), 2);


			// ��݂̂����o��
			frame_rect = Mat(frame, Rect(x_start, y_start, width, height));

			
		}



		if (!frame.empty())
		{
			// frame��\��
			cv::imshow("image", frame);

			if (!frame_rect.empty())
			{
				//���T�C�Y
				cv::resize(frame_rect, frame_rect, Point(frame.cols, frame.rows));

				// frame_rect��\��
				//cv::imshow("image_rect", frame_rect);

				//���悩��摜�����o���C�t�H���_�ɕۑ�
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


	cap.release();//�ǂݍ��ݗp�̃I�u�W�F�N�g�����
	cv::destroyAllWindows();//�E�B���h�E��j��

	return 0;
}






//��3�����(���̒ǐՁi�t���[���ԍ����j)
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


	//3�t���[�����ǂݍ���
	cap >> frame;
	cv::cvtColor(frame, gray_frame1, cv::COLOR_BGR2GRAY);//�O���C�X�P�[����
	
	cap >> frame;
	cv::cvtColor(frame, gray_frame2, cv::COLOR_BGR2GRAY);

	cap >> frame;
	cv::cvtColor(frame, gray_frame3, cv::COLOR_BGR2GRAY);
	



	cout << "start" << endl;

	while (1)
	{
		//�t���[���̍������߂�
		cv::absdiff(gray_frame1, gray_frame2, diff1);
		cv::absdiff(gray_frame2, gray_frame3, diff2);

		//����1�ƍ���2�̌��ʂ��r�i�_���ρj
		cv::bitwise_and(diff1, diff2, diff);

		cv::namedWindow("diff image", cv::WINDOW_AUTOSIZE);
		imshow("diff image", diff);


		//�֊s�𒊏o
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

			//�����ȍ����͎�菜��
			int w = x_max - x_min;
			int h = y_max - y_min;
			if (w > 30 && h > 30)
			{
				cv::rectangle(frame, Point(x_min, y_min), Point(x_max, y_max), green, 2); //�l�p�ň͂�
			}


		}


		//�\��
		cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
		cv::imshow("result", frame);


		//�摜��1�t���[�����炷
		gray_frame2.copyTo(gray_frame1, gray_frame2); //�t���[��2���t���[��1��
		gray_frame3.copyTo(gray_frame2, gray_frame3); //�t���[��3���t���[��2��
		cap >> frame;
		cv::cvtColor(frame, gray_frame3, cv::COLOR_BGR2GRAY);
		


		if (waitKey(30) >= 0) break;
	}


	cap.release();//�ǂݍ��ݗp�̃I�u�W�F�N�g�����
	cv::destroyAllWindows();//�E�B���h�E��j��


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
//	//3�t���[�����ǂݍ���
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
//		//�t���[���̍������߂�
//		cv::absdiff(gray_frame1, gray_frame2, diff1);
//		cv::absdiff(gray_frame2, gray_frame3, diff2);
//
//		//����1�ƍ���2�̌��ʂ��r�i�_���ρj
//		cv::bitwise_and(diff1, diff2, diff);
//
//		cv::threshold(diff, diff_out, 4, 255, CV_THRESH_BINARY);
//		imshow("diff", diff);
//		imshow("diff_out", diff_out);
//
//		dilate(diff_out, diff_out, 3,Point(-1,-1),3);
//
//		//�֊s�𒊏o
//		cv::findContours(diff_out, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//
//		//�������������_���摜�ɕ`��
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
//		//�\��
//		cv::namedWindow("diff image", cv::WINDOW_NORMAL);
//		cv::imshow("diff image", frame);
//
//		//�摜��1�t���[�����炷
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
//	cap.release();//�ǂݍ��ݗp�̃I�u�W�F�N�g�����
//	cv::destroyAllWindows();//�E�B���h�E��j��
//
//
//	return 0;
//}







//��5����Ɓ~
//�e���v���[�g�}�b�`���O(����ł͏�肭�����Ȃ�)
//https://qiita.com/appin/items/0bb42ef108b49e9f72c3
//https://qiita.com/satsukiya/items/c8828f48be7673007007 
//http://imgprolab.sys.fit.ac.jp/~yama/imgproc/proc/Document_OpenCVforC_8_2017.pdf (���Q�l�L��)

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

		// ���{�^���������ꂽ��`��J�n
		if (isClick == true) {
			rectangle(draw_img, rectangle_value, Scalar(255, 0, 0), 3, CV_AA);
		}

		imshow(window_name, draw_img);
		draw_img = img.clone();

		// q�L�[�������ꂽ��I��
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

			//�e���v���[�g�}�b�`���O
			cv::matchTemplate(img, last_roi_image, result_img, CV_TM_CCOEFF_NORMED);
			Point max_pt;
			double maxVal;
			cv::minMaxLoc(result_img, NULL, &maxVal, NULL, &max_pt);

			//�T�����ʂ̏ꏊ�ɋ�`��`��
			Rect roi_rect(0, 0, rectangle_value.width, rectangle_value.height);
			roi_rect.x = max_pt.x;
			roi_rect.y = max_pt.y;
			cv::rectangle(img, roi_rect, Scalar(0, 255, 255), 2);

			//���t���[���ł̃e���v���[�g�}�b�`���O�p��ROI��ۑ�
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



//�R�[���o�b�N�֐��i�}�E�X�ő���j
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







//��5����ƁZ
//Hough�ϊ�
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

			//�O���C�X�P�[��
			cv::cvtColor(src, gray_src, COLOR_BGR2GRAY);

			cv::namedWindow("src", 1);
			cv::imshow("src", src);

			cv::namedWindow("gray", 1);
			cv::imshow("gray", gray_src);

			//�֊s���o
			cv::Canny(src, edge, 320, 370);

			cv::namedWindow("edge", 1);
			cv::imshow("edge", edge);

			//�������o
			cv::HoughLinesP(
				edge,// 8�r�b�g�C�V���O���`�����l����2�l���͉摜
				lines, // ���o���ꂽ�������o�͂����x�N�g��
				1,// �s�N�Z���P�ʂł̋�������\
				CV_PI / 180.0,// ���W�A���P�ʂł̊p�x����\
				80,// 臒l.threshold���\���ɒ����Ă��钼���݂̂��o�͑Ώ�
				100,// �ŏ��̐�����
				10 // 2�_�����������ɂ���ƌ��Ȃ��ꍇ�ɋ��e�����ő勗��
			);

			//�����`��
			for (int i = 0; i < lines.size(); i++) {
				cv::line(src, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 2);

				//printf("%d\t%d\t%d\t%d\n", lines[i][0], lines[i][1], lines[i][2], lines[i][3]);
			}


			cv::namedWindow("Hough", 1);
			cv::imshow("Hough", src);



			//4��ʕ\��






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







//��6�����
//https://teratail.com/questions/130841
//https://so-zou.jp/software/tech/library/opencv/retrieval/hough-transform.htm
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

		src_clone = src.clone();

		cvtColor(src_clone, hsv, CV_BGR2HSV); //hsv�ɕϊ�

		//hsv����
		std::vector<cv::Mat>mat_channels;
		cv::split(hsv, mat_channels);

		//���o���F������ 0 - 30 �܂�
		int hue_min = 0;
		int hue_max = hue_min + 30;

		int hsv_value;
		int cols = hsv.cols;
		int rows = hsv.rows;

		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				hsv_value = static_cast<int>(mat_channels[0].at<uchar>(row, col));

				//�F��(Hue)
				if (!((hue_min <= hsv_value) && (hsv_value <= hue_max))) {
					//RGB�l��0�ɂ���
					src_clone.at<cv::Vec3b>(row, col)[0] = 0;
					src_clone.at<cv::Vec3b>(row, col)[1] = 0;
					src_clone.at<cv::Vec3b>(row, col)[2] = 0;
				}
			}
		}



		cvtColor(src_clone, gray, CV_BGR2GRAY); //�O���C�X�P�[��

		GaussianBlur(gray, gray, Size(9, 9), 2, 2);



		HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, 60, 200, 20, 0, 35);

		for (int i = 0; i < circles.size(); i++) {

			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);

			cv::circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);//circle center
			cv::circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);//sircle outline

			std::cout << "center: " << center << "\nradius: " << radius << endl;
		}



		/*namedWindow("src_clone", CV_WINDOW_AUTOSIZE);
		imshow("Hough src_clone", src_clone);*/
		namedWindow("Hough Circle", CV_WINDOW_AUTOSIZE);
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




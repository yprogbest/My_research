#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //テキストファイルを扱うために用意
#include <string>
#include <thread>
#include <process.h>


#include <opencv2/opencv.hpp>//OpenCVのインクルード
#include "opencv2/highgui/highgui.hpp"


//テキストファイル内の各行の可能最大文字数として定義
#define YOLO_FILE_LINE_MAX_BUFFER_SIZE	1024
#define LiDAR_FILE_LINE_MAX_BUFFER_SIZE 1024

//YOLOで学習させる物体の名称を入れるファイル可能最大領域を定義
#define YOLO_LABEL_STRING_MAX_LENGTH  100


#define IMG_XSIZE 672
#define IMG_YSIZE 376



//using宣言
using namespace cv;
using namespace std;


//プロトタイプ宣言
int Get_LidarPointcloud_in_Boundinbbox();
void menu_screen();
unsigned _stdcall thread_gnuplot(void *p);
std::vector<Point3f> make_histgram();
int max_distance(int hist[]);




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




//LiDARのx,y,zの値を読み込むために用意
struct point_cloud
{
	double x;
	double y;
	double z;
	double intensity;
};




//yolo bounding box parameters//
//vector<struct yolo_bounfing_box> yolo_BBox; //1まいの画像結果ごとに追加していくので，配列型（vector）にしておく
struct yolo_bounfing_box yolo_buf_bbox;

// [ 3D ] LiDAR //
std::vector<struct point_cloud>point_cloud;
struct point_cloud point;

double lidar_distance;
std::vector<double>all_lidar_distance;

//Bounding box内のLiDAR点群のみ(2D)//
vector<Point2f> yolo_lidar_inlier_pointcloud;
cv::Point2f yolo_lidar_inlier_point;

//LiDAR 3D → 2D//
vector<Point2f> imagePoints_LiDAR2ZED;

std::vector<Point3f> point_cloud_LiDAR_yzx;
cv::Point3f buf_LiDAR_yzx;

std::vector<Point3f>all_point_cloud_in_a_boundingbox_transporting;
cv::Point3f point_cloud_in_a_boundingbox_transporting;

std::vector<Point3f>all_point_cloud_in_a_boundingbox_person;
cv::Point3f point_cloud_in_a_boundingbox_person;

std::vector<Point3f>all_point_cloud_in_a_boundingbox_container;
cv::Point3f point_cloud_in_a_boundingbox_container;


std::vector<Point3f>all_hist_result_transporting;
cv::Point3f hist_result_transporting;

std::vector<Point3f>all_hist_result_person;
cv::Point3f hist_result_person;

std::vector<Point3f>all_hist_result_container;
cv::Point3f hist_result_container;





int nCommand;


HANDLE hThread_gnuplot;
int n_flag_thread_gnuplot_exit;

//テキストファイル内の各1行目の文字を格納するために用意
char yolo_bb_file_BUFFER[YOLO_FILE_LINE_MAX_BUFFER_SIZE]; //1024

char *transporting;
char *person;
char *container;


const int hist_array = 50;
int hist[hist_array];
int i_max;





int i = 0;





int main()
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

			Get_LidarPointcloud_in_Boundinbbox();

			break;

		case 2:



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
		printf("<<1>>:〇〇\n");
		printf("<<2>>:〇〇\n");
		printf("<<0>>:終了します．\n");
		printf("----------------------------------------------------\n");
		printf("\n");

		printf("command=");
		scanf_s("%d", &nCommand);


	}









int Get_LidarPointcloud_in_Boundinbbox()
{

	//yamlファイルの読み込み(rvecとtvecを使う)

	cv::Mat K1, D1;
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat mat_R_LiDAR2ZED = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::Mat mat_T_LiDAR2ZED = cv::Mat::zeros(3, 1, CV_64FC1);


	std::string sFilePath_PnP_rvec_tvec;

	std::cout << "[PnP PARAMETERS: rvec, tvec ] File path = ";
	std::cin >> sFilePath_PnP_rvec_tvec;




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






	//YOLOの結果のテキストファイルパスを指定

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



	//LiDARの点群座標のテキストファイルパスを指定
	std::cout << "Text file path of LiDAR point cloud = " << std::endl;
	std::string FilePath_Lidar;
	std::cin >> FilePath_Lidar;



	// begin gnuplot thread //
	n_flag_thread_gnuplot_exit = 0;

	hThread_gnuplot = (HANDLE)_beginthreadex(NULL, 0, thread_gnuplot, "gnuplot", 0, NULL); //スレッドの開始



	int n_yolo_bb_file_stream_size; //各行の文字の数



	

	//全体の画像に対しての処理
	while (1)
	{




		//1枚ずつの画像に対しての処理
		while (1)
		{

			//LiDARのテキストファイルを読み込むためのファイルを用意
			std::string sFilePath_LiDAR_point_cloud;
			sFilePath_LiDAR_point_cloud = FilePath_Lidar + "/point_cloud" + std::to_string(i) + ".txt_NaN_data_removed.txt";

			//ファイルを開く
			std::ifstream fp_lidar_point_cloud;
			fp_lidar_point_cloud.open(sFilePath_LiDAR_point_cloud);


			if (!fp_lidar_point_cloud.is_open())
			{
				std::cerr << "Can not open " + sFilePath_LiDAR_point_cloud << std::endl;

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
					point_cloud.clear();                       point_cloud.shrink_to_fit();
					all_lidar_distance.clear();                all_lidar_distance.shrink_to_fit();

					yolo_lidar_inlier_pointcloud.clear();      yolo_lidar_inlier_pointcloud.shrink_to_fit();
					imagePoints_LiDAR2ZED.clear();             imagePoints_LiDAR2ZED.shrink_to_fit();
					point_cloud_LiDAR_yzx.clear();             point_cloud_LiDAR_yzx.shrink_to_fit();

					all_point_cloud_in_a_boundingbox_transporting.clear();  all_point_cloud_in_a_boundingbox_transporting.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_person.clear();        all_point_cloud_in_a_boundingbox_person.shrink_to_fit();
					all_point_cloud_in_a_boundingbox_container.clear();     all_point_cloud_in_a_boundingbox_container.shrink_to_fit();


					all_hist_result_container.clear();                      all_hist_result_container.shrink_to_fit();
					all_hist_result_person.clear();                         all_hist_result_person.shrink_to_fit();
					all_hist_result_transporting.clear();                   all_hist_result_transporting.shrink_to_fit();




					i++; //次のLiDARの点群の座標が書かれたテキストファイルを読み込む

					std::cout << i << std::endl; //何枚目かを表示しておく


					break; //while文から抜ける

				}
				else
				{
					//YOLOのBounding boxのデータをスキャンする
					sscanf_s(yolo_bb_file_BUFFER, "%s\t%lf\t%d\t%d\t%d\t%d", &yolo_buf_bbox.name, YOLO_LABEL_STRING_MAX_LENGTH, &yolo_buf_bbox.likelihood, &yolo_buf_bbox.x, &yolo_buf_bbox.y, &yolo_buf_bbox.width, &yolo_buf_bbox.height);

					//yolo_BBox.push_back(yolo_buf_bbox);


					//std::cout << yolo_buf_bbox.name << "\n";


					container = strstr(yolo_bb_file_BUFFER, "container");
					person = strstr(yolo_bb_file_BUFFER, "person");
					transporting = strstr(yolo_bb_file_BUFFER, "transporting");
					
					




					//LiDAR//


					//1行ずつ格納ためのバッファファイルを用意
					std::string lidar_textfile_line_BUFFER;


					//YOLOのバウンディングボックス内の『LiDAR』の点群を取得
					while (1)
					{
						if (!fp_lidar_point_cloud.eof())
						{
							
							fp_lidar_point_cloud >> point.x;
							fp_lidar_point_cloud >> point.y;
							fp_lidar_point_cloud >> point.z;
							fp_lidar_point_cloud >> point.intensity;

							point_cloud.push_back(point);
							
						}
						else
						{
							fp_lidar_point_cloud.close();
							break;
						}

					} //『YOLOのバウンディングボックス内の『LiDAR』の点群を取得』終了(while文)



					for (int k = 0; k < (int)point_cloud.size(); k++)
					{
						// change the coordinates of xyz -> yzx (LiDAR)
						buf_LiDAR_yzx.x = (float)(-point_cloud[k].y);
						buf_LiDAR_yzx.y = (float)(-point_cloud[k].z);
						buf_LiDAR_yzx.z = (float)point_cloud[k].x;


						double lidar_distance = sqrt(pow(buf_LiDAR_yzx.x, 2.0) + pow(buf_LiDAR_yzx.y, 2.0) + pow(buf_LiDAR_yzx.z, 2.0));


						all_lidar_distance.push_back(lidar_distance);
						point_cloud_LiDAR_yzx.push_back(buf_LiDAR_yzx);


						//std::cout << buf_LiDAR_yzx.x << "\t" << buf_LiDAR_yzx.y << "\t" << buf_LiDAR_yzx.z << "\n";

					}



					//LiDAR　3D → 2D
					cv::projectPoints(point_cloud_LiDAR_yzx, rvec, tvec, K1, D1, imagePoints_LiDAR2ZED);





					
					for (int j = 0; j < (int)imagePoints_LiDAR2ZED.size(); j++)
					{

						if (point_cloud_LiDAR_yzx[j].z < 0) continue; //
						if (imagePoints_LiDAR2ZED[j].x < 0 || imagePoints_LiDAR2ZED[j].x >= IMG_XSIZE || imagePoints_LiDAR2ZED[j].y < 0 || imagePoints_LiDAR2ZED[j].y >= IMG_YSIZE) continue;


						//LiDARの点群がYOLOのバウンディングボックス内の時
						if ((imagePoints_LiDAR2ZED[j].x >= yolo_buf_bbox.x && imagePoints_LiDAR2ZED[j].x <= yolo_buf_bbox.x + yolo_buf_bbox.width) && (imagePoints_LiDAR2ZED[j].y >= yolo_buf_bbox.y && imagePoints_LiDAR2ZED[j].y <= yolo_buf_bbox.y + yolo_buf_bbox.height))
						{
							//バウンディングボックス内のLiDARの点群(2次元)
							yolo_lidar_inlier_point.x = imagePoints_LiDAR2ZED[j].x;
							yolo_lidar_inlier_point.y = imagePoints_LiDAR2ZED[j].y;

							yolo_lidar_inlier_pointcloud.push_back(yolo_lidar_inlier_point);



							//バウンディングボックス内のLiDARの点群(3次元)
							if (container)
							{
								point_cloud_in_a_boundingbox_container.x = point_cloud_LiDAR_yzx[j].x;
								point_cloud_in_a_boundingbox_container.y = point_cloud_LiDAR_yzx[j].y;
								point_cloud_in_a_boundingbox_container.z = point_cloud_LiDAR_yzx[j].z;

								all_point_cloud_in_a_boundingbox_container.push_back(point_cloud_in_a_boundingbox_container);
							}
							else if (person)
							{
								point_cloud_in_a_boundingbox_person.x = point_cloud_LiDAR_yzx[j].x;
								point_cloud_in_a_boundingbox_person.y = point_cloud_LiDAR_yzx[j].y;
								point_cloud_in_a_boundingbox_person.z = point_cloud_LiDAR_yzx[j].z;

								all_point_cloud_in_a_boundingbox_person.push_back(point_cloud_in_a_boundingbox_person);
							}
							else if (transporting)
							{
								point_cloud_in_a_boundingbox_transporting.x = point_cloud_LiDAR_yzx[j].x;
								point_cloud_in_a_boundingbox_transporting.y = point_cloud_LiDAR_yzx[j].y;
								point_cloud_in_a_boundingbox_transporting.z = point_cloud_LiDAR_yzx[j].z;

								all_point_cloud_in_a_boundingbox_transporting.push_back(point_cloud_in_a_boundingbox_transporting);
							}


							//std::cout << imagePoints_LiDAR2ZED[j].x << "\t" << imagePoints_LiDAR2ZED[j].y << "\t" << point_cloud_LiDAR_yzx[j].x << "\t" << point_cloud_LiDAR_yzx[j].y << "\t" << point_cloud_LiDAR_yzx[j].z << "\n";


						}

					}





					//2021_1_5
					//ヒストグラム化
					//make_histgram();






					//clear
					//yolo_BBox.clear();                         yolo_BBox.shrink_to_fit();
					
					

				}

				//clear(2021_1_3)←違うかも...
				//yolo_BBox.clear();                         yolo_BBox.shrink_to_fit();
				


			}
			else
			{
				std::cout << "Detected the end of file!!" << std::endl;
				break;
			}


		}//1枚ずつの画像に対しての処理finish(while文)







		if (n_yolo_bb_file_stream_size == 0)
		{
			n_flag_thread_gnuplot_exit = 1; //gnuplot 終了
			break;
		}


	} //全体の画像に対しての処理finish(while文)





	//all_point_cloud_in_a_boundingbox_transporting.clear();  all_point_cloud_in_a_boundingbox_transporting.shrink_to_fit();
	//all_point_cloud_in_a_boundingbox_person.clear();        all_point_cloud_in_a_boundingbox_person.shrink_to_fit();
	//all_point_cloud_in_a_boundingbox_container.clear();     all_point_cloud_in_a_boundingbox_container.shrink_to_fit();


	CloseHandle(hThread_gnuplot);



	std::cout << "\n" << std::endl;
	std::cout << "OK!!" << std::endl;

	return 0;
}










//gnuplot(mapping)
unsigned _stdcall thread_gnuplot(void *p)
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
		//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201117_low\\image%d.png'\n", i);
		//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201223112457\\image%d.png'\n", i);
		//fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201223113541\\image%d.png'\n", i);


		//copyしたフォルダ（ヒストグラムのテスト用）
		fprintf(gid, "set output 'D:\\Experiments\\YOLO\\result_!YOLO_MASUDA3_20000\\mapping_image_20201117_low_copy\\image%d.png'\n", i);


		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




		fprintf_s(gid, "plot '-' with points ps 1 lc rgb 'red' title 'Container in bounding box'\n");
		



		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//draw trajectory
		for (int a = 0; a < (int)all_point_cloud_in_a_boundingbox_container.size(); a++)
		{
			fprintf(gid, "%lf\t%lf\n", all_point_cloud_in_a_boundingbox_container[a].x, all_point_cloud_in_a_boundingbox_container[a].z);

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










std::vector<Point3f> make_histgram()
{

	int z_class = 0;



	for (int h = 0; h < hist_array; h++)
	{
		hist[h] = 0;
	}



	for (int h = 0; h < (int)all_lidar_distance.size(); h++)
	{
		double z_fractional_part = all_lidar_distance[h] - int(all_lidar_distance[h]);


		if (z_fractional_part >= 0.5)
		{
			z_class = int(all_lidar_distance[h] + 1.0); //例)8.8→9
		}
		else
		{
			z_class = int(all_lidar_distance[h]);  //例)8.4 →　8
		}



		hist[z_class]++;


	}



	max_distance(hist);  //返り値　i_max




	

	for (int h = 0; h < (int)all_lidar_distance.size(); h++)
	{
		double z_fractional_part = all_lidar_distance[h] - int(all_lidar_distance[h]);


		if (z_fractional_part >= 0.5)
		{
			z_class = int(all_lidar_distance[h] + 1.0); //例)8.8→9
		}
		else
		{
			z_class = int(all_lidar_distance[h]);  //例)8.4 →　8
		}


		if (z_class == i_max)
		{
			hist_result_container.x = all_point_cloud_in_a_boundingbox_container[h].x;
			hist_result_container.y = all_point_cloud_in_a_boundingbox_container[h].y;
			hist_result_container.z = all_point_cloud_in_a_boundingbox_container[h].z;

			hist_result_person.x = all_point_cloud_in_a_boundingbox_person[h].x;
			hist_result_person.y = all_point_cloud_in_a_boundingbox_person[h].y;
			hist_result_person.z = all_point_cloud_in_a_boundingbox_person[h].z;

			hist_result_transporting.x = all_point_cloud_in_a_boundingbox_transporting[h].x;
			hist_result_transporting.y = all_point_cloud_in_a_boundingbox_transporting[h].y;
			hist_result_transporting.z = all_point_cloud_in_a_boundingbox_transporting[h].z;

			all_hist_result_container.push_back(hist_result_container);
			all_hist_result_person.push_back(hist_result_person);
			all_hist_result_transporting.push_back(hist_result_transporting);
		}
		else
		{
			continue;
		}


	}



	return all_hist_result_container, all_hist_result_person, all_hist_result_transporting;
}








int max_distance(int hist[])
{
	int max;

	//hist[0]を最大値としておく
	max = hist[0];

	for (int i = 0; i < hist_array; i++)
	{
		if (max < hist[i])
		{
			max = hist[i];
			i_max = i;
		}
	}


	return i_max;

}

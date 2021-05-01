#include <iostream>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

struct tracking_object_type {

	double x;
	double y;
	double z;
	int type;

};

void show_objects_pose_via_gnuplot(FILE* gid, std::vector<std::vector<struct tracking_object_type>> Objects) {

	fprintf(gid, "set size ratio -1\n");
	fprintf(gid, "set xl 'X[m]'\n");
	fprintf(gid, "set yl 'Z[m]'\n");
	fprintf(gid, "set xr [-30:10]\n");
	fprintf(gid, "set yr [0:40]\n");

	fprintf(gid, "set key right outside\n");
	fflush(gid);

	fprintf(gid, "plot for [i=0:*] '-' index i with lines\n");
	for (size_t i = 0; i < Objects.size(); i++) {
		for (size_t j = 0; j < Objects[i].size(); j++) {

			fprintf(gid, "%lf\t%lf\n", Objects[i][j].x, Objects[i][j].z);

		}

		fprintf(gid, "\n\n");
	}

	fprintf(gid, "e\n");
	fflush(gid);
	fprintf(gid, "pause 0.5\n");
	fflush(gid);

};

int main()
{

	FILE* gid;

	vector<struct tracking_object_type> IN_Objects;
	vector<struct tracking_object_type> IN_Objects_buf;



	struct tracking_object_type in_object;
	vector<struct tracking_object_type> targeted_object;

	vector<vector<struct tracking_object_type>> Objects;



	// Prepare sample point cloud //
	std::string filename_all_median_pointcloud;



	////////////////////////////////////////////////////////////////////////////////
	
	//filename_all_median_pointcloud = "D:/MyTemp/all_median_points_mine_person_1m.txt";

	//filename_all_median_pointcloud = "D:/MyTemp/all_median_points_mine_person_57_1m.txt";

	//filename_all_median_pointcloud = "D:/MyTemp/all_median_points_mine_person_2m.txt";

	filename_all_median_pointcloud = "D:/MyTemp/all_median_points_mine_person_57_2m.txt";

	




	//filename_all_median_pointcloud = "D:/MyTemp/all_median_points_mine_container_1m.txt";

	//filename_all_median_pointcloud = "D:/MyTemp/all_median_points_mine_container_57_1m.txt";

	//filename_all_median_pointcloud = "D:/MyTemp/all_median_points_mine_container_2m.txt";

	//filename_all_median_pointcloud = "D:/MyTemp/all_median_points_mine_container_57_2m.txt";

	////////////////////////////////////////////////////////////////////////////////



	std::ifstream ifs;
	ifs.open(filename_all_median_pointcloud);
	if (!ifs.is_open()) 
	{
		std::cerr << "cannot open this file!" << std::endl;
		exit(1);
	}
	while (!ifs.eof()) 
	{

		ifs >> in_object.x >> in_object.y >> in_object.z;
		//std::cout << in_object.x << "\t" << in_object.y << "\t" << in_object.z << std::endl;
		IN_Objects_buf.push_back(in_object);

	}
	ifs.close();


	// tracking //


	double d_dx;
	double d_dy;
	double d_dz;

	double d_object_distance;
	double d_object_distance_min;
	int n_object_distance_min_id;
	int n_object_tracked_count;

	double d_object_distance_thresh = 1.0;

	tracking_object_type input_vec;
	gid = _popen("C:/Win64App/gnuplot/bin/gnuplot.exe", "w");
	for (size_t num = 0; num < IN_Objects_buf.size(); num++) 
	{

		IN_Objects.clear();
		IN_Objects.shrink_to_fit();
		input_vec = IN_Objects_buf[num];
		IN_Objects.push_back(input_vec);
		std::cout << "current frame: " << to_string((int)num + 1) << " / " << IN_Objects_buf.size() << std::endl;

		for (size_t f = 0; f < IN_Objects.size(); f++) 
		{ //each frame


			in_object.x = IN_Objects[f].x;
			in_object.y = IN_Objects[f].y;
			in_object.z = IN_Objects[f].z;
			in_object.type = IN_Objects[f].type;


			// using targeted object taken from 1st frame for tracking
			if (Objects.size() == 0) 
			{

				targeted_object.clear();
				targeted_object.shrink_to_fit();

				targeted_object.push_back(in_object);

				Objects.push_back(targeted_object); //add the 1st object		

			}
			else 
			{

				// search the nearest object //

				d_object_distance_min = 100000;

				for (size_t k = 0; k < Objects.size(); k++) 
				{
	
					n_object_tracked_count = (int)Objects[k].size();
					d_dx = (in_object.x - Objects[k][n_object_tracked_count - 1].x);
					d_dy = (in_object.y - Objects[k][n_object_tracked_count - 1].y);
					d_dz = (in_object.z - Objects[k][n_object_tracked_count - 1].z);
						

					/*d_dx = (in_object.x - Objects[k][l].x);
					d_dy = (in_object.y - Objects[k][l].y);
					d_dz = (in_object.z - Objects[k][l].z);*/

					d_object_distance = sqrt(d_dx*d_dx + d_dy*d_dy + d_dz*d_dz);


					if (d_object_distance_min > d_object_distance) {

						d_object_distance_min = d_object_distance;
						n_object_distance_min_id = k;

					}
				}

				// check the distance between the input object and the nearest object //

				if (d_object_distance_min < d_object_distance_thresh) {

					Objects[n_object_distance_min_id].push_back(in_object);

				}
				else 
				{

					targeted_object.clear();
					targeted_object.shrink_to_fit();

					targeted_object.push_back(in_object);

					Objects.push_back(targeted_object);

				}

			}

			show_objects_pose_via_gnuplot(gid, Objects);

		}
	}


	_pclose(gid);

	// check the tracked object data //
	std::string filename_save_tracked_objects;

	////////////////////////////////////////////////////////////////////////////////

	filename_save_tracked_objects = "D:/MyTemp/oomurasann_code_result/text_for_masuda_after_processing.txt";

	////////////////////////////////////////////////////////////////////////////////


	std::ofstream ofs;
	ofs.open(filename_save_tracked_objects);


	for (size_t j = 0; j < Objects.size(); j++) {

		ofs << "Object: " << j << endl;

		for (size_t i = 0; i < Objects[j].size(); i++) {

			//cout << to_string(Objects[j][i].x) << "," << to_string(Objects[j][i].y) << "," << to_string(Objects[j][i].z) << endl;
			ofs << j << "\t" << Objects[j][i].x << "\t" << Objects[j][i].y << "\t" << Objects[j][i].z << std::endl;

		}

		cout << endl;
		ofs << std::endl << std::endl;

	}

	ofs.close();


	return 0;
}


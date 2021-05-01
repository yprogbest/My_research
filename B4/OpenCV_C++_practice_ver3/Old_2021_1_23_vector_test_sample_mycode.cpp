//2021_1_20 addition
//Moving Trajectory
int vector_test_sample()
{


	std::cout << "total number of IN_Objects array: " << IN_Objects.size() << std::endl;





	//2021_1_21 addtion

	for (size_t f = 0; f < IN_Objects.size(); f++)
	{ //each frame


		in_object.x = IN_Objects[f].x;
		in_object.y = IN_Objects[f].y;
		in_object.z = IN_Objects[f].z;
		in_object.type = IN_Objects[f].type;





		if (Labels.size() == 0)
		{

			//targeted_object.clear();
			//targeted_object.shrink_to_fit();

			targeted_object = IN_Objects;

			Objects.push_back(targeted_object); //add the 1st object


			for (int a = 0; a < (int)IN_Objects.size(); a++)
			{
				label.push_back(IN_Objects[a]);

				Labels.push_back(label);


				label.clear();      label.shrink_to_fit();
			}

		}
		else
		{


			// search the nearest object //

			for (size_t k = 0; k < Objects.size(); k++)
			{

				n_object_tracked_count = (int)Objects[k].size();



				d_object_distance_array.clear();            d_object_distance_array.shrink_to_fit();


				for (int a = 0; a < n_object_tracked_count; a++)
				{
					d_dx = (in_object.x - Objects[Objects.size() - 1][a].x);
					d_dy = (in_object.y - Objects[Objects.size() - 1][a].y);
					d_dz = (in_object.z - Objects[Objects.size() - 1][a].z);


					d_object_distance = sqrt(d_dx * d_dx + d_dy * d_dy + d_dz * d_dz);

					d_object_distance_array.push_back(d_object_distance);


				}


				double d_object_distance_min = d_object_distance_array[0];


				for (int a = 0; a < (int)d_object_distance_array.size(); a++)
				{
					if (d_object_distance_array[a] < d_object_distance_min)
					{
						d_object_distance_min = d_object_distance_array[a];
						n_object_distance_min_id = a;
					}
				}

			}


			// check the distance between the input object and the nearest object //

			if (d_object_distance_min < d_object_distance_thresh)
			{

				Labels[n_object_distance_min_id].push_back(in_object);

			}
			else
			{

				Labels[n_object_tracked_count + 1].push_back(in_object);

			}


			targeted_object.clear();    targeted_object.shrink_to_fit();


			targeted_object.push_back(in_object);


		}

	}



	Objects.push_back(targeted_object);



	// check the tracked object data //

	for (size_t j = 0; j < Labels.size(); j++)
	{

		cout << "Object: " << j << endl;

		for (size_t i = 0; i < Labels[j].size(); i++)
		{

			cout << j << "\t" << i << "\t" << to_string(Labels[j][i].x) << "," << to_string(Labels[j][i].y) << "," << to_string(Labels[j][i].z) << endl;

		}

		cout << endl;

	}

	return 0;

}
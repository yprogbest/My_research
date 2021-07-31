#include "Particle.hpp"




//初期化
Particle::Particle()
{
	Particle(0, 0, 0, 0);
}


//初期化
Particle::Particle(int height_point, int width_point,
	int height_speed_point, int width_speed_point)
{
	height = height_point;
	width = width_point;
	height_speed = height_speed_point;
	width_speed = width_speed_point;
}


//パラメータの表示
void Particle::PrintParameter()
{
	std::cout << "width = " << width <<
		", height = " << height <<
		", v = " << width_speed <<
		", u = " << height_speed << "\n";
}




//パーティクルフィルターの初期化
ParticleFilter::ParticleFilter()
{
	//ランダム
	int rand_point = rand() * 100 + 1;
	std::vector<int>upper = { rand_point, rand_point, 10,10 };
	std::vector<int>lower = { 0, 0, -10,-10 };
	std::vector<int>noise = { 30, 30, 10,10 };
	ParticleFilter(100, rand_point, rand_point, upper, lower, noise);
}




//パーティクルフィルターの初期化
ParticleFilter::ParticleFilter(int particle_num, int height_point, int width_point,
	std::vector<int> &upper_point, std::vector<int> &lower_point,
	std::vector<int> &noise_point)
{
	num = particle_num; ////粒子数
	height = height_point;
	width = width_point;

	upper = upper_point;
	lower = lower_point;
	noise = noise_point;


	//パーティクルの初期化を行う
	for (int i = 0; i < num; ++i)
	{
		//ランダムで初期化する
		int n[4];

		for (int i = 0; i < 4; ++i)
		{
			int x = (int)(((double)rand() / RAND_MAX)*INT_MAX);
			n[i] = x % (upper[i] - lower[i]) + lower[i];
		}

		Particle particle(n[0], n[1], n[2], n[3]);

		//粒子の数だけ重みを初期化する
		particle_vector.push_back(particle);

	}
}
#pragma once
#include <iostream>
#include <vector>
#include <random>

#include "OPENCV.hpp"

class Particle;       //パーティクルに関するクラス
class ParticleFilter; //パーティクルフィルタ関係のクラス

class Particle {
public:
	//重み
	double weight; // = 1.0;

	//パラメータ
	int height;
	int width;
	int height_speed;
	int width_speed;

	//コンストラクタ
	Particle();
	Particle(int height_point, int width_point,
		int height_speed_point, int width_speed_point);

	void PrintParameter();
};

class ParticleFilter
{
private:
	int num; //粒子数
	int width; //横
	int height; //縦
	std::vector<int> upper; //最大値
	std::vector<int> lower; //最小値
	std::vector<int> noise; //ノイズ
	std::vector<Particle> particle_vector; //パーティクルの管理 

public:
	ParticleFilter();
	ParticleFilter(int particle_num, int height_point, int width_point,
		std::vector<int> &upper_point, std::vector<int> &lower_point,
		std::vector<int> &noise_point);

	Particle Measure(); //重心の計算
	void Predict(); //予測
	void CalcWeight(cv::Mat &input_image); //重みの計算
	void Resampling(); //リサンプリング
	double Likelifood(int x, int y, cv::Mat &input_image); //尤度計算

	std::vector<Particle> GetPaticleVector(); //粒子を返却する
};
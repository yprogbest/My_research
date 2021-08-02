#pragma once
#include <iostream>
#include <vector>
#include <random>

#include "OPENCV.hpp"

class Particle;       //�p�[�e�B�N���Ɋւ���N���X
class ParticleFilter; //�p�[�e�B�N���t�B���^�֌W�̃N���X

class Particle {
public:
	//�d��
	double weight; // = 1.0;

	//�p�����[�^
	int height;
	int width;
	int height_speed;
	int width_speed;

	//�R���X�g���N�^
	Particle();
	Particle(int height_point, int width_point,
		int height_speed_point, int width_speed_point);

	void PrintParameter();
};

class ParticleFilter
{
private:
	int num; //���q��
	int width; //��
	int height; //�c
	std::vector<int> upper; //�ő�l
	std::vector<int> lower; //�ŏ��l
	std::vector<int> noise; //�m�C�Y
	std::vector<Particle> particle_vector; //�p�[�e�B�N���̊Ǘ� 

public:
	ParticleFilter();
	ParticleFilter(int particle_num, int height_point, int width_point,
		std::vector<int> &upper_point, std::vector<int> &lower_point,
		std::vector<int> &noise_point);

	Particle Measure(); //�d�S�̌v�Z
	void Predict(); //�\��
	void CalcWeight(cv::Mat &input_image); //�d�݂̌v�Z
	void Resampling(); //���T���v�����O
	double Likelifood(int x, int y, cv::Mat &input_image); //�ޓx�v�Z

	std::vector<Particle> GetPaticleVector(); //���q��ԋp����
};
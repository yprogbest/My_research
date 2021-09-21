#include "Particle.hpp"




//������
Particle::Particle()
{
	Particle(0, 0, 0, 0);
}

//������
Particle::Particle(int height_point, int width_point, int height_speed_point, int width_speed_point)
{
	height = height_point;
	width = width_point;
	height_speed = height_speed_point;
	width_speed = width_speed_point;
}


//�p�����[�^�̕\��
void Particle::PrintParameter()
{
	std::cout << "width = " << width <<
		", height = " << height <<
		", v = " << width_speed <<
		", u = " << height_speed << "\n";
}


//�p�[�e�B�N���t�B���^�̏�����
ParticleFilter::ParticleFilter()
{
	//�����_��(�摜�T�C�Y�ɕύX����ׂ��H)
	int rand_point = rand() * 100 + 1;
	std::vector<int> upper = { rand_point, rand_point, 10, 10 };
	std::vector<int> lower = { 0, 0, -10, -10 };
	std::vector<int> noise = { 30, 30, 10, 10 };
	ParticleFilter(300, rand_point, rand_point, upper, lower, noise);
}


//�p�[�e�B�N���t�B���^�̏�����
ParticleFilter::ParticleFilter(int particle_num, int height_point, int width_point,
	std::vector<int> &upper_point, std::vector<int> &lower_point,
	std::vector<int> &noise_point)
{
	num = particle_num;
	height = height_point;
	width = width_point;

	upper = upper_point;
	lower = lower_point;
	noise = noise_point;

	//�p�[�e�B�N���̏��������s��
	for (int i = 0; i < num; ++i)
	{
		//�����_���ŏ���������
		int n[4];
		for (int i = 0; i < 4; ++i)
		{
			int x = (int)(((double)rand() / RAND_MAX)*INT_MAX); //�����_���Ȓl
			n[i] = x % (upper[i] - lower[i]) + lower[i];
		}
		Particle particle(n[0], n[1], n[2], n[3]);

		//���q�̐������d�݂�����������
		particle_vector.push_back(particle);
		particle_vector[i].weight = 1.0 / num;
	}

}


//���T���v�����O
void ParticleFilter::Resampling()
{
	// accumulate weight
	std::vector<double> sum_weight(num);
	sum_weight[0] = particle_vector[0].weight;
	for (int i = 1; i < num; ++i)
	{
		sum_weight[i] = sum_weight[i - 1] + particle_vector[i].weight;
	}

	std::vector<Particle> pre_particle(particle_vector);

	for (int i = 0; i < num; ++i)
	{
		double weight_threshold = (double)(rand() % 10000) / 10000.0;

		for (int j = 0; j < num; ++j)
		{
			if (weight_threshold > sum_weight[j])
			{
				continue;
			}
			else
			{
				particle_vector[i] = pre_particle[j];
				particle_vector[i].weight = 0.0;
				break;
			}
		}
	}
}


//���̂̈ʒu��\������
void ParticleFilter::Predict()
{
	for (int i = 0; i < num; ++i)
	{
		int n[4];
		for (int i = 0; i < 4; ++i)

			//
			n[i] = (int)(((double)rand() / RAND_MAX) * INT_MAX) % (noise[i] * 2) - noise[i];

		particle_vector[i].width += particle_vector[i].width_speed + n[0];
		particle_vector[i].height += particle_vector[i].height_speed + n[1];
		particle_vector[i].width_speed += n[2];
		particle_vector[i].height_speed += n[3];

		//�ׂ��������͂�����
		//update state
		if (particle_vector[i].width < lower[0]) particle_vector[i].width = lower[0];
		if (particle_vector[i].height < lower[1]) particle_vector[i].height = lower[1];
		if (particle_vector[i].width_speed < lower[2]) particle_vector[i].width_speed = lower[2];
		if (particle_vector[i].height_speed < lower[3]) particle_vector[i].height_speed = lower[3];

		if (particle_vector[i].width >= upper[0]) particle_vector[i].width = upper[0];
		if (particle_vector[i].height >= upper[1]) particle_vector[i].height = upper[1];
		if (particle_vector[i].width_speed >= upper[2]) particle_vector[i].width_speed = upper[2];
		if (particle_vector[i].height_speed >= upper[3]) particle_vector[i].height_speed = upper[3];
	}
}


//�d�݂̌v�Z
void ParticleFilter::CalcWeight(cv::Mat &input_image)
{
	double sum_weight = 0.0;

	for (int i = 0; i < particle_vector.size(); ++i)
	{
		int x = particle_vector[i].width;
		int y = particle_vector[i].height;

		//�ޓx�̌v�Z
		if (x < 0 || x > input_image.size().width || y < 0 || y > input_image.size().height)
			particle_vector[i].weight = 0.001;
		else
			particle_vector[i].weight = Likelifood(x, y, input_image);

		sum_weight += particle_vector[i].weight;
	}

	//���K��
	for (int i = 0; i < particle_vector.size(); ++i)
	{
		particle_vector[i].weight /= sum_weight;
	}
}


//�ޓx�̌v�Z
//�ޓx�̌v�Z��ύX����ƐF�X�Ǝg����
//����͐ԐF��ǐՂ���
double ParticleFilter::Likelifood(int x, int y, cv::Mat &input_image)
{
	float b, g, r;
	float dist = 0.0, sigma = 50.0;
	float weight;

	b = input_image.data[y * input_image.step + x * input_image.elemSize() + 0];    //B
	g = input_image.data[y * input_image.step + x * input_image.elemSize() + 1];    //G
	r = input_image.data[y * input_image.step + x * input_image.elemSize() + 2];    //R

	dist = sqrt(b * b + g * g + (255.0 - r) * (255.0 - r)); //dist���ŏ��ɂȂ�΁Cweight�͍ő�ɂȂ�

	weight = 1.0 / (sqrt(2.0 * CV_PI) * sigma*sigma) * expf(-dist * dist / (2.0 * sigma * sigma));

	return weight;
}


//Particle�̏d�S���v�Z
Particle ParticleFilter::Measure()
{
	Particle p = Particle(0, 0, 0, 0);

	//������
	double n[4];
	for (int i = 0; i < 4; ++i)
		n[i] = 0.0;

	for (int i = 0; i < particle_vector.size(); ++i)
	{
		n[0] += particle_vector[i].width * particle_vector[i].weight;
		n[1] += particle_vector[i].height * particle_vector[i].weight;
		n[2] += particle_vector[i].width_speed * particle_vector[i].weight;
		n[3] += particle_vector[i].height_speed * particle_vector[i].weight;
	}
	p.width = (int)n[0];
	p.height = (int)n[1];
	p.width_speed = (int)n[2];
	p.height_speed = (int)n[3];

	return p;
}



//���q�x�N�g���̕ԋp
std::vector<Particle> ParticleFilter::GetPaticleVector()
{
	return particle_vector;
}



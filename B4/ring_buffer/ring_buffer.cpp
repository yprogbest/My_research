#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //�e�L�X�g�t�@�C�����������߂ɗp��
#include <string>
#include <process.h>

#include <opencv2/opencv.hpp>//OpenCV�̃C���N���[�h
#include "opencv2/highgui/highgui.hpp"



#define SUCCESS 1 //����
#define FAILURE 0 //���s

#define QUEUE_SIZE 10 //�҂��s��ɓ���f�[�^�̍ő�l

typedef int data_t;  //�f�[�^�^ (https://programming.pc-note.net/c/typedef.html)

data_t queue_data[QUEUE_SIZE]; //�҂��s��f�[�^�{��
int queue_head; //�f�[�^�擪
int queue_num; //�f�[�^��



//prototype declaration
int enqueue(data_t enq_data);
int dequeue(data_t *deq_data);
void queuePrint();





//http://www.cc.kyoto-su.ac.jp/~yamada/ap/queue.html
int main()
{
	int i;
	data_t d;

	queue_head = queue_num = 0;     /* queue ���g�p����O�ɁC�K�������̕ϐ��������� */





	for (i = 0; i < 100; i++)
	{
		if (enqueue(i) == SUCCESS)
		{
			queuePrint();
		}
		else
		{
			dequeue(&d);

			if (enqueue(i) == SUCCESS)
			{
				queuePrint();
			}
			else
			{
				dequeue(&d);
			}

		}


	}


}



//�҂��s��փf�[�^��ǉ�����֐�
int enqueue(data_t enq_data)
{

	if (queue_num < QUEUE_SIZE) 
	{
		queue_data[(queue_head + queue_num) % QUEUE_SIZE] = enq_data;
		queue_num++;
		return SUCCESS;
	}
	else 
	{
		return FAILURE;
	}
}





//�҂��s��̐擪�̃f�[�^�����o���֐�
int dequeue(data_t *deq_data)
{
	if (queue_num > 0)
	{
		*deq_data = queue_data[queue_head];
		queue_head = (queue_head + 1) % QUEUE_SIZE;
		queue_num--;

		return SUCCESS;
	}
	else
	{
		return FAILURE;
	}
}



void queuePrint()
{
	int i;

	printf("queue [");

	for (i = 0; i < QUEUE_SIZE; i++)
	{
		if ((queue_head + queue_num <= QUEUE_SIZE && queue_head <= i && i < queue_head + queue_num)
				|| (queue_head + queue_num > QUEUE_SIZE && (queue_head <= i || i < (queue_head + queue_num) % QUEUE_SIZE))) {

			printf("%3d", queue_data[i]);

		}
		else
		{
			printf("%3c", '.');       /* queue �ɓ����Ă��Ȃ��f�[�^�͕\�����Ȃ� */
		}


	}


	printf("]\n");
}
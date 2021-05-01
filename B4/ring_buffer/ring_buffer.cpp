#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <fstream> //テキストファイルを扱うために用意
#include <string>
#include <process.h>

#include <opencv2/opencv.hpp>//OpenCVのインクルード
#include "opencv2/highgui/highgui.hpp"



#define SUCCESS 1 //成功
#define FAILURE 0 //失敗

#define QUEUE_SIZE 10 //待ち行列に入るデータの最大値

typedef int data_t;  //データ型 (https://programming.pc-note.net/c/typedef.html)

data_t queue_data[QUEUE_SIZE]; //待ち行列データ本体
int queue_head; //データ先頭
int queue_num; //データ個数



//prototype declaration
int enqueue(data_t enq_data);
int dequeue(data_t *deq_data);
void queuePrint();





//http://www.cc.kyoto-su.ac.jp/~yamada/ap/queue.html
int main()
{
	int i;
	data_t d;

	queue_head = queue_num = 0;     /* queue を使用する前に，必ずこれらの変数を初期化 */





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



//待ち行列へデータを追加する関数
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





//待ち行列の先頭のデータを取り出す関数
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
			printf("%3c", '.');       /* queue に入っていないデータは表示しない */
		}


	}


	printf("]\n");
}
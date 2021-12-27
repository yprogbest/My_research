
#include <windows.h>
#include <process.h>
#include <tchar.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "gps.h"




void class_GPS::gps_latitude_longitude_to_degree(double latitude, double longitude, double* latitude_degree, double* longitude_degree)
{

	double d_degree_dd;
	double d_minute_mm;
	double d_degree_ddd;


	d_degree_dd = (int)(latitude / 100.0);
	d_minute_mm = latitude - d_degree_dd*100.0;

	*latitude_degree = d_degree_dd + d_minute_mm / 60.0;



	d_degree_ddd = (int)(longitude / 100.0);
	d_minute_mm = longitude - d_degree_ddd * 100.0;

	*longitude_degree = d_degree_ddd + d_minute_mm / 60.0;


}

void class_GPS::gps_LLH_to_ECEF_coordinates(double latitude, double longitude, double height, double *X, double *Y, double *Z)
{

	double ECEF_e2;
	double ECEF_f = 1.0 / 298.257223563;
	double ECEF_a = 6378137;
	double ECEF_N;

	double d_latitude_radian;
	double d_longitude_radian;


	d_latitude_radian = latitude / 180.0 * GPS_PI;
	d_longitude_radian = longitude / 180.0 * GPS_PI;



	ECEF_e2 = 2 * ECEF_f - ECEF_f * ECEF_f;

	ECEF_N = ECEF_a / sqrt(1 - ECEF_e2 * sin(d_latitude_radian)*sin(d_latitude_radian));

	*X = (ECEF_N + height)*cos(d_latitude_radian)*cos(d_longitude_radian);
	*Y = (ECEF_N + height)*cos(d_latitude_radian)*sin(d_longitude_radian);
	*Z = (ECEF_N * (1.0 - ECEF_e2) + height) * sin(d_latitude_radian);


}


struct strct_GPS_matrix class_GPS::GPS_rotate_x(double d_theta)
{
	double d_theta_radian;
	struct strct_GPS_matrix rotate_x;

	d_theta_radian = d_theta / 180.0 * GPS_PI;

	rotate_x.row_num = 3;
	rotate_x.col_num = 3;

	rotate_x.R[0][0] = 1; rotate_x.R[0][1] = 0;                      rotate_x.R[0][2] = 0;
	rotate_x.R[1][0] = 0; rotate_x.R[1][1] = cos(d_theta_radian); rotate_x.R[1][2] = sin(d_theta_radian);
	rotate_x.R[2][0] = 0; rotate_x.R[2][1] = -sin(d_theta_radian); rotate_x.R[2][2] = cos(d_theta_radian);


	return rotate_x;



}

struct strct_GPS_matrix class_GPS::GPS_rotate_y(double d_theta)
{
	double d_theta_radian;
	struct strct_GPS_matrix rotate_y;

	d_theta_radian = d_theta / 180.0 * GPS_PI;


	rotate_y.row_num = 3;
	rotate_y.col_num = 3;

	rotate_y.R[0][0] = cos(d_theta_radian); rotate_y.R[0][1] = 0; rotate_y.R[0][2] = -sin(d_theta_radian);
	rotate_y.R[1][0] = 0;                     rotate_y.R[1][1] = 1; rotate_y.R[1][2] = 0;
	rotate_y.R[2][0] = sin(d_theta_radian); rotate_y.R[2][1] = 0; rotate_y.R[2][2] = cos(d_theta_radian);


	return rotate_y;



}

struct strct_GPS_matrix class_GPS::GPS_rotate_z(double d_theta)
{
	double d_theta_radian;
	struct strct_GPS_matrix rotate_z;

	d_theta_radian = d_theta / 180.0 * GPS_PI;

	rotate_z.row_num = 3;
	rotate_z.col_num = 3;

	rotate_z.R[0][0] = cos(d_theta_radian);  rotate_z.R[0][1] = sin(d_theta_radian); rotate_z.R[0][2] = 0;
	rotate_z.R[1][0] = -sin(d_theta_radian); rotate_z.R[1][1] = cos(d_theta_radian); rotate_z.R[1][2] = 0;
	rotate_z.R[2][0] = 0;                      rotate_z.R[2][1] = 0;                     rotate_z.R[2][2] = 1;


	return rotate_z;



}


struct strct_GPS_matrix class_GPS::GPS_matrix_multi(struct strct_GPS_matrix *mat_1, struct strct_GPS_matrix *mat_2)
{

	int i, j;
	int k;

	int row1_num, col1_num;
	int col2_num;

	double d_sum;

	struct strct_GPS_matrix mat_out;



	row1_num = mat_1->row_num;
	col1_num = mat_1->col_num;
	col2_num = mat_2->col_num;



	mat_out.row_num = row1_num;
	mat_out.col_num = col2_num;



	for (j = 0; j < row1_num; j++){
		for (i = 0; i < col2_num; i++){

			d_sum = 0;

			for (k = 0; k < col1_num; k++){

				d_sum += mat_1->R[j][k] * mat_2->R[k][i];

			}


			mat_out.R[j][i] = d_sum;

		}
	}


	return mat_out;

}



void class_GPS::gps_ECEF_to_ENU_coordinates(double base_latitude, double base_longitude, struct strct_ECEF_coordinates x0y0z0, struct strct_ECEF_coordinates xyz, double *e, double *n, double *u)
{
	double x, y, z;

	struct strct_GPS_matrix matrix_Rotate_y, matrix_Rotate_z, matrix_Rotate_z2;
	struct strct_GPS_matrix matrix_Rotate_yz;
	struct strct_GPS_matrix matrix_Rotate_z2y;
	struct strct_GPS_matrix matrix_Rotate_zyz;


	x = xyz.X - x0y0z0.X;
	y = xyz.Y - x0y0z0.Y;
	z = xyz.Z - x0y0z0.Z;


	matrix_Rotate_y = GPS_rotate_y(90 - base_latitude);
	matrix_Rotate_z = GPS_rotate_z(base_longitude);
	matrix_Rotate_z2 = GPS_rotate_z(90);


	matrix_Rotate_z2y = GPS_matrix_multi(&matrix_Rotate_z2, &matrix_Rotate_y);
	matrix_Rotate_yz = GPS_matrix_multi(&matrix_Rotate_y, &matrix_Rotate_z);





	matrix_Rotate_zyz = GPS_matrix_multi(&matrix_Rotate_z2y, &matrix_Rotate_z);





	*e = matrix_Rotate_zyz.R[0][0] * x + matrix_Rotate_zyz.R[0][1] * y + matrix_Rotate_zyz.R[0][2] * z;
	*n = matrix_Rotate_zyz.R[1][0] * x + matrix_Rotate_zyz.R[1][1] * y + matrix_Rotate_zyz.R[1][2] * z;
	*u = matrix_Rotate_zyz.R[2][0] * x + matrix_Rotate_zyz.R[2][1] * y + matrix_Rotate_zyz.R[2][2] * z;


}



int class_GPS::gps_open_COMM(int nComPortNo)
{

	TCHAR sComPortNo[30];

	_stprintf_s(sComPortNo, 30, _T("\\\\.\\COM%d"), nComPortNo);


	//Open Comm port

	//非同期読み取りをする場合、FILE_FLAG_OVERLAPPED 指定する。
	hComm = CreateFile(
		sComPortNo,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);



	if (hComm == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "%d : failed to open a comm port\n", GetLastError());
		return -1;
	}

	// Commポート設定


	// DCB を取得
	GetCommState(hComm, &dcb);
	dcb.BaudRate = 9600;//BAUD_115200 ;		// ボーレート(Bluetoothでは無視される)
	dcb.ByteSize = 8;					// 1バイトのサイズ
	dcb.fParity = FALSE;				// パリティを使用するか
	dcb.Parity = NOPARITY;				// パリティ設定
	dcb.StopBits = ONESTOPBIT;			// ストップビット
	dcb.fOutxCtsFlow = FALSE;			// 送信時に、CTS を監視するかどうかを
	dcb.fOutxDsrFlow = FALSE;			// 送信時に、DSR を監視するかどうかを
	dcb.fDsrSensitivity = FALSE;		// DSR がOFFの間は受信データを無視するか
	SetCommState(hComm, &dcb);		// DCB を設定

	printf("OPEN RS-232C => OK\n");



	return 0;


}

int class_GPS::gps_receive_data(HANDLE hComm, _TCHAR szRecvBuffer[])
{

	char rBuf[128];
	DWORD dw_Recv_size;

	int i;



	i = 0;


	while (1){

		ReadFile(hComm, rBuf, 1, &dw_Recv_size, 0);

		if (dw_Recv_size == 1){

			if (rBuf[0] == '\n'){ // 13:\r 10:\n

				if (i != 0){

					szRecvBuffer[i] = '\n';
					szRecvBuffer[i + 1] = '\0';


					//printf("Received:%s\n", szRecvBuffer);



					break;
				}

			}
			else{

				szRecvBuffer[i] = rBuf[0];
				i++;

			}

		}

	}


	return 1;




}




int class_GPS::gps_send_command(HANDLE hComm, _TCHAR szCommandBuffer[])
{
	
	
	hEvent = CreateEvent(NULL, FALSE, FALSE, _T("olp"));

	memset(&ovl, 0, sizeof(OVERLAPPED));
	ovl.hEvent = hEvent;
	
	bRet = WriteFile(hComm, szCommandBuffer, (DWORD)_tclen(szCommandBuffer), &dwWritten, &ovl);

	if (!bRet){

		bLError = GetLastError();

		if (bLError == ERROR_IO_PENDING){
			GetOverlappedResult(hComm, &ovl, &dwWritten, TRUE);

		}
		else{
			_tprintf_s(_T("error occuered!!"));
			
			CloseHandle(hEvent);
			return -2;

		}
	}


	CloseHandle(hEvent);


	return 0;

}




int class_GPS::gps_set_base_GPS(void)
{

	_tcscpy_s(sz_send_Command, _T("$JRTK,1,P\n\r"));

	gps_send_command(hComm, sz_send_Command);

	return 0;


}
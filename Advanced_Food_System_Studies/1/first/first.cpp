#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

int main(int, char**)
{
	VideoCapture cap(0);

	if (!cap.isOpened()) return -1;

	Mat frame;

	namedWindow("image", WINDOW_AUTOSIZE);

	while (1) {

		cap >> frame;

		imshow("image", frame);

		if (waitKey(30) >= 0) break;

	}

	return 0;
}

/*
Line detector 
*/
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>*/

//#define PORT "/dev/"

//int tty_fd;
//struct termios old_stdio;


using namespace std;
using namespace cv;

int main()
{

	VideoCapture cap(1);

	if (!cap.isOpened())
	{
		cout << "error";
		return -1;
	}

	Mat frame,imgHSV,whiteLane,roi,obstacle,gauss,dst;
	vector< vector<Point> > contours, contours_ylw; // list of contour points
	vector<Vec4i> hierarchy, hirarki_ylw;
	vector<vector<Point>> box;
	int i;
	double thresh = 100;
	double maxValue = 200;



	/*
	struct termios tio, stdio;

	 unsigned char c='D';
	tcgetattr(STDOUT_FILENO,&old_stdio);

	 printf("Please start with %s /dev/ttyACM0 (for example)\n",argv[0]);
	 memset(&stdio,0,sizeof(stdio));
	 stdio.c_iflag=0;
	 stdio.c_oflag=0;
	 stdio.c_cflag=0;
	 stdio.c_lflag=0;
	 stdio.c_cc[VMIN]=1;
	 stdio.c_cc[VTIME]=0;
	 tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
	 tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
	 fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // make the reads non-blocking

	 memset(&tio,0,sizeof(tio));
	 tio.c_iflag=0;
	 tio.c_oflag=0;
	 tio.c_cflag=CS8|CREAD|CLOCAL; // 8n1, see termios.h for more information
	 tio.c_lflag=0;
	 tio.c_cc[VMIN]=1;
	 tio.c_cc[VTIME]=5;
	 tty_fd=open(argv[1], O_RDWR | O_NONBLOCK);
	 cfsetospeed(&tio,B9600); // 9600 baud
	 cfsetispeed(&tio,B9600); // 9600 baud
	 tcsetattr(tty_fd,TCSANOW,&tio);

	*/
	

	while (1)
	{

		bool b = cap.read(frame);

		if (!b)
		{
			cout << "err";
			cap.open(1);
			cap.read(frame);
			// break;
		}

		//resize image
		resize(frame, frame, Size(320, 240));

		GaussianBlur(frame, gauss, Size(5, 5), 0);
		//threshold(gauss, dst, thresh, maxValue, THRESH_BINARY);
	
		cvtColor(gauss,imgHSV, CV_RGB2HSV);
		//white color thresholding
		Scalar whiteMinScalar = Scalar(0, 0, 179);
		Scalar whiteMaxScalar = Scalar(180, 64, 255);

		Scalar yellowMinScalar = Scalar(81, 119, 200);
		Scalar yellowMaxScalar = Scalar(101, 255, 255);

		inRange(imgHSV, whiteMinScalar, whiteMaxScalar, whiteLane);
		inRange(imgHSV, yellowMinScalar, yellowMaxScalar, obstacle);

		erode(whiteLane, whiteLane, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(whiteLane, whiteLane, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		// Contour detection using findContours
		findContours(whiteLane, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		findContours(obstacle, contours_ylw, hirarki_ylw, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		
		// Edge detection using canny detector
		/*int minCannyThreshold = 190;
		int maxCannyThreshold = 230;
		Canny(whiteLane, whiteLane, minCannyThreshold, maxCannyThreshold, 5, true);
		*/ 
		//pilih metode line detecetion antara menggunakan contour atau edge

		// Morphological Operation
		/*Mat k = getStructuringElement(CV_SHAPE_RECT, Size(9, 9)); //MATLAB :k=Ones(9)
		morphologyEx(whiteLane, whiteLane, MORPH_CLOSE, k);*/

		// now applying hough transform TO dETECT Lines in our image
		/*
		vector<Vec4i> lines;
		double rho = 1;
		double theta = CV_PI / 180;
		int threshold = 43;
		double minLinLength = 35;
		double maxLineGap = 210;

		HoughLinesP(whiteLane, lines, rho, theta, threshold, minLinLength, maxLineGap);

		//draw our lines

		for (size_t i = 0; i < lines.size(); i++)
		{
			Vec4i l = lines[i];  // we have 4 elements p1=x1,y1  p2= x2,y2
			Scalar greenColor = Scalar(0, 250, 30);  // B=0 G=250 R=30
			line(frame, Point(l[0], l[1]), Point(l[2], l[3]), greenColor, 3, CV_AA);

		}
		*/
		/*if (contours_ylw.size() > 0)
		{
			Rect s = boundingRect(contours_ylw[0]);
			Point center_ylw = Point(s.x + (s.width / 2));
			line(frame, Point(s.x + (s.width / 2), 200), Point(s.x + (s.width / 2), 250), Scalar(0, 0, 255), 3);
		}*/

		if (contours.size() > 0)
		{	
			Rect r = boundingRect(contours[0]);
			Point pt1 = Point(r.x, r.y);
			Point pt2 = Point(r.x + r.width, r.y + r.height);
			rectangle (frame, pt1, pt2, Scalar(255,0, 0), 3);
			//line(frame, Point(r.x + (r.width / 2), 0), Point(r.x + (r.width / 2), 160), Scalar(255, 0, 0), 3);

			Point2f vtx[4];
			RotatedRect wb = minAreaRect(contours[0]);
			RotatedRect(Point2f(wb.center.x, wb.center.y), Size2f(wb.size.width, wb.size.height), wb.angle);
			float x1 = wb.center.x, y1 = wb.center.y, w = wb.size.width, h = wb.size.height, ang = wb.angle;
			
			wb.points(vtx);
			for (i = 0; i < 4; i++)
				line(frame, vtx[i], vtx[(i + 1) % 4], Scalar(0, 0, 255), 1,LINE_AA);

			if (ang < -45)
				ang = 90 + ang;
			if (w < h && ang > 0)
				ang = (90 - ang)*(-1);
			if (w > h && ang < 0)
				ang = 90 + ang;
			
			drawContours(frame, contours, 0, Scalar(0, 255, 0), 3);
			int sudut = static_cast<int>(ang);
			float sp  = 160;
			//Point center = Point(r.x + (r.width / 2));
			float err = x1 - sp;
			string sdt = to_string(sudut);
			string error = to_string(err);
			//format("%d", err)
			putText(frame, error, Point(100,170),FONT_HERSHEY_SIMPLEX, 2, Scalar(255,0,0),3);
			putText(frame, sdt, Point(10,40),FONT_HERSHEY_SIMPLEX, 2, Scalar(0,0,255),3);

			char bytes[] = { (int)(sudut),(float)(err)};
			//char message[1024];
			//sprintf(message, "%d,%d\n", sudut, err);
			//write(fd, bytes,3);
		}

		imshow("original", frame);
		imshow("White Lane", whiteLane);
		//imshow("Obstacle", obstacle);
		

		if (waitKey(30) == 27)
		{
			cout << "esc";
			break;
		}
	}


	//close(tty_fd);
	//tcsetattr(STDOUT_FILENO, TCSANOW, &old_stdio);

	return 0;
}
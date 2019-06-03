#include <sl_zed/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <vector_types.h>
#include <cstdlib>
#include <math.h>
#include <thread>
#include <stdlib.h>
#include <time.h>
#include <windows.h>

using namespace sl;
using namespace cv;
using namespace std;

//DEFINES
#define PI 3.14159265
#define CAM_DISTANCE 559
#define TRACK_DISTANCE 2200

//ZED
const int MAX_ZED = 12;
sl::Camera* zed1[MAX_ZED];  //array of camera objects
int thresh = 50, N = 1;

//POINTS
vector<vector<Point>> centers(2);  //initial centers of each square
vector<vector<Point>> min_centers(2);  //centers minimized to one point per square
vector<vector<Point>> final_centers(2);  //final five squares


vector<float> leftDepthV(2);
vector<float> topDepthV(2);
vector<float> bottomDepthV(2);
vector<float> rightDepthV(2);
vector<float> centerDepthV(2);
vector<float> Toe(2);
vector<float> Camber(2);
vector<float> centerX(2);

bool found_five[2] = { false, false };

//FUNCTIONS
int ZED_init(InitParameters &init_params); //initialize ZED cameras
static double angle(Point pt1, Point pt2, Point pt0); //calculate angle of three given Point objects
static void findSquares(const cv::Mat& image, vector<vector<Point> >& squares, int x);  //find all squares in image  --referenced and adjusted from opencv github sample
cv::Mat slMat2cvMat(sl::Mat & input);  //convert between ZED image and cv image
void findActuals(int x);   //condense multiple squares on one square into a single point
bool withinRange(Point A, Point B, int range);  //true if two points are within a proximity of each other
void combinationUtil(vector<Point> arr, int n, int r, int index, vector<Point> data, int i, int x);  //matches all groups of size 4  -- referenced and adjusted from geeksforgeeks.com
void fixDepth(sl::Mat depth_map, int x); //drops squares past a depth range
void findFive(int x);  //makes sure squares are equidistant to center square
void setPoints(Point& left, Point& right, Point& center, Point& top, Point& bottom, int x);  //choose correct layout of squares
void mark(cv::Mat &image, Point left, Point top, Point center, Point bottom, Point right);  //mark final squares with a different color
float get_toe_camber(Point left, Point right, sl::Mat point_cloud);  //calculate the toe and camber of the wheel
void _draw(cv::Mat &image, int x);  //draw min_centers onto image
void print_toe(float leftDepth, float rightDepth, Point left, Point right, sl::Mat point_cloud);
void print_camber(float topDepth, float bottomDepth, Point top, Point bottom, sl::Mat point_cloud);
void print_finals(float leftDepth, float topDepth, float bottomDepth, float rightDepth, float centerDepth, Point left, Point top, Point bottom, Point right, Point center, sl::Mat point_cloud, int x);
void get_centers(Point center, sl::float4 &center_right, sl::float4 &center_left, int x, sl::Mat point_cloud);
void grab_loop(int x);
void pp();
bool floatRange(float A, float B, int range);
int loopFeed;


int main(int argc, char **argv) {

	//cv and sl image declarations
	sl::Mat image, depth_map, point_cloud;  //initial ZED img, dpeth map, and point cloud
	cv::Mat image2;  //opencv img to be converted from sl img
	string windows[] = { "Left Camera", "Right Camera" };  //opencv windows
	
	vector<vector<Point>> squares; //2D array of squares

	//finals
	Point left, right, center, top, bottom;  //points for final squares
	float leftDepth, rightDepth, centerDepth, topDepth, bottomDepth; //depth values for final squares
	sl::float4 center_left, center_right;  //center square loc for each cam

	//config for ZED
	InitParameters init_params;
	int nb_detected_zed = ZED_init(init_params);
	string choice;
	cout << "Still image(0) or Live feed(1): ";
	cin >> choice;
	loopFeed = stoi(choice);
	vector<thread*> thread_vec;
	thread_vec.push_back(new thread(grab_loop, 0));
	thread_vec.push_back(new thread(grab_loop, 1));

	float l1, r1, c1, t1, b1, l2, r2, c2, b2, t2;
	while (1) {
		c1 = centerDepthV[0];
		c2 = centerDepthV[1];
		Sleep(50);

		if (!floatRange(c1, centerDepthV[0], 1))
		{
			//if (system("CLS")) system("clear");
			//pp();
		}
		else if (!floatRange(c2, centerDepthV[1], 1))
		{
			//if (system("CLS")) system("clear");
			//pp();
		}
		
	}


	for (auto it : thread_vec) it->join();
	

	return 0;

}

void pp() {
	cout << "-----CAMERA 1-----\t\t\t-----CAMERA2-----\n"
		<< "Distance from right camera: " << floor(centerDepthV[0]) << "mm\t\t\t" << "Distance from left camera: " << floor(centerDepthV[1]) << "mm\n";
}

bool triangleArea(Point A, Point B, Point C){
	cout << "This is the triangle area: " << (A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y)) / 2 << endl;
	if (abs((A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y)) / 2) > 500) {
		return false;
	}
	return true;
}

void grab_loop(int x) {
	sl::Mat image, depth_map, point_cloud;  //initial ZED img, dpeth map, and point cloud
	cv::Mat image2;  //opencv img to be converted from sl img
	string windows[] = { "Left Camera", "Right Camera" };
	Point left, right, center, top, bottom;  //points for final squares
	float leftDepth, rightDepth, centerDepth, topDepth, bottomDepth; //depth values for final squares
	sl::float4 center_cloud;  //center square loc for each cam
	vector<vector<Point>> squares; //2D array of squares
	namedWindow(windows[x]);

	do{
		found_five[x] = false;
		centers[x].clear();
		min_centers[x].clear();
		final_centers[x].clear();
		squares.clear();

		if (zed1[x]->grab() == SUCCESS) {  //if we grab our image

			zed1[x]->retrieveImage(image, VIEW_LEFT); // Get the left image
			zed1[x]->retrieveMeasure(depth_map, MEASURE_DEPTH);  //generate depth map
			zed1[x]->retrieveMeasure(point_cloud, MEASURE_XYZRGBA);  //generate 3D point cloud

			image2 = slMat2cvMat(image); //convert to opencv img

			findSquares(image2, squares, x);


			findActuals(x);


			fixDepth(depth_map, x);


			_draw(image2, x);  //draw unfinalized sq's in yellow

			if (min_centers[x].size() > 4)  //continue if we find at least 5 squares
			{

				findFive(x);
				cout << x << ": FINAL CENTERS COUNT: " << final_centers[x].size() << endl;
				if (final_centers[x].size() == 5) {

					setPoints(left, right, center, top, bottom, x);
					
					mark(image2, left, top, center, bottom, right);

					//get a depth value for each final square
					depth_map.getValue(left.x, left.y, &leftDepth);
					depth_map.getValue(right.x, right.y, &rightDepth);
					depth_map.getValue(center.x, center.y, &centerDepth);
					depth_map.getValue(top.x, top.y, &topDepth);
					depth_map.getValue(bottom.x, bottom.y, &bottomDepth);

					//print current cameras measurements
					//print_finals(leftDepth, topDepth, bottomDepth, rightDepth, centerDepth, left, top, bottom, right, center, point_cloud, x);
					leftDepthV[x] = leftDepth;
					rightDepthV[x] = rightDepth;
					topDepthV[x] = topDepth;
					bottomDepthV[x] = bottomDepth;
					centerDepthV[x] = centerDepth;
					Toe[x] = get_toe_camber(left, right, point_cloud);
					Camber[x] = get_toe_camber(top, bottom, point_cloud);

					float _toe = get_toe_camber(left, right, point_cloud);
					float _camber = get_toe_camber(top, bottom, point_cloud);
					float track = CAM_DISTANCE + centerX[0] - centerX[1];

					string displayText;
					string displayName = "CAMERA " + to_string(x + 1);
					string displayDistance = "Distance: " + to_string((int)centerDepth) + "mm";
					string displayTrack = "Track: " + to_string((int)track) + "mm";
					string displayToe;
					string displayCamber;
					if (leftDepth < rightDepth)
						displayToe = "Toe in: " + to_string(_toe).substr(0,4);
					else
						displayToe = "Toe out: " + to_string(_toe).substr(0,4);

					if (topDepth < bottomDepth)
						displayCamber = "Camber: " + to_string(_camber).substr(0,4);
					else
						displayCamber = "Camber: -" + to_string(_camber).substr(0,4);


					//get_centers(center, center_right, center_left, x, point_cloud);  //save center offsets
					point_cloud.getValue(center.x, center.y, &center_cloud);
					centerX[x] = center_cloud.x;
					Point testLoc;

					rectangle(image2, Point(45, 80), Point(300, 310), Scalar(0, 0, 0),  CV_FILLED, LINE_8, 0); // draw background square
					putText(image2, displayName, Point(50, 100), CV_FONT_HERSHEY_TRIPLEX, .5, Scalar(0, 255, 255), 1, LINE_8, false);
					putText(image2, displayDistance, Point(50, 150), CV_FONT_HERSHEY_TRIPLEX, .5, Scalar(0, 255, 255), 1, LINE_8, false);
					putText(image2, displayTrack, Point(50, 200), CV_FONT_HERSHEY_TRIPLEX, .5, Scalar(0, 255, 255), 1, LINE_8, false);
					putText(image2, displayToe, Point(50, 250), CV_FONT_HERSHEY_TRIPLEX, .5, Scalar(0, 255, 255), 1, LINE_8, false);
					putText(image2, displayCamber, Point(50, 300), CV_FONT_HERSHEY_TRIPLEX, .5, Scalar(0, 255, 255), 1, LINE_8, false);
				}
				imshow(windows[x], image2);  //display image
				waitKey(1);
			}
			else {
				imshow(windows[x], image2);  //display image without final squares
				waitKey(1);
			}
		}
		//cout << endl << endl << "Wheelbase: " << CAM_DISTANCE + centerX[0] - centerX[1] << "mm" << endl;
		//cout << "Track: " << TRACK_DISTANCE - center_right.z - center_left.z << "mm" << endl;
	} while (loopFeed);
	waitKey(0);
}

// create ZED camara objects
int ZED_init(InitParameters &init_params) {
	string res;
	cout << "Enter 720 for 720P and 1080 for 1080P: ";
	cin >> res;
	if (res == "1080") {
		init_params.camera_resolution = RESOLUTION_HD1080; // Use HD1080 video mode
	}
	else if (res == "720") {
		init_params.camera_resolution = RESOLUTION_HD720;  //Use HD720 video mode
	}
	else {
		init_params.camera_resolution = RESOLUTION_HD1080; //1080 by default
	}
	init_params.camera_fps = 15; // Set fps

	std::vector<sl::DeviceProperties> devList = sl::Camera::getDeviceList();  //list of connected Cameras
	int nb_detected_zed = devList.size();

	cout << " Number of ZED Detected : " << nb_detected_zed << endl;

	for (int i = 0; i < nb_detected_zed; i++) {
		zed1[i] = new sl::Camera();
		init_params.input.setFromCameraID(i);

		sl::ERROR_CODE err = zed1[i]->open(init_params);


		if (err != sl::SUCCESS) {
			cout << zed1[i]->getCameraInformation().camera_model << " n" << i << " -> Result: " << sl::toString(err) << endl;
			delete zed1[i];
		}
		else
			cout << zed1[i]->getCameraInformation().camera_model << " n" << i << " SN " <<
			zed1[i]->getCameraInformation().serial_number << " -> Result: " << sl::toString(err) << endl;
	}

	return nb_detected_zed;
}

void get_centers(Point center, sl::float4 &center_right, sl::float4 &center_left, int x, sl::Mat point_cloud) {
	if (x == 0) {
		point_cloud.getValue(center.x, center.y, &center_right);
	}
	else
	{
		point_cloud.getValue(center.x, center.y, &center_left);
	}
}

void print_finals(float leftDepth, float topDepth, float bottomDepth, float rightDepth, float centerDepth, Point left, Point top, Point bottom, Point right, Point center, sl::Mat point_cloud, int x) {

	cout << endl << "----------CAMERA " << x + 1 << "----------" << endl;
	
	cout 
		<< " Top square depth: "<< topDepth << "mm" << endl 
		<< " Bottom square depth: " << bottomDepth << "mm" << endl
		<< " Right square depth: " << rightDepth << "mm" << endl
		<< " Left square depth: " << leftDepth << "mm" << endl
		<< " Center square depth " << centerDepth << "mm" << endl;

	print_toe(leftDepth, rightDepth, left, right, point_cloud);
	print_camber(topDepth, rightDepth, top, bottom, point_cloud);

	cout << "-----------------------------" << endl;

}

void print_toe(float leftDepth, float rightDepth, Point left, Point right, sl::Mat point_cloud) {
	if (leftDepth < rightDepth)
		cout << " Toe in of: " << get_toe_camber(left, right, point_cloud) << " degrees" << endl;
	else
		cout << " Toe out of: " << get_toe_camber(left, right, point_cloud) << " degrees" << endl;
}

void print_camber(float topDepth, float bottomDepth, Point top, Point bottom, sl::Mat point_cloud) {
	if (topDepth < bottomDepth)
		cout << " Camber: " << get_toe_camber(top, bottom, point_cloud) << " degrees" << endl;
	else
		cout << " Camber: -" << get_toe_camber(top, bottom, point_cloud) << " degrees" << endl;
}

void _draw(cv::Mat &image, int x) {
	for (int j = 0; j < min_centers[x].size(); j++) {
		circle(image, min_centers[x][j], 2, Scalar(0, 255, 255), 2, 8, 0);
	}
}

static double angle(Point pt1, Point pt2, Point pt0)  //used to find the angle between three points
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1 * dy1)*(dx2*dx2 + dy2 * dy2) + 1e-10);
}

static void findSquares(const cv::Mat& image, vector<vector<Point> >& squares, int x)  //squares.cpp modified from opencv github pages
{
	squares.clear();

	cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;

	// filter out noise by down and then upscaling the image
	pyrDown(image, pyr, Size(image.cols / 2, image.rows / 2));
	pyrUp(pyr, timg, image.size());
	vector<vector<Point> > contours;

	// find squares in every color plane of the image
	for (int c = 0; c < 3; c++)
	{
		int ch[] = { c, 0 };
		mixChannels(&timg, 1, &gray0, 1, ch, 1);

			// Canny for edge detection
			Canny(gray0, gray, 0, thresh, 5);
			dilate(gray, gray, cv::Mat(), Point(-1, -1));



			// store all our contours
			findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

			vector<Point> approx;

			// test each contour for 4 corners and 90 degrees
			for (size_t i = 0; i < contours.size(); i++)
			{
				approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);
				
				if (approx.size() == 4 &&
					fabs(contourArea(approx)) > 1000 &&
					isContourConvex(approx) &&
					((fabs(contourArea(approx)) / 2073600) < 0.1))
				{
					double maxCosine = 0;

					for (int j = 2; j < 5; j++)
					{
						// find the maximum cosine of the angle between joint edges
						double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
						maxCosine = MAX(maxCosine, cosine);
					}

					//Cosines must be close to 0
					Scalar color;
					if (maxCosine < 0.4) {

						squares.push_back(approx);

						Point center = (approx[0] + approx[1] + approx[2] + approx[3])*0.25;
						centers[x].push_back(center);
					}
				}
			}
		
	}
}


//
void findActuals(int x) {
	int accept = 1;
	for (int i = 0; i < centers[x].size(); i++) {
		accept = 1;
		for (int j = 0; j < min_centers[x].size(); j++) {
			if ((abs(centers[x][i].x - min_centers[x][j].x) < 20) && (abs(centers[x][i].y - min_centers[x][j].y) < 20)) {
				accept = 0;
			}
		}
		if (accept == 1) {
			min_centers[x].push_back(centers[x][i]);
		}
	}

}

//
bool floatRange(float A, float B, int range) {
	if (abs((A - B)) < range) {
		return true;
	}
	return false;
}

bool withinRange(Point A, Point B, int range) {
	if (abs((A.x - B.x)) < range) {
		if (abs(A.y - B.y) < range) {
			return true;
		}
	}
	return false;
}

bool isNotMember(vector<Point> data, Point center, int x) {
	for (int x = 0; x < data.size(); x++) {
		if (data[x] == center) {
			return false;
		}
	}
	cout << x << ": IS NOT MEMBER \n";
	return true;
}

void combinationUtil(vector<Point> arr, int n, int r, int index,
	vector<Point> data, int i, int x) //algo to find all combinations of 4
{
	if (found_five[x]) {
		return;
	}
	Point left, right, center, top, bottom;  //points for final squares
	// Current cobination is ready, print it 
	Point sum = 0;
	if (index == r) {
		for (int j = 0; j < r; j++) {
			sum += data[j];
		}
		Point avg = sum / 4;
		for (int k = 0; k < min_centers[x].size(); k++) {
			if (withinRange(avg, min_centers[x][k], 10)) {
				if (isNotMember(data, min_centers[x][k], x)) {
					final_centers[x].clear();
					for (int j = 0; j < r; j++) {
						final_centers[x].push_back(data[j]);
					}
					final_centers[x].push_back(min_centers[x][k]);
					

					setPoints(left, right, center, top, bottom, x);

					if (triangleArea(left, right, center) && triangleArea(top, bottom, center)) {
						cout << x << ": THIS PASSED UTILS\n";
						found_five[x] = true;
						return;
					}
				}
			}
		}
		return;
	}

	// When no more elements are there to put in data[] 
	if (i >= n)
		return;

	// current is included, put next at next location 
	data[index] = arr[i];
	combinationUtil(arr, n, r, index + 1, data, i + 1, x);

	// current is excluded, replace it with next 
	// (Note that i+1 is passed, but index is not 
	// changed) 
	combinationUtil(arr, n, r, index, data, i + 1, x);
}

void fixDepth(sl::Mat depth_map, int x) { //filter our far away squares
	float temp;
	vector<Point> temp_vec;
	for (int i = 0; i < min_centers[x].size(); i++) {
		depth_map.getValue(min_centers[x][i].x, min_centers[x][i].y, &temp);
		if (temp < 600) {  //keep squares found closer than 600mm away from camera
			temp_vec.push_back(min_centers[x][i]);
		}
	}
	min_centers[x] = temp_vec;
}

void findFive(int x) {
	vector<Point> temp;
	temp.resize(min_centers[x].size());

	combinationUtil(min_centers[x], min_centers[x].size(), 4, 0, temp, 0, x);

}

void setPoints(Point& left, Point& right, Point& center, Point& top, Point& bottom, int x) {
	left = final_centers[x][0];
	int i;
	for (i = 0; i < final_centers[x].size(); i++) {
		if (left.x > final_centers[x][i].x) {
			left = final_centers[x][i];
		}
	}
	right = final_centers[x][0];
	for (i = 0; i < final_centers[x].size(); i++) {
		if (right.x < final_centers[x][i].x) {
			right = final_centers[x][i];
		}
	}
	top = final_centers[x][0];
	for (i = 0; i < final_centers[x].size(); i++) {
		if (top.y > final_centers[x][i].y) {
			top = final_centers[x][i];
		}
	}
	bottom = final_centers[x][0];
	for (i = 0; i < final_centers[x].size(); i++) {
		if (bottom.y < final_centers[x][i].y) {
			bottom = final_centers[x][i];
		}
	}

	int accept = 1;
	for (int i = 0; i < final_centers[x].size(); i++) {
		accept = 1;
		if (final_centers[x][i] == left) {
			accept = 0;
		}
		if (final_centers[x][i] == right) {
			accept = 0;
		}
		if (final_centers[x][i] == top) {
			accept = 0;
		}
		if (final_centers[x][i] == bottom) {
			accept = 0;
		}
		if (accept == 1) {
			center = final_centers[x][i];
		}
	}
}

void mark(cv::Mat &image, Point left, Point top, Point center, Point bottom, Point right) {
	Scalar color(0, 255, 255);
	circle(image, center, 2, Scalar(255, 255, 153), 2, 8, 0);
	circle(image, top, 2, Scalar(255, 255, 153), 2, 8, 0);
	circle(image, left, 2, Scalar(255, 255, 153), 2, 8, 0);
	circle(image, right, 2, Scalar(255, 255, 153), 2, 8, 0);
	circle(image, bottom, 2, Scalar(255, 255, 153), 2, 8, 0);
}

float get_toe_camber(Point left, Point right, sl::Mat point_cloud) {
	sl::float4 left3D, right3D;
	point_cloud.getValue(left.x, left.y, &left3D);
	point_cloud.getValue(right.x, right.y, &right3D);
	float d = sqrt(((left3D.x - right3D.x)*(left3D.x - right3D.x)) + ((left3D.y - right3D.y)*(left3D.y - right3D.y)));
	float z = abs(left3D.z - right3D.z);
	float angle;
	angle = atan(z / d)*180/PI;
	return angle;
}




cv::Mat slMat2cvMat(sl::Mat& input) {
	// Mapping between MAT_TYPE and CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}
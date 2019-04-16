#include <sl_zed/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <vector_types.h>
#include <cstdlib>
#include <math.h>

using namespace sl;
using namespace cv;
using namespace std;

//DEFINES
#define PI 3.14159265
#define CAM_DISTANCE 5600
#define TRACK_DISTANCE 2200

//ZED
const int MAX_ZED = 12;
sl::Camera* zed1[MAX_ZED];  //array of camera objects
int thresh = 50, N = 1;

//POINTS
vector<Point> centers;  //initial centers of each square
vector<Point> min_centers;  //centers minimized to one point per square
vector<Point> final_centers;  //final five squares

//FUNCTIONS
int ZED_init(InitParameters &init_params); //initialize ZED cameras
static double angle(Point pt1, Point pt2, Point pt0); //calculate angle of three given Point objects
static void findSquares(const cv::Mat& image, vector<vector<Point> >& squares);  //find all squares in image  --referenced and adjusted from opencv github sample
cv::Mat slMat2cvMat(sl::Mat & input);  //convert between ZED image and cv image
void findActuals();   //condense multiple squares on one square into a single point
bool withinRange(Point A, Point B, int range);  //true if two points are within a proximity of each other
void combinationUtil(vector<Point> arr, int n, int r, int index, vector<Point> data, int i);  //matches all groups of size 4  -- referenced and adjusted from geeksforgeeks.com
void fixDepth(sl::Mat depth_map); //drops squares past a depth range
void findFive();  //makes sure squares are equidistant to center square
void setPoints(Point& left, Point& right, Point& center, Point& top, Point& bottom);  //choose correct layout of squares
void mark(cv::Mat &image, Point left, Point top, Point center, Point bottom, Point right);  //mark final squares with a different color
float get_toe_camber(Point left, Point right, sl::Mat point_cloud);  //calculate the toe and camber of the wheel
void _draw(cv::Mat &image);  //draw min_centers onto image
void print_toe(float leftDepth, float rightDepth, Point left, Point right, sl::Mat point_cloud);
void print_camber(float topDepth, float bottomDepth, Point top, Point bottom, sl::Mat point_cloud);
void print_finals(float leftDepth, float topDepth, float bottomDepth, float rightDepth, float centerDepth, Point left, Point top, Point bottom, Point right, Point center, sl::Mat point_cloud, int x);
void get_centers(Point center, sl::float4 &center_right, sl::float4 &center_left, int x, sl::Mat point_cloud);

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
    
	//take measurements for each ZED detected
	for (int x = 0; x < nb_detected_zed; x++) {  
		namedWindow(windows[x]);
		centers.clear();
		min_centers.clear();
		final_centers.clear();

		if (zed1[x]->grab() == SUCCESS) {  //if we grab our image
			
			zed1[x]->retrieveImage(image, VIEW_LEFT); // Get the left image
			zed1[x]->retrieveMeasure(depth_map, MEASURE_DEPTH);  //generate depth map
			zed1[x]->retrieveMeasure(point_cloud, MEASURE_XYZRGBA);  //generate 3D point cloud

			image2 = slMat2cvMat(image); //convert to opencv img

			findSquares(image2, squares);

			findActuals(); 

			fixDepth(depth_map);

			_draw(image2);  //draw unfinalized sq's in yellow

			if (min_centers.size() > 4)  //continue if we find at least 5 squares
			{
				findFive();

				setPoints(left, right, center, top, bottom);

				mark(image2, left, top, center, bottom, right);

				//get a depth value for each final square
				depth_map.getValue(left.x, left.y, &leftDepth);
				depth_map.getValue(right.x, right.y, &rightDepth);
				depth_map.getValue(center.x, center.y, &centerDepth);
				depth_map.getValue(top.x, top.y, &topDepth);
				depth_map.getValue(bottom.x, bottom.y, &bottomDepth);

				//print current cameras measurements
				print_finals(leftDepth, topDepth, bottomDepth, rightDepth, centerDepth, left, top, bottom, right, center, point_cloud, x);

				get_centers(center, center_right, center_left, x, point_cloud);  //save center offsets

				imshow(windows[x], image2);  //display image
			}
			else {
				imshow(windows[x], image2);  //display image without final squares
			}
		}
	}

	cout << endl << endl << "Wheelbase: " << CAM_DISTANCE + center_right.x - center_left.x << "mm" << endl;
	cout << "Track: " << TRACK_DISTANCE - center_right.z - center_left.z << "mm" << endl;

	waitKey(0);
	for (int i = 0; i < nb_detected_zed; i++)
		zed1[i]->close();

	return 0;

}

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

void _draw(cv::Mat &image) {
	for (int j = 0; j < min_centers.size(); j++) {
		circle(image, min_centers[j], 2, Scalar(0, 255, 255), 2, 8, 0);
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

static void findSquares(const cv::Mat& image, vector<vector<Point> >& squares)  //squares.cpp modified from opencv github pages
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
					if (maxCosine < 0.1) {

						squares.push_back(approx);

						Point center = (approx[0] + approx[1] + approx[2] + approx[3])*0.25;
						centers.push_back(center);
					}
				}
			}
		
	}
}




void findActuals() {
	int accept = 1;
	for (int i = 0; i < centers.size(); i++) {
		accept = 1;
		for (int j = 0; j < min_centers.size(); j++) {
			if ((abs(centers[i].x - min_centers[j].x) < 10) && (abs(centers[i].y - min_centers[j].y) < 10)) {
				accept = 0;
			}
		}
		if (accept == 1) {
			min_centers.push_back(centers[i]);
		}
	}

}

bool withinRange(Point A, Point B, int range) {
	if (abs((A.x - B.x)) < range) {
		if (abs(A.y - B.y) < range) {
			return true;
		}
	}
	return false;
}

void combinationUtil(vector<Point> arr, int n, int r, int index,
	vector<Point> data, int i) //algo to find all combinations of 4
{
	// Current cobination is ready, print it 
	Point sum = 0;
	if (index == r) {
		for (int j = 0; j < r; j++) {
			sum += data[j];
		}
		Point avg = sum / 4;
		for (int k = 0; k < min_centers.size(); k++) {
			if (withinRange(avg, min_centers[k], 10)) {
				for (int j = 0; j < r; j++) {
					final_centers.push_back(data[j]);
				}
				final_centers.push_back(min_centers[k]);
			}
		}
		return;
	}

	// When no more elements are there to put in data[] 
	if (i >= n)
		return;

	// current is included, put next at next location 
	data[index] = arr[i];
	combinationUtil(arr, n, r, index + 1, data, i + 1);

	// current is excluded, replace it with next 
	// (Note that i+1 is passed, but index is not 
	// changed) 
	combinationUtil(arr, n, r, index, data, i + 1);
}

void fixDepth(sl::Mat depth_map) { //filter our far away squares
	float temp;
	vector<Point> temp_vec;
	for (int i = 0; i < min_centers.size(); i++) {
		depth_map.getValue(min_centers[i].x, min_centers[i].y, &temp);
		if (temp < 600) {  //keep ones closer than 600mm
			temp_vec.push_back(min_centers[i]);
		}
	}
	min_centers = temp_vec;
}

void findFive() {
	vector<Point> temp;
	temp.resize(min_centers.size());

	combinationUtil(min_centers, min_centers.size(), 4, 0, temp, 0);

}

void setPoints(Point& left, Point& right, Point& center, Point& top, Point& bottom) {
	left = final_centers[0];
	int i;
	for (i = 0; i < final_centers.size(); i++) {
		if (left.x > final_centers[i].x) {
			left = final_centers[i];
		}
	}
	right = final_centers[0];
	for (i = 0; i < final_centers.size(); i++) {
		if (right.x < final_centers[i].x) {
			right = final_centers[i];
		}
	}
	top = final_centers[0];
	for (i = 0; i < final_centers.size(); i++) {
		if (top.y > final_centers[i].y) {
			top = final_centers[i];
		}
	}
	bottom = final_centers[0];
	for (i = 0; i < final_centers.size(); i++) {
		if (bottom.y < final_centers[i].y) {
			bottom = final_centers[i];
		}
	}

	int accept = 1;
	for (int i = 0; i < final_centers.size(); i++) {
		accept = 1;
		if (final_centers[i] == left) {
			accept = 0;
		}
		if (final_centers[i] == right) {
			accept = 0;
		}
		if (final_centers[i] == top) {
			accept = 0;
		}
		if (final_centers[i] == bottom) {
			accept = 0;
		}
		if (accept == 1) {
			center = final_centers[i];
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
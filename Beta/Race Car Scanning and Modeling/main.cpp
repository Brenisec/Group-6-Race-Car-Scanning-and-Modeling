///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include <sl_zed/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <vector_types.h>
#include <cstdlib>
#include <math.h>

using namespace sl;
using namespace cv;
using namespace std;

#define PI 3.14159265
#define CAM_DISTANCE 580

const int MAX_ZED = 12;

sl::Camera* zed1[MAX_ZED];
cv::Mat SbSResult[MAX_ZED];
cv::Mat ZED_LRes[MAX_ZED];
int width, height;
long long ZED_Timestamp[MAX_ZED];

int thresh = 50, N = 1;
const char* wndname = "Square Detection Demo";
vector<Point> centers;  //initial centers of each square
vector<Point> min_centers;  //centers minimized to one point per square
vector<Point> final_centers;  //final five squares

struct offset{
	float x;
	float y;
};


static double angle(Point pt1, Point pt2, Point pt0)  //used to find the angle between three points
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1 * dy1)*(dx2*dx2 + dy2 * dy2) + 1e-10);
}

static void findSquares(const cv::Mat& image, vector<vector<Point> >& squares)
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

		// try several threshold levels
		for (int l = 0; l < N; l++)
		{
			// hack: use Canny instead of zero threshold level.
			// Canny helps to catch squares with gradient shading
			if (l == 0)
			{
				// apply Canny. Take the upper threshold from slider
				// and set the lower to 0 (which forces edges merging)
				Canny(gray0, gray, 0, thresh, 5);
// dilate canny output to remove potential
// holes between edge segments
dilate(gray, gray, cv::Mat(), Point(-1, -1));
			}
			else
			{
			// apply threshold if l!=0:
			//     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
			gray = gray0 >= (l + 1) * 255 / N;
			}

			// find contours and store them all as a list
			findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
			//cout << "SIZE: " << contours.size() << endl;

			vector<Point> approx;

			// test each contour
			for (size_t i = 0; i < contours.size(); i++)
			{
				// approximate contour with accuracy proportional
				// to the contour perimeter
				approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);

				// square contours should have 4 vertices after approximation
				// relatively large area (to filter out noisy contours)
				// and be convex.
				// Note: absolute value of an area is used because
				// area may be positive or negative - in accordance with the
				// contour orientation
				
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

					// if cosines of all angles are small
					// (all angles are ~90 degree) then write quandrange
					// vertices to resultant sequence
					Scalar color;
					if (maxCosine < 0.1) {
						//cout << maxCosine << endl;
						squares.push_back(approx);
						//cout << approx[0] << " " << approx[1] << " " << approx[2] << " " << approx[3] << endl;
						Point center = (approx[0] + approx[1] + approx[2] + approx[3])*0.25;
						centers.push_back(center);
						//circle(image, center, 2, color, 2, 8, 0);
					}
				}
			}
		}
	}
}



static void drawSquares(cv::Mat& image, const vector<vector<Point> >& squares)
{
	/*for (size_t i = 0; i < squares.size(); i++)
	{
		const Point* p = &squares[i][0];
		int n = (int)squares[i].size();
		polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 3, LINE_AA);
	}*/
	imshow(wndname, image);
	waitKey(0);
}


cv::Mat slMat2cvMat(sl::Mat & input);

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
			//cout << centers[i] << endl;
		}
	}

	for (int k = 0; k < min_centers.size(); k++) {
		//cout << "new actual" << min_centers[k] << endl;
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
			//cout << data[j];
		}
		printf("\n");
		Point avg = sum / 4;
		for (int k = 0; k < min_centers.size(); k++) {
			if (withinRange(avg, min_centers[k], 10)) {
				//cout << "FOUND A MATCH: " << min_centers[k] << endl;
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
	//cout << "MIN CENTERS SIZE: " << min_centers.size() << endl;
	for (int i = 0; i < min_centers.size(); i++) {
		depth_map.getValue(min_centers[i].x, min_centers[i].y, &temp);
		//cout << "DEPTH: " << temp << endl;
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
	//cout << "in set" << endl;
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
	//cout << "D: " << d << "   Z: " << z << endl;
	float angle;
	angle = atan(z / d)*180/PI;
	return angle;
}

int main(int argc, char **argv) {
	Point left, right, center, top, bottom;
    // Create a ZED camera object
    Camera zed;     // Create a ZED camera object
	vector<vector<Point> > squares;
    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD1080; // Use HD1080 video mode
    init_params.camera_fps = 15; // Set fps at 30
	
	std::vector<sl::DeviceProperties> devList = sl::Camera::getDeviceList();
	int nb_detected_zed = devList.size();

	std::cout << " Number of ZED Detected : " << nb_detected_zed << std::endl;
	// Create every ZED and init them
	for (int i = 0; i < nb_detected_zed; i++) {
		zed1[i] = new sl::Camera();
		init_params.input.setFromCameraID(i);

		sl::ERROR_CODE err = zed1[i]->open(init_params);


		if (err != sl::SUCCESS) {
			cout << zed1[i]->getCameraInformation().camera_model << " n°" << i << " -> Result: " << sl::toString(err) << endl;
			delete zed1[i];
		}
		else
			cout << zed1[i]->getCameraInformation().camera_model << " n°" << i << " SN " <<
			zed1[i]->getCameraInformation().serial_number << " -> Result: " << sl::toString(err) << endl;
	}
	sl::Mat image1sl, image2sl;
	cv::Mat img1, img2;

	string windows[] = { "window1", "window2" };
	sl::float4 center_left, center_right;

	for (int x = 0; x < 2; x++) {
		sl::float4 point3D;

		int i = 0;

		float sum_toe = 0;
		float avg_toe;

		sl::Mat image;
		sl::Mat depth_map;
		cv::Mat image2;
		sl::Mat point_cloud;
		float leftDepth, rightDepth, centerDepth, topDepth, bottomDepth;
		namedWindow(windows[x]);
		//while(i < 10){
		centers.clear();
		min_centers.clear();
		final_centers.clear();


		if (zed1[x]->grab() == SUCCESS) {  //if we grab our image
			zed1[x]->retrieveImage(image, VIEW_LEFT); // Get the left image
			zed1[x]->retrieveMeasure(depth_map, MEASURE_DEPTH);
			zed1[x]->retrieveMeasure(point_cloud, MEASURE_XYZRGBA);

			image2 = slMat2cvMat(image); //convert to opencv img

			findSquares(image2, squares);   //find all square contours - store in squares

			findActuals();  //minimize extra squares

			fixDepth(depth_map);  //minimize far away squares

			for (int j = 0; j < min_centers.size(); j++) {   //draw circles on ALL squares
				circle(image2, min_centers[j], 2, Scalar(0, 255, 255), 2, 8, 0);
			}

			if (min_centers.size() > 4)  //continue if we find at least 5 squares
			{
				findFive();  //minimize more extra squares

				setPoints(left, right, center, top, bottom);  //find top bottom etc..

				mark(image2, left, top, center, bottom, right);  //draw only final five squares


				depth_map.getValue(left.x, left.y, &leftDepth);
				depth_map.getValue(right.x, right.y, &rightDepth);
				depth_map.getValue(center.x, center.y, &centerDepth);
				depth_map.getValue(top.x, top.y, &topDepth);
				depth_map.getValue(bottom.x, bottom.y, &bottomDepth);

				cout << endl << "------CAMERA " << x+1 << "------" << endl;
				cout << " Top square depth: " << topDepth << endl << " Bottom square depth: " << bottomDepth << endl << " Right square depth: " << rightDepth << endl << " Left square depth: " << leftDepth << endl << " Center square depth " << centerDepth << endl;

				//TOE
				if (leftDepth < rightDepth)
					cout << "Toe in of: " << get_toe_camber(left, right, point_cloud) << endl;
				else
					cout << "Toe out of: " << get_toe_camber(left, right, point_cloud) << endl;
				sum_toe += get_toe_camber(left, right, point_cloud);

				//CAMBER
				if (topDepth < bottomDepth)
					cout << "Camber: " << get_toe_camber(top, bottom, point_cloud) << endl;
				else
					cout << "Camber: -" << get_toe_camber(top, bottom, point_cloud) << endl;
				cout << endl;

				if (x == 0) {
					point_cloud.getValue(center.x, center.y, &center_right);
				}
				else
				{
					point_cloud.getValue(center.x, center.y, &center_left);
				}

				imshow(windows[x], image2);
			}
			else {
				imshow(windows[x], image2);
				//waitKey(1);
			}
		}
		i++;
	}
	
	cout << "Wheelbase: " << CAM_DISTANCE + center_right.x - center_left.x << endl;
	waitKey(0);
		zed1[0]->close();
		zed1[1]->close();
		return 0;
	
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
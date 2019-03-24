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
#include <cstdlib>

using namespace sl;
using namespace cv;
using namespace std;


int thresh = 50, N = 1;
const char* wndname = "Square Detection Demo";
vector<Point> center_guess;
vector<Point> center_actual;



static double angle(Point pt1, Point pt2, Point pt0)
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

	// down-scale and upscale the image to filter out the noise
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
					isContourConvex(approx))
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
					if (maxCosine < 0.05) {
						//cout << maxCosine << endl;
						squares.push_back(approx);
						//cout << approx[0] << " " << approx[1] << " " << approx[2] << " " << approx[3] << endl;
						Point center = (approx[0] + approx[1] + approx[2] + approx[3])*0.25;
						center_guess.push_back(center);
						circle(image, center, 2, color, 2, 8, 0);
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
	for (int i = 0; i < center_guess.size(); i++) {
		accept = 1;
		for (int j = 0; j < center_actual.size(); j++) {
			if ((abs(center_guess[i].x - center_actual[j].x) < 5) && (abs(center_guess[i].y - center_actual[j].y) < 5)) {
				accept = 0;
			}
		}
		if (accept == 1) {
			center_actual.push_back(center_guess[i]);
		}
	}

	for (int k = 0; k < center_actual.size(); k++) {
		//cout << "new actual" << center_actual[k] << endl;
	}
}

void setPoints(Point& left, Point& right, Point& center, Point& top, Point& bottom) {
	//cout << "in set" << endl;
	left = center_actual[0];
	int i;
	for (i = 0; i < center_actual.size(); i++) {
		if (left.x > center_actual[i].x) {
			left = center_actual[i];
		}
	}
	right = center_actual[0];
	for (i = 0; i < center_actual.size(); i++) {
		if (right.x < center_actual[i].x) {
			right = center_actual[i];
		}
	}
	top = center_actual[0];
	for (i = 0; i < center_actual.size(); i++) {
		if (top.y > center_actual[i].y) {
			top = center_actual[i];
		}
	}
	bottom = center_actual[0];
	for (i = 0; i < center_actual.size(); i++) {
		if (bottom.y < center_actual[i].y) {
			bottom = center_actual[i];
		}
	}

	int accept = 1;
	for (int i = 0; i < center_actual.size(); i++) {
		accept = 1;
		if (center_actual[i] == left) {
			accept = 0;
		}
		if (center_actual[i] == right) {
			accept = 0;
		}
		if (center_actual[i] == top) {
			accept = 0;
		}
		if (center_actual[i] == bottom) {
			accept = 0;
		}
		if (accept == 1) {
			center = center_actual[i];
		}
	}
}


int main(int argc, char **argv) {
	Point left, right, center, top, bottom;
    // Create a ZED camera object
    Camera zed;     // Create a ZED camera object
	vector<vector<Point> > squares;
	cout << "Opening Camera..." << endl;
    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD1080; // Use HD1080 video mode
    init_params.camera_fps = 30; // Set fps at 30

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        exit(-1);
	cout << "Camera Found" << endl;


    int i = 0;
	sl::Mat image;
	sl::Mat depth_map;
	cv::Mat image2;
	float leftDepth, rightDepth, centerDepth, topDepth, bottomDepth;
    if (zed.grab() == SUCCESS) {
        // A new image is available if grab() returns SUCCESS
		cout << "Retreiving image..." << endl;
		zed.retrieveImage(image, VIEW_LEFT); // Get the left image
		cout << "Making depth map" << endl;
		zed.retrieveMeasure(depth_map, MEASURE_DEPTH);
		image2 = slMat2cvMat(image);
		cout << "Finding Squares..." << endl;
		findSquares(image2, squares);
		findActuals();
		setPoints(left, right, center, top, bottom);
		depth_map.getValue(left.x, left.y, &leftDepth);
		depth_map.getValue(right.x, right.y, &rightDepth);
		depth_map.getValue(center.x, center.y, &centerDepth);
		depth_map.getValue(top.x, top.y, &topDepth);
		depth_map.getValue(bottom.x, bottom.y, &bottomDepth);

		
		cout << " Top square depth: " << topDepth << endl << " Bottom square depth: " << bottomDepth << endl << " Right square depth: " << rightDepth << endl << " Left square depth: " << leftDepth << endl << " Center square depth " << centerDepth << endl;


		drawSquares(image2, squares);
        unsigned long long timestamp = zed.getCameraTimestamp(); // Get the timestamp at the time the image was captured
        i++;
    }
    // Close the camera
    zed.close();
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
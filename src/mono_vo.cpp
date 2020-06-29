/*
Shivani & Raghav
24th June-2020
Project-2 We detect ORB, BRIEF descriptor, Scale Estimation, Good Matched points in an image.
 */

#include "moildev.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv/cv.h>
#include <iterator>
#include <vector>
using namespace cv;
using namespace std;

#define MAX_FRAME 922
#define MIN_NUM_FEAT 2000

#include "mono_vo.h"
string groundtruth_location;
char *filename_format;
double all_angles[6];
double zoom;
Mat source_front_extraction;
Mat source_left_extraction;
Mat source_right_extraction;

mono_vo::mono_vo(string name){
	Name = name;
	textOrg = Point(10, 50);
	textZoom = Point(320,100);
	textFront= Point (10,160);
	textLeft= Point (10,175);
	textRight= Point (10,190);
	textOrg1 = Point(10,100);
	textOrg2 = Point(10,115);
	textOrg3 = Point(10,130);
	textOrg4 = Point(10,145);
	trajOrg = Point(300,100);

	focal = 616.8560;	// T265 dataset
	pp = Point2d(427,394);

	traj = Mat::zeros(600, 600, CV_8UC3);
	trajColor = CV_RGB(255,0,0);
}

double mono_vo::getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
	string line;
	int i = 0;
	ifstream myfile (groundtruth_location.c_str());
	double x =0, y=0, z = 0;
	double x_prev, y_prev, z_prev;
	if (myfile.is_open())
	{
		while (( getline (myfile,line) ) && (i<=frame_id))
		{
			z_prev = z;
			x_prev = x;
			y_prev = y;
			std::istringstream in(line);
			for (int j=0; j<12; j++)  {
				in >> z ;
				if (j==7) y=z;
				if (j==3)  x=z;
			}
			i++;
		}
		myfile.close();
	}

	else {
		cout << "Unable to open file";
		return 0;
	}
	return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;
}
void mono_vo::featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{

	//this function automatically gets rid of points for which tracking fails
	vector<float> err;
	Size winSize=Size(21,21);
	TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

	calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

	//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
	int indexCorrection = 0;
	for( int i=0; i<status.size(); i++)
	{  Point2f pt = points2.at(i- indexCorrection);
	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
		if((pt.x<0)||(pt.y<0))	{
			status.at(i) = 0;
		}
		points1.erase (points1.begin() + (i - indexCorrection));
		points2.erase (points2.begin() + (i - indexCorrection));
		indexCorrection++;
	}
	}
}
void mono_vo::featureDetection(Mat img_1, vector<Point2f>& points1)	{


	vector<KeyPoint> keypoints_1;
	int fast_threshold =20.5;
	bool nonmaxSuppression = true;
	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	KeyPoint::convert(keypoints_1, points1, vector<int>());
}

void mono_vo::init(Point txtPos, Point trajPos, Scalar color, double trajscale) {
	numFrame = 0;
	textOrg = txtPos;
	trajOrg = trajPos;
	trajColor = color;
	trajScale = trajscale;
}

void mono_vo::runOneFrame(Mat Frame, Mat outputTraj,int count) {

	if ( numFrame == 0 ){
		cvtColor(Frame, img_1, COLOR_BGR2GRAY);
		numFrame++;
	}
	else if( numFrame == 1 )
	{
		cvtColor(Frame, img_2, COLOR_BGR2GRAY);
		featureDetection(img_1, points1);
		featureTracking(img_1, img_2, points1, points2, status);

		E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
		recoverPose(E, points2, points1, R, t, focal, pp, mask);
		prevImage = img_2;
		prevFeatures = points2;
		R_f = R.clone();
		t_f = t.clone();
		numFrame++;
	}
	else
	{  // for numFrame >=2
		cvtColor(Frame, currImage, COLOR_BGR2GRAY);
		vector<uchar> status;
		featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
		scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

		if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)))		//FORMULAAA
		{
			t_f = t_f - scale * R_f * t ;
			R_f = R.t() * R_f ;
		}
		
		if (prevFeatures.size() < MIN_NUM_FEAT)	{

			featureDetection(prevImage, prevFeatures);
			featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
		}
		prevImage = currImage.clone();
		prevFeatures = currFeatures;

		int x = int(t_f.at<double>(0) * trajScale) + trajOrg.x;
		int y = int(t_f.at<double>(2) * trajScale) + trajOrg.y;

		circle(outputTraj, Point(x, y) ,1, trajColor, 2);

		rectangle( outputTraj, textOrg, Point(textOrg.x + 550, textOrg.y-20), CV_RGB(10,10,10), CV_FILLED);

		sprintf(text, "%s : x = %02fm y = %02fm z = %02fm", Name.c_str(), t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
		sprintf(Z, "%s  Zoom Factor = %02fm ", Name1.c_str(),zoom );
		sprintf(Frontview, "%s  Front Alpha = %02fm Beta = %02fm ", Name2.c_str(), all_angles[0], all_angles[1] );
		sprintf(Leftview, "%s Left Alpha = %02fm Beta = %02fm ", Name3.c_str(), all_angles[2], all_angles[3] );
		sprintf(Rightview, "%s  Right Alpha = %02fm Beta = %02fm ", Name4.c_str(), all_angles[4], all_angles[5] );

		putText(outputTraj, text, textOrg, fontFace, fontScale, CV_RGB(200,200,200), thickness, 8);
		putText(outputTraj, Z, textZoom, fontFace, fontScale, CV_RGB(200,200,200), thickness, 8);
		putText(outputTraj, Frontview, textFront, fontFace, fontScale, CV_RGB(200,200,200), thickness, 8);
		putText(outputTraj, Leftview, textLeft, fontFace, fontScale, CV_RGB(200,200,200), thickness, 8);
		putText(outputTraj, Rightview, textRight, fontFace, fontScale, CV_RGB(200,200,200), thickness, 8);

		putText(outputTraj, text1, textOrg1, fontFace, fontScale, CV_RGB(200,200,200), thickness, 8);
		putText(outputTraj, text2, textOrg2, fontFace, fontScale, CV_RGB(255,0,0), thickness, 8);
		putText(outputTraj, text3, textOrg3, fontFace, fontScale, CV_RGB(0,255,0), thickness, 8);
		putText(outputTraj, text4, textOrg4, fontFace, fontScale, CV_RGB(0,0,255), thickness, 8);

		for (auto point : currFeatures)
		{
			cv::drawMarker(Frame, cv::Point(point.x, point.y),CV_RGB(0, 255, 0), cv::MARKER_TILTED_CROSS, 2, 1,cv::LINE_AA) ;
		}
		numFrame++;
	}
}
int mono_vo::runAll() {

	namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
	namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

	//read the first two frames from the dataset
	sprintf(filename, filename_format, 1);
	Mat img_1_c = imread(filename);
	sprintf(filename, filename_format, 2);
	Mat img_2_c = imread(filename);

	if ( !img_1_c.data || !img_2_c.data ) {
		std::cout<< " --(!) Error reading images " << std::endl; return -1;
	}

	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

	featureDetection(img_1, points1);
	featureTracking(img_1,img_2,points1,points2, status);

	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points2, points1, R, t, focal, pp, mask);

	prevImage = img_2;
	prevFeatures = points2;

	R_f = R.clone();
	t_f = t.clone();

	for(int numFrame=3; numFrame < MAX_FRAME; numFrame++)	{
		sprintf(filename, filename_format, numFrame);
		Mat currImage_c = imread(filename);
		cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
		vector<uchar> status;
		featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
		cout << "  feature matching 1 " <<endl ;
		Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);

		for(int i=0;i<prevFeatures.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
			prevPts.at<double>(0,i) = prevFeatures.at(i).x;
			prevPts.at<double>(1,i) = prevFeatures.at(i).y;

			currPts.at<double>(0,i) = currFeatures.at(i).x;
			currPts.at<double>(1,i) = currFeatures.at(i).y;
		}

		scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));
		if ((scale>0.15)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

			t_f = t_f - scale * R_f * t ;
			R_f = R.t() * R_f ;

		}
		// a re-detection is triggered in case the number of features being tracked go below a particular threshold
		if (prevFeatures.size() < MIN_NUM_FEAT)	{
			//cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
			//cout << "trigerring re-detection" << endl;
			featureDetection(prevImage, prevFeatures);
			featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);

		}
		prevImage = currImage.clone();
		prevFeatures = currFeatures;

		int x = 2*int(t_f.at<double>(0)) + 300;
		int y = 2*int(t_f.at<double>(2)) + 100;
		circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

		rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
		sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
		putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

		imshow( "Road facing camera", currImage_c );
		imshow( "Trajectory", traj );
		waitKey(1);
	}
	return 0;
}
Point2f getGroundTruth(string filename, int frame_id)	{
	string line;
	int i = 0;
	double z;
	ifstream myfile (filename.c_str());
	Point2f Pt(0,0);
	if (myfile.is_open())
	{
		while (( getline (myfile,line) ) && (i < frame_id))i++;
		std::istringstream in(line);
		for (int j=0; j<12; j++)  {
			in >> z ;
			if (j==3) Pt.x = (float)z;
			if (j==11) Pt.y = (float)z;
		}
		myfile.close();
	}
	else {
		cout << "Unable to open file";
	}
	return Pt;
}

void mono_vo::FeatureExtraction(Mat img_1 , Mat img_2, int check)
{
	///--- initialize
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	Mat descriptors_1, descriptors_2;
	Ptr<FeatureDetector> detector = ORB::create();
	Ptr<DescriptorExtractor> descriptor = ORB::create();


	Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

	//--- Step 1: Detect Oriented FAST corner position
	detector->detect ( img_1,keypoints_1 );
	detector->detect ( img_2,keypoints_2 );

	//--- Step 2: Calculate the BRIEF descriptor based on the corner position
	descriptor->compute ( img_1, keypoints_1, descriptors_1 );
	descriptor->compute ( img_2, keypoints_2, descriptors_2 );

	Mat outimg1;
	drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	imshow("ORB Feature Point",outimg1);

	//--- Step 3: Match the BRIEF descriptors in the two images, using Hamming distance
	vector<DMatch> matches;
	//BFMatcher matcher ( NORM_HAMMING );
	matcher->match ( descriptors_1, descriptors_2, matches );

	// Step 4: Matching point pair screening
	double min_dist=10000, max_dist=0;

	for ( int i = 0; i < descriptors_1.rows; i++ )
	{
		double dist = matches[i].distance;
		if ( dist < min_dist ) min_dist = dist;
		if ( dist > max_dist ) max_dist = dist;
	}

	//When the distance between the descriptors is greater than twice the minimum distance,
	//It is considered that the match is wrong. But sometimes the minimum distance is very small, set an empirical value of 30 as the lower limit.
	std::vector< DMatch > good_matches;
	for ( int i = 0; i < descriptors_1.rows; i++ )
	{
		if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
		{
			good_matches.push_back ( matches[i] );
		}
	}
	Mat img_match;
	Mat img_goodmatch;
	drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
	drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );

	std::ofstream Maximum;
	std::ofstream Minimum;
	std::ofstream ImageMatch;
	std::ofstream ImagegoodMatch;
	std::ofstream keypointsize;
	std::ofstream keypoint2;

	if (check==0)
	{
		imshow ( "FRONT: All matching points", img_match );
		imshow ( "FRONT: Match point pairs after optimization", img_goodmatch );
	}

	else if (check==1)
	{

		imshow ( "LEFT: All matching points", img_match );
		imshow ( "LEFT: Match point pairs after optimization", img_goodmatch );
		//img_goodmatch.copyTo(Feature_Extraction[1]);
	}

	else
	{
		imshow ( "RIGHT: All matching points", img_match );
		imshow ( "RIGHT: Match point pairs after optimization", img_goodmatch);
	}
	waitKey(100);

}

void multi_vo()	{

	Moildev *md = new Moildev();
	Mat image_input, image_input_s;
	Mat image_display[6];
	Mat mapX[6], mapY[6];
	Mat Traj = Mat::zeros(1080, 610, CV_8UC3);
	int w = 848, h = 800;
	double m_ratio;

	md->Config("car", 3, 3,
			427,394, 1, //center
			848, 800, 1.68, // resolution
			0, 0, -24.964, 38.2, -16.956, 183.42 //calibration
	);

	double calibrationWidth = md->getImageWidth();
	m_ratio = w / calibrationWidth;
	Mat image_result[3];
	// For 3 view trajectory
	for (uint i = 0; i < 3; i++){									//For view change here
		mapX[i] = Mat(h, w, CV_32F);
		mapY[i] = Mat(h, w, CV_32F);
		image_result[i] = Mat(h, w, CV_32F);
	}
	//double zoom = 3.8;
	md->AnyPointM2((float*) mapX[0].data, (float*) mapY[0].data, mapX[0].cols,
			mapX[0].rows, all_angles[0], all_angles[1], zoom, m_ratio); 			// front view
	md->AnyPointM2((float*) mapX[1].data, (float*) mapY[1].data, mapX[1].cols,
			mapX[1].rows, all_angles[2], all_angles[3], zoom, m_ratio); 			// left view
	md->AnyPointM2((float*) mapX[2].data, (float*) mapY[2].data, mapX[2].cols,
			mapX[2].rows, all_angles[4], all_angles[5], zoom, m_ratio); 			// right view

	namedWindow("Input", WINDOW_AUTOSIZE);
	namedWindow("Trajectory", WINDOW_AUTOSIZE);
	namedWindow("Front", WINDOW_AUTOSIZE);
	namedWindow("Left", WINDOW_AUTOSIZE);
	namedWindow("Right", WINDOW_AUTOSIZE);
	namedWindow("FRONT: Match point pairs after optimization", WINDOW_AUTOSIZE);
	namedWindow("LEFT: Match point pairs after optimization", WINDOW_AUTOSIZE);
	namedWindow("RIGHT: Match point pairs after optimization", WINDOW_AUTOSIZE);

	moveWindow("Input", 0, 0);
	moveWindow("Trajectory", 1280, 0);
	moveWindow("Left", 0, 440);
	moveWindow("Front", 470, 440);
	moveWindow("Right", 875, 440);
	moveWindow("FRONT: Match point pairs after optimization", 170,800);
	moveWindow("LEFT: Match point pairs after optimization", 270,800);
	moveWindow("RIGHT: Match point pairs after optimization",470,800);

	char filename[100];
	string Title[3] = { "Front", "Left", "Right"};
	mono_vo *vo[3];
	for (int i = 0; i <3; i++) {
		vo[i] = new mono_vo(Title[i]);
	}
	cout << "multi vo 1" << endl;
	double drawScale = 3.0;

	vo[0]->init(Point(10, 30), Point(300, 500), CV_RGB(255, 0, 0), drawScale);// Red is front
	vo[1]->init(Point(10, 50), Point(302, 500), CV_RGB(0, 255, 0), drawScale);// Green is left
	vo[2]->init(Point(10, 70), Point(304, 500), CV_RGB(0, 0, 255), drawScale);// Blue is right

	int numFrame = 1;
	bool isExit = false;
	while (numFrame <= MAX_FRAME && !isExit)
	{
		sprintf(filename, filename_format, numFrame);
		image_input = imread(filename);
		if (!image_input.empty()) {
			cv::resize(image_input, image_input_s, Size(640, 480));

			// For 3 view trajectory
			for (int i = 0; i < 3; i++) {                							 //For view change here
				remap(image_input, image_result[i], mapX[i], mapY[i],
						INTER_CUBIC, BORDER_CONSTANT, Scalar(0, 0, 0));
				vo[i]->runOneFrame(image_result[i], Traj,i);
				cv::resize(image_result[i], image_display[i], Size(400, 300));

				imshow(Title[i], image_display[i]);
				vo[i]->FeatureExtraction(image_input, image_display[i],i);			// feature extract

			}
			Point2f Pt = getGroundTruth(groundtruth_location, numFrame);
			int x = int(Pt.x * drawScale) + 300;
			int y = int(Pt.y * drawScale) + 500;

			circle(Traj, Point(x, y), 1, CV_RGB(200, 200, 200), 2);

			imshow("Input", image_input_s);
			imshow("Trajectory", Traj);
			cout << "Frame# : " << numFrame << endl;
		}
		cout << "For closing program, please press 'esc'  OR to pause program please press 'Z'-- !!!within one second !!!"<<endl;

		char c = waitKey(30);
		if (c == 27) {  							// 27 is equal to esc
			isExit = true;
		} else if (c == 90 || c== 122) {			// 90 is ASCII value of 'Z'
			char x;
			while (1) {
				cout <<"   To resume program again, press 'Q'--  "<<endl;
				x = waitKey(30);
				if (x == 81 || x==113) {			// 81 is ASCII value of 'Q'
					break;
				}

			} // end of while
		} // end of else-if
		numFrame++;
	}
	waitKey(0);
	putText(Traj, "Monocular VO", Point(250, 900), FONT_HERSHEY_PLAIN, 2,Scalar::all(255), 2, 8) ;
	sprintf(filename, "/home/shivani/Documents/github/mainmoil-mono-vo/result/%d.png") ;
	imwrite(filename, Traj) ;

	return;
}

int main( int argc, char** argv )	{

	groundtruth_location = argv[1];
	filename_format = argv[2];
	string zoom_factor;
	zoom_factor = argv[3];
	zoom = atof(zoom_factor.c_str());
	cout<<"print value of zoom  " << zoom <<endl;
	int i=0;
	for( int idx=4; idx<argc; idx++)
	{
		char * angles= argv[idx];
		char *angle= strtok (angles,  ",");

		while (angle!=  NULL)
		{
			string angleStr=angle;
			all_angles[i++]= atof(angleStr.c_str());

			angle=strtok(NULL, ",");
		}
	}
	cout<<"print the value of all-angles 0  "<<all_angles[0]<<endl;
	cout<<"print the value of all-angles 1  "<<all_angles[1]<<endl;
	cout<<"print the value of all-angles 2  "<<all_angles[2]<<endl;
	cout<<"print the value of all-angles 3  "<<all_angles[3]<<endl;
	cout<<"print the value of all-angles 4  "<<all_angles[4]<<endl;
	cout<<"print the value of all-angles 5  "<<all_angles[5]<<endl;
#if 0 // single vo

	mono_vo *vo1= new mono_vo("A");
	vo->runAll();

#else

	multi_vo();

#endif
}

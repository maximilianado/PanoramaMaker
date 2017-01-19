#include "Pcv2.h"

String POINT = "point";
String LINE = "line";

// compute homography
/*
base		first set of points x'
attach		second set of points x
return		homography H, so that x' = Hx
*/
Mat Pcv2::homography2D(Mat& base, Mat& attach){
	// TO DO !!!
	//condition -> to get T and T'
	Mat T = getCondition2D(base);
	Mat T2 = getCondition2D(attach);
	//transform -> to get tranformed base and attach
	Mat base2 = applyH(base, T, POINT);
	Mat attach2 = applyH(attach, T2, POINT);
	//design matrix -> to get the design matrix A
	Mat A = getDesignMatrix_homography2D(base2, attach2);
	//SVD -> solve SVD to get a homography matrix H
	Mat H = solve_dlt(A);
	//decondition -> decondition H
	decondition(T,T2,H);
    	return H;
}

// solve homogeneous equation system by usage of SVD
/*
A		the design matrix
return	solution of the homogeneous equation system
*/
Mat Pcv2::solve_dlt(Mat& A){
	// TO DO !!!
	Mat Aquat = Mat::zeros(A.cols, A.cols, CV_32FC1);
	if (A.rows < A.cols) {
		for (int i=0; i<A.rows; i++) {
			A.row(i).copyTo(Aquat.row(i));
		}
	} else {
		Aquat = A;
	}
	SVD svd = cv::SVD(Aquat, SVD::FULL_UV);
	int index = 0;
	float min = svd.w.at<float>(0);
	for (int i=1; i<svd.w.rows; i++) {
		if (min>svd.w.at<float>(i)) {
			index = i;
			min = svd.w.at<float>(i);
		}
	}
	Mat column = svd.vt.t().col(index);
	Mat H = Mat(3,3,CV_32FC1);
	for (int i=0; i<column.rows; i++) {
		H.at<float>(i/3,i%3) = column.at<float>(i,0);
	}
    	return H;
}

// decondition a homography that was estimated from conditioned point clouds
/*
T_base		conditioning matrix T' of first set of points x'
T_attach	conditioning matrix T of second set of points x
H			conditioned homography that has to be un-conditioned (in-place)
*/
void Pcv2::decondition(Mat& T_base, Mat& T_attach, Mat& H){
  	// TO DO !!!
	H = T_base.inv()*H*T_attach;
}

/*
base	point x' --> x' = H * x
attach	point x --> x' = H * x
return 	design matrix to be computed
*/
Mat getDesignMatrixOfOnePoint(Mat base, Mat attach) {
	Mat designMatrix = Mat::zeros(2,9,CV_32FC1);
	for (int i = 0; i<3; i++) {
		designMatrix.at<float>(0,i) = -base.at<float>(2,0)*attach.at<float>(i,0);
		designMatrix.at<float>(1,i+3) = -base.at<float>(2,0)*attach.at<float>(i,0);
		designMatrix.at<float>(0,i+6) = base.at<float>(0,0)*attach.at<float>(i,0);
		designMatrix.at<float>(1,i+6) = base.at<float>(1,0)*attach.at<float>(i,0);
	}
	return designMatrix;
}

// define the design matrix as needed to compute 2D-homography
/*
base	first set of points x' --> x' = H * x
attach	second set of points x --> x' = H * x
return	the design matrix to be computed
*/
Mat Pcv2::getDesignMatrix_homography2D(Mat& base, Mat& attach){
	// TO DO !!!
	Mat designMatrix = Mat::zeros((base.cols*2),9, CV_32FC1);//for random number of points
	for (int i=0; i<base.cols; i++) {
		Mat dMOnePoint = getDesignMatrixOfOnePoint(base.col(i), attach.col(i));
		dMOnePoint.row(0).copyTo(designMatrix.row(i*2));
		dMOnePoint.row(1).copyTo(designMatrix.row((i*2)+1));
	}
    	return designMatrix;
}

// apply transformation to set of points
/*
H			matrix representing the transformation
geomObj		matrix with input objects (one per column)
type		the type of the geometric object (for now: only point and line)
return		transformed objects (one per column)
*/
Mat Pcv2::applyH(Mat& geomObj, Mat& H, string type){

  	// if object is a point
  	if (type.compare(POINT) == 0){
    		Mat ret = H * geomObj;
    		return ret;
  	}
  	// if object is a line
  	if (type.compare(LINE) == 0){
    		Mat ret = (H.inv()).t() * geomObj;
    		return ret;
  	}
  	cout << "ERROR: Do not know how to move " << type << endl;
  	cout << "Returning original object" << endl;
  	cout << "Press enter key to continue..." << endl;
  	cin.get();
  	return geomObj;
}

// get the conditioning matrix of given points
/*
p		the points as matrix
return	the condition matrix (already allocated)
*/
Mat Pcv2::getCondition2D(Mat& p){
	// TO DO !!!
	float tx = 0;
	float ty = 0;
	float sx = 0;
	float sy = 0;
	for (int i=0; i<p.cols; i++) {
		tx = tx + p.at<float>(0,i);
		ty = ty + p.at<float>(1,i);
	}
	tx = tx / p.cols;
	ty = ty / p.cols;
	for (int i=0; i<p.cols; i++) {
		float x = tx-p.at<float>(0,i);
		sx = sx + sqrt(pow(x,2));
		float y = ty-p.at<float>(1,i);
		sy = sy + sqrt(pow(y,2));
	}
	sx = sx / p.cols;
	sy = sy / p.cols;
	Mat ret = Mat::eye(3,3,CV_32FC1);
	ret.at<float>(0,0) = 1/sx;
	ret.at<float>(1,1) = 1/sy;
	ret.at<float>(0,2) = -tx/sx;
	ret.at<float>(1,2) = -ty/sy;
	return ret;
}

/* *****************************
  GIVEN FUNCTIONS
***************************** */

// stitch two images together by transforming one of them by a given homography
/*
base		the base image
attach		the image to be attached
H		the homography to warp the second image
panorama	the resulting image
*/
Mat Pcv2::stitch(Mat& base, Mat& attach, Mat& H){

    // compute corners of warped image
    Mat corners(1, 4, CV_32FC2);
    corners.at<Vec2f>(0, 0) = Vec2f(0,0);
    corners.at<Vec2f>(0, 1) = Vec2f(0,attach.rows);
    corners.at<Vec2f>(0, 2) = Vec2f(attach.cols,0);
    corners.at<Vec2f>(0, 3) = Vec2f(attach.cols,attach.rows);
    perspectiveTransform(corners, corners, H);
    
    // compute size of resulting image and allocate memory
    float x_start = min( min( corners.at<Vec2f>(0, 0)[0], corners.at<Vec2f>(0, 1)[0]), (float)0);
    float x_end   = max( max( corners.at<Vec2f>(0, 2)[0], corners.at<Vec2f>(0, 3)[0]), (float)base.cols);
    float y_start = min( min( corners.at<Vec2f>(0, 0)[1], corners.at<Vec2f>(0, 2)[1]), (float)0);
    float y_end   = max( max( corners.at<Vec2f>(0, 1)[1], corners.at<Vec2f>(0, 3)[1]), (float)base.rows);

    // create translation matrix in order to copy both images to correct places
    Mat T = Mat::zeros(3,3,CV_32FC1);
    T.at<float>(0, 0) = 1;
    T.at<float>(1, 1) = 1;
    T.at<float>(2, 2) = 1;
    T.at<float>(0, 2) = -x_start;
    T.at<float>(1, 2) = -y_start;
  
    // change homography to take necessary translation into account
    T = T * H;
    // warp second image and copy it to output image
    Mat panorama;
    warpPerspective(attach, panorama, T, Size(x_end - x_start + 1, y_end - y_start + 1), CV_INTER_LINEAR);

    // copy base image to correct position within output image
    Mat roi(panorama, Rect(-x_start,-y_start,base.cols, base.rows));
    base.copyTo(roi, base);
  
    return panorama;
 
}

// mouse call back to get points and draw circles
/*
event	specifies encountered mouse event
x,y	position of mouse pointer
flags	not used here
param	a struct containing used IplImage and window title
*/
void getPointsCB(int event, int x, int y, int flags, void* param){

  // cast to structure
  struct winInfo* win = (struct winInfo*)param;
  
  switch(event){
    // if left mouse button was pressed
    case CV_EVENT_LBUTTONDOWN:{
		// create point representing mouse position
		Point2f p = Point2f(x,y);
		// draw green point
		circle(win->img, p, 2, Scalar(0, 255, 0), 2);
		// draw green circle
		circle(win->img, p, 15, Scalar(0, 255, 0), 2);
		// update image
		imshow(win->name.c_str(), win->img);
		// add point to point list
		win->pointList.push_back(p);
    }break;
  }
}

// display two images and catch the point pairs marked by left mouse clicks
// points will be in homogeneous coordinates
/*
base		structure containing base image
attach		structure containing image that has to be attached
p_base		points within the base image (to be defined by this method)
p_attach	points within the second image (to be defined by this method)
*/
int Pcv2::getPoints(struct winInfo& base, struct winInfo& attach, Mat& p_base, Mat& p_attach){
 
    cout << endl;
    cout << "Please select at least four points by clicking at the corresponding image positions:" << endl;
    cout << "Firstly click at the point that shall be transformed (within the image to be attached), followed by a click on the corresponding point within the base image" << endl;
    cout << "Continue until you have collected as many point pairs as you wish" << endl;
    cout << "Stop the point selection by pressing any key" << endl << endl;
  
    // show input images and install mouse callback
    namedWindow( base.name.c_str(), 0 );
    imshow( base.name.c_str(), base.img );
    base.pointList.clear();
    setMouseCallback(base.name.c_str(), getPointsCB, (void*) &base);
    namedWindow( attach.name.c_str(), 0 );
    imshow( attach.name.c_str(), attach.img );
    attach.pointList.clear();
    setMouseCallback(attach.name.c_str(), getPointsCB, (void*) &attach);
    // wait until any key was pressed
    waitKey(0);
    
    destroyWindow( base.name.c_str() );
    destroyWindow( attach.name.c_str() );

    // allocate memory for point-lists (represented as matrix)
    int numOfPoints = base.pointList.size();
    p_base = Mat(3, numOfPoints, CV_32FC1);
    p_attach = Mat(3, numOfPoints, CV_32FC1);
    // read points from global variable, transform them into homogeneous coordinates
    for(int p = 0; p<numOfPoints; p++){
		p_attach.at<float>(0, p) = attach.pointList.at(p).x;
		p_attach.at<float>(1, p) = attach.pointList.at(p).y;
		p_attach.at<float>(2, p) = 1;
		p_base.at<float>(0, p) = base.pointList.at(p).x;
		p_base.at<float>(1, p) = base.pointList.at(p).y;
		p_base.at<float>(2, p) = 1;
    }
    return numOfPoints;

}

// function loads input image, calls processing function, and saves result
/*
fname	path to input image
*/
void Pcv2::run(string fnameBase, string fnameLeft, string fnameRight){

    // titles of some windows
    string winName1 = string("Base image");
    string winName2 = string("Image to attach");
    string winName3 = string("Panorama");

    // load image first two images, paths in argv[1] and argv[2]
    Mat baseImage = imread(fnameBase);
    Mat attachImage = imread(fnameLeft);
    if (!baseImage.data){
		cerr << "ERROR: Cannot read image ( " << fnameBase << endl;
		cin.get();
		exit(-1);
	}
	if (!attachImage.data){
		cerr << "ERROR: Cannot read image ( " << fnameLeft << endl;
		cin.get();
		exit(-1);
	}

    // fuse image data and window title
    struct winInfo base;
    base.img = baseImage.clone();
    base.name = winName1;    
    struct winInfo attach;
    attach.img = attachImage.clone();
    attach.name = winName2;
   
    // get corresponding points within the two image
    // start with one point within the attached image, then click on corresponding point in base image
    Mat p_basis, p_attach;
    int numberOfPointPairs = getPoints(base, attach, p_basis, p_attach);
    
    // just some putput
    cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
    cout << endl << "Points in base image:" << endl;
    cout << p_basis << endl;
    cout << endl << "Points in second image:" << endl;
    cout << p_attach << endl;

    // calculate homography
    Mat H = homography2D(p_basis, p_attach);
    
    // create panorama
    Mat panorama = stitch(baseImage, attachImage, H);

    // display panorama (resizeable)
    namedWindow( winName3.c_str(), 0 );
    imshow( winName3.c_str(), panorama );
    waitKey(0);
    destroyWindow( winName3.c_str());
    
    // panorama is new base image, third image is the image to attach
    baseImage = panorama;
    // load third image
    attachImage = imread(fnameRight);
    if (!attachImage.data){
		cout << "ERROR: Cannot read image ( " << fnameRight << " )" << endl;
		cin.get();
		exit(-1);
	}
    
    // fuse image data and window title
    base.img = baseImage.clone();
    attach.img = attachImage.clone();
    
    // get corresponding points within the two image
    // start with one point within the attached image, then click on corresponding point in base image
    numberOfPointPairs = getPoints(base, attach, p_basis, p_attach);
    
    // just some putput
    cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
    cout << endl << "Points in base image:" << endl;
    cout << p_basis << endl;
    cout << endl << "Points in second image:" << endl;
    cout << p_attach << endl;
    
    // calculate homography
    H = homography2D(p_basis, p_attach);
    
    // create panorama
    panorama = stitch(baseImage, attachImage, H);
     
    // display panorama (resizeable)
    namedWindow( winName3.c_str(), 0 );
    imshow( winName3.c_str(), panorama );
    waitKey(0);
    destroyWindow( winName3.c_str());
    
    imwrite("panorama.png", panorama);

}

// the following function is for load some pictures from a folder and create a panorama picture
// the constant kPathSeperator is used at this function
/*
kPathSeperator for seperate folders and files at windows and linux systems
*/
const char kPathSeparator =
#ifdef _WIN32
                            '\\';
#else
                            '/';
#endif

// function loads input images from a folder, calls processing function, and saves result at this folder
/*
fname	path to input image
*/
void Pcv2::run2(string directoryName){
    string fname = "";
    string fnameLeft = "";
    string fnameRight = "";

    // titles of some windows
    string winName1 = string("Base image");
    string winName2 = string("Image to attach");
    string winName3 = string("Panorama");

    Mat baseImage;
    Mat attachImage;

    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(directoryName.c_str())) == NULL) {
        cout << "Can not opening " << directoryName << endl;
    }

    vector <string> ordredImageNames;
    while ((dirp = readdir(dp)) != NULL) {
	string name = string(dirp->d_name);
	if (name == "." || name == "..") continue;
	ordredImageNames.push_back(name);
    }
    closedir(dp);
    sort(ordredImageNames.begin(), ordredImageNames.end());
    int index = ordredImageNames.size()/2;
    for ( int i=0; i<ordredImageNames.size(); i++) {
    //for ( vector<string>::iterator name = ordredImageNames.begin(); name != ordredImageNames.end(); name++ ) {
	index = index+(pow(-1,i)*i);
cout << "bis hier hin komme ich " << index << endl;
	string name = ordredImageNames[index]; 
	if (name != "." && name != "..") {
		fname = directoryName+kPathSeparator+name;
	} else {
		continue;	
	}
        cout << endl << "Add image: " << name << endl;

    	// load image
	if (!baseImage.data) {
    		baseImage = imread(fname);
		continue;
	} else {
    		attachImage = imread(fname);
	}
    	if (!baseImage.data){
		cerr << "ERROR: Cannot read base image ( " << fname << endl;
		cin.get();
		exit(-1);
	}
	if (!attachImage.data){
		cerr << "ERROR: Cannot read attach image ( " << fname << endl;
		cin.get();
		exit(-1);
	}

    	// fuse image data and window title
    	struct winInfo base;
    	base.img = baseImage.clone();
    	base.name = winName1;    
    	struct winInfo attach;
    	attach.img = attachImage.clone();
    	attach.name = winName2;
   
    	// get corresponding points within the two image
    	// start with one point within the attached image, then click on corresponding point in base image
    	Mat p_basis, p_attach;
    	int numberOfPointPairs = getPoints(base, attach, p_basis, p_attach);
    
    	// just some putput
    	cout << "Number of defined point pairs: " << numberOfPointPairs << endl;
    	cout << endl << "Points in base image:" << endl;
    	cout << p_basis << endl;
    	cout << endl << "Points in second image:" << endl;
    	cout << p_attach << endl;

    	// calculate homography
    	Mat H = homography2D(p_basis, p_attach);
    
    	// create panorama
    	Mat panorama = stitch(baseImage, attachImage, H);

    	// display panorama (resizeable)
    	namedWindow( winName3.c_str(), 0 );
    	imshow( winName3.c_str(), panorama );
    	waitKey(0);
    	destroyWindow( winName3.c_str());
    
    	// panorama is new base image, third image is the image to attach
    	baseImage = panorama;
    }

    imwrite(directoryName+kPathSeparator+"panorama.png", baseImage);
}

// function calls processing functions
// output is tested on "correctness" 
void Pcv2::test(void){
	test_homography2D();
	test_getCondition2D();
	test_getDesignMatrix_homography2D();
	test_solve_dlt();
	test_decondition();
}

void Pcv2::test_getCondition2D(void){
	Mat p = (Mat_<float>(3,4) << 93, 729, 703, 152, 617, 742, 1233, 1103, 1, 1, 1, 1);
	Mat Ttrue = (Mat_<float>(3,3) << 1./296.75, 0, -419.25/296.75, 0, 1./244.25, -923.75/244.25, 0, 0, 1);
	
	Mat Test = getCondition2D(p);
	if ( (Test.rows != 3) || (Test.cols != 3) || (Test.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::getCondition2D(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	Test.convertTo(Test, CV_32FC1);
	Test = Test / Test.at<float>(2,2);
	float eps = pow(10,-3);
	if (sum(abs(Test - Ttrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::getCondition2D(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
}

void Pcv2::test_getDesignMatrix_homography2D(void){
	
	Mat p1 = (Mat_<float>(3,4) << -1, 1, 1, -1, -1, -1, 1, 1,  1, 1, 1, 1);
	Mat p2 = (Mat_<float>(3,4) << -1.0994103, 1.0438079, 0.9561919, -0.90058976,  -1.2558856, -0.74411488, 1.2661204, 0.73387909, 1, 1, 1, 1);
	
	Mat Aest = getDesignMatrix_homography2D(p1, p2);
	if ( ( (Aest.rows != 8) && (Aest.rows != 9) ) || (Aest.cols != 9) || (Aest.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::getDesignMatrix_homography2D(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	Aest.convertTo(Aest, CV_32FC1);
	Mat Atrue;
	if (Aest.rows == 8)
		Atrue = (Mat_<float>(8,9) << 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 1.0994103, 1.2558856, -1, -1.0438079, 0.74411488, -1, 0, 0, 0, 1.0438079, -0.74411488, 1, 0, 0, 0, -1.0438079, 0.74411488, -1, -1.0438079, 0.74411488, -1, -0.9561919, -1.2661204, -1, 0, 0, 0, 0.9561919, 1.2661204, 1, 0, 0, 0, -0.9561919, -1.2661204, -1, 0.9561919, 1.2661204, 1, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, -0.90058976, 0.73387909, 1);
	else
		Atrue = (Mat_<float>(9,9) << 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 1.0994103, 1.2558856, -1, -1.0438079, 0.74411488, -1, 0, 0, 0, 1.0438079, -0.74411488, 1, 0, 0, 0, -1.0438079, 0.74411488, -1, -1.0438079, 0.74411488, -1, -0.9561919, -1.2661204, -1, 0, 0, 0, 0.9561919, 1.2661204, 1, 0, 0, 0, -0.9561919, -1.2661204, -1, 0.9561919, 1.2661204, 1, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, -0.90058976, 0.73387909, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	float eps = pow(10,-3);
	if (sum(abs(Aest - Atrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::getDesignMatrix_homography2D(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
		
}

void Pcv2::test_solve_dlt(void){
	Mat A = (Mat_<float>(8,9) << 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 0, 0, 0, 1.0994103, 1.2558856, -1, 1.0994103, 1.2558856, -1, -1.0438079, 0.74411488, -1, 0, 0, 0, 1.0438079, -0.74411488, 1, 0, 0, 0, -1.0438079, 0.74411488, -1, -1.0438079, 0.74411488, -1, -0.9561919, -1.2661204, -1, 0, 0, 0, 0.9561919, 1.2661204, 1, 0, 0, 0, -0.9561919, -1.2661204, -1, 0.9561919, 1.2661204, 1, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, 0, 0, 0, 0.90058976, -0.73387909, -1, -0.90058976, 0.73387909, 1);
	Mat Hest = solve_dlt(A);
	if ( (Hest.rows != 3) || (Hest.cols != 3) || (Hest.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::solve_dlt(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	Hest.convertTo(Hest, CV_32FC1);
	Hest = Hest / Hest.at<float>(2,2);
	Mat Htrue = (Mat_<float>(3,3) << 0.57111752, -0.017852778, 0.013727478, -0.15091757, 0.57065326, -0.04098846, 0.024604173, -0.041672569, 0.56645769);
	Htrue = Htrue / Htrue.at<float>(2,2);
	float eps = pow(10,-3);
	if (sum(abs(Hest - Htrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::solve_dlt(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
}

void Pcv2::test_decondition(void){
	
	Mat H = (Mat_<float>(3,3) << 0.57111752, -0.017852778, 0.013727478, -0.15091757, 0.57065326, -0.04098846, 0.024604173, -0.041672569, 0.56645769);
	Mat T1 = (Mat_<float>(3,3) << 1./319.5, 0, -1, 0, 1./319.5, -1, 0, 0, 1);
	Mat T2 = (Mat_<float>(3,3) << 1./296.75, 0, -419.25/296.75, 0, 1./244.25, -923.75/244.25, 0, 0, 1);
	decondition(T1, T2, H);
	if ( (H.rows != 3) || (H.cols != 3) || (H.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::decondition(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	H.convertTo(H, CV_32FC1);
	H = H / H.at<float>(2,2);
	Mat Htrue = (Mat_<float>(3,3) << 0.9304952, -0.11296108, -16.839279, -0.19729686, 1.003845, -601.02362, 0.00012028422, -0.00024751772, 1);
	float eps = pow(10,-3);
	if (sum(abs(H - Htrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::decondition(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
}

void Pcv2::test_homography2D(void){

	Mat p1 = (Mat_<float>(3,4) << 0, 639, 639, 0, 0, 0, 639, 639, 1, 1, 1, 1);	
	Mat p2 = (Mat_<float>(3,4) << 93, 729, 703, 152, 617, 742, 1233, 1103, 1, 1, 1, 1);
		
	Mat Hest = homography2D(p1, p2);
	if ( (Hest.rows != 3) || (Hest.cols != 3) || (Hest.channels() != 1) ){
		cout << "Warning: There seems to be a problem with Pcv2::homography2D(..)!" << endl;
		cout << "\t==> Wrong dimensions!" << endl;
		cin.get();
	}
	Hest.convertTo(Hest, CV_32FC1);
	Hest = Hest / Hest.at<float>(2,2);
	Mat Htrue = (Mat_<float>(3,3) << 0.9304952, -0.11296108, -16.839279, -0.19729686, 1.003845, -601.02362, 0.00012028422, -0.00024751772, 1);
	float eps = pow(10,-3);
	if (sum(abs(Hest - Htrue)).val[0] > eps){
		cout << "Warning: There seems to be a problem with Pcv2::homography2D(..)!" << endl;
		cout << "\t==> Wrong or inaccurate calculations!" << endl;
		cin.get();
	}
}

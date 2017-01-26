#include <iostream>
#include <opencv2/opencv.hpp>

#include <sys/types.h>
#include <dirent.h>
#include <string>

using namespace std;
using namespace cv;

struct winInfo {Mat img; string name; vector<Point2f> pointList;};

class PMakerBG{

	public:
		// constructor
		PMakerBG(void){};
		// destructor
		~PMakerBG(void){};
		
		// processing routine
		void run(string, string, string, bool automatic);
		// processing routine for more then 3 pictures
		void run2(string, bool automatic);
		// testing routine
		void test(void);

	private:
		// given functions
		int getPoints(struct winInfo&, struct winInfo&, Mat&, Mat&, bool automatic);
		Mat stitch(Mat&, Mat&, Mat&);
		// functions to be implemented
		// --> edit ONLY these functions!
		Mat homography2D(Mat&, Mat&);
		void decondition(Mat&, Mat&, Mat&);
		Mat getDesignMatrix_homography2D(Mat&, Mat&);
		Mat applyH(Mat&, Mat&, string);
		Mat getCondition2D(Mat&);
		Mat solve_dlt(Mat&);

		// test functions
		void test_homography2D(void);
		void test_decondition(void);
		void test_getDesignMatrix_homography2D(void);
		void test_getCondition2D(void);
		void test_solve_dlt(void);
		
};

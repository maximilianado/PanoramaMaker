#include <iostream>

#include "PMakerUI.h"
#include "PMakerBG.h"

using namespace std;

// usage: path to image in argv[1]
// main function. loads and saves image
int main(int argc, char** argv) {

	// will contain path to input image (taken from argv[1])
	string fnameBase, fnameLeft, fnameRight, directoryName;

    	// check if image paths were defined
   	 if (!(argc == 4 || argc == 2)){
		cout << "Usage: pcv2 <path to base image> <path to 2nd image> <path to 3rd image>" << endl;
		cout << "OR pcv2 <path to imageDirectory>" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		return -1;
    	}else{
	    if (argc == 4) {
		    // if yes, assign it to variable fname
		    fnameBase = argv[1];
		    fnameLeft = argv[2];
		    fnameRight = argv[3];
	    } else {
	   	    directoryName = argv[1];
	    }
	}
	
	// construct processing object
	Pcv2 pcv2;

	// run some test routines
	pcv2.test();

	// start processing
	if (argc == 4) {
		pcv2.run(fnameBase, fnameLeft, fnameRight);
	} else {
		pcv2.run2(directoryName);
	}

	cout << "Press enter to continue..." << endl;
	cin.get();

	return 0;

}

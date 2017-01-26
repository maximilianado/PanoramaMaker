#include <iostream>

#include "PMakerUI.h"
#include "PMakerBG.h"

using namespace std;

// usage: path to image in argv[1]
// main function. loads and saves image
int main(int argc, char** argv) {

	// will contain path to input image (taken from argv[1])
	string fnameBase, fnameLeft, fnameRight, directoryName;

	bool automatic = false;
    	// check if image paths were defined
   	 if (!(argc > 5 && argc < 3)){
		cout << "Usage: pcv2 <path to base image> <path to 2nd image> <path to 3rd image> <automatic?>" << endl;
		cout << "OR pcv2 <path to imageDirectory> <automatic?>" << endl;
		cout << "Press enter to continue..." << endl;
		cin.get();
		return -1;
    	}else{
	    if (argc > 3) {
		    // if yes, assign it to variable fname
		    fnameBase = argv[1];
		    fnameLeft = argv[2];
		    fnameRight = argv[3];
		    if (argc==5) automatic = argv[4];
	    } else {
	   	    directoryName = argv[1];
		    if (argc==3) automatic = argv[2];
	    }
	}
	
	// construct processing object
	PMakerBG pmaker;

	// run some test routines
	pmaker.test();

	// start processing
	if (argc == 4) {
		pmaker.run(fnameBase, fnameLeft, fnameRight, automatic);
	} else {
		pmaker.run2(directoryName, automatic);
	}

	cout << "Press enter to continue..." << endl;
	cin.get();

	return 0;

}

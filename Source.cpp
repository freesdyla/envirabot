#include "swift/SwiftLib.h"
#include <iostream>
#include "TOF_Swift.h"
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;
bool exit();

int main(int argc, const char**argv) {
	TOF_Swift camh;
	Mat image;
	camh.setCamera("169.254.4.17");
	//Sleep(300);
	camh.listNodes();
	camh.setPower(256);
	camh.setupStreaming();
	camh.startStreaming();
	camh.takePicture();
	//camh.streamPointCloud(); Not working properly
	std::cout << "Press Q to continue"<< std::endl;
	while (!(exit())) {
		camh.takePicture();
		camh.Visualizer();
	}
	return 0;

}

bool exit() {
	return (GetKeyState('Q') & 0x8000);
}

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "PTAM.h"

int main(int argc, char** argv) {
    ptam::PTAM mPTAM(cv::Size(640, 480));
    cv::VideoCapture cap(1);
    cv::Mat frame;
    cap >> frame;
    while(true) {
        mPTAM.process(frame);
        cv::imshow("frame", frame);
        int k = cv::waitKey(10);
        if(k == 's') {
            mPTAM.startTracking();
        } else if(k == 'p') {
            mPTAM.stopTracking();
        } else if(k == 'q') {
            break;
        }
    }
    return 0;
}

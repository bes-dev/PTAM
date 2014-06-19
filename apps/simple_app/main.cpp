#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "PTAM.h"

int main() {
    ptam::PTAM mPTAM(cv::Size(640, 480));
    cv::VideoCapture cap(0);
    cv::Mat frame;
    bool startTracking = false;
    while(true) {
        cap >> frame;
        if(startTracking) {
            mPTAM.process(frame);
            std::vector<cv::Point2f> corners = mPTAM.getCorners();
            for(int i = 0; i < corners.size(); i++) {
                cv::circle(frame, corners[i], 2, cv::Scalar(255, 0, 255));
            }
        }
        cv::imshow("frame", frame);
        char k = cv::waitKey(10);
        if(k == 's') {
            if(!startTracking) startTracking = true;
            mPTAM.startTracking();
        } else if(k == 'p') {
            mPTAM.stopTracking();
        } else if(k == 'q') {
            break;
        }
    }
    return 0;
}

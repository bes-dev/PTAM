#include "PTAM.h"
#include "image.h"
#include "KeyFrame.h"

namespace ptam {

PTAM::PTAM(cv::Size frameSize) {
    mCamera = new ATANCamera();
    mMap = new Map;
    mMapMaker = new MapMaker(*mMap, *mCamera);
    mSize = frameSize;
    mTracker = new Tracker(CVD::ImageRef(mSize.width, mSize.height), *mCamera, *mMap, *mMapMaker);
    mCvdFrame = CVD::ImageRef(mSize.width, mSize.height);
}

PTAM::~PTAM() {
    delete mCamera;
    delete mMap;
    delete mMapMaker;
    delete mTracker;
}

void PTAM::process(const cv::Mat& frame) {
    // Grab video frame in black and white from videobuffer
    convertOpenCVToCvdByte(frame, mCvdFrame);
    mTracker->TrackFrame(mCvdFrame);
}

void PTAM::startTracking() {
    mTracker->StartTracking();
}

void PTAM::stopTracking() {
    mTracker->StopTracking();
}

std::vector<float> PTAM::getPose() {
    SE3<> se3pose = mTracker->GetCurrentPose();
    std::vector<float> pose(7);
    Vector<3> translation, rotation;

    translation = se3pose.get_translation();
    rotation = se3pose.get_rotation().ln();
    pose[0]=translation[0];
    pose[1]=translation[1];
    pose[2]=translation[2];
    pose[3]=norm(rotation) / 3.14159 * 360.0;
    pose[4]=rotation[0] / pose[3];
    pose[5]=rotation[1] / pose[3];
    pose[6]=rotation[2] / pose[3];
    return pose;
}

std::vector<cv::Point2f> PTAM::getPoints() {
    return mTracker->cvPoints;
}

void PTAM::convertOpenCVToCvdByte(const cv::Mat& in, CVD::Image<CVD::byte>& out) {
    cv::Mat clone = in.clone();
    cv::Mat_<cv::Vec3b>& frame_p = (cv::Mat_<cv::Vec3b>&)clone;
    for (int i = 0; i < in.size().height; i++) {
        for (int j = 0; j < in.size().width; j++) {
            out[i][j] = (frame_p(i,j)[0] + frame_p(i,j)[1] + frame_p(i,j)[2]) / 3;
        }
    }
}

void PTAM::convertOpenCVToCvdRGB(const cv::Mat& in, CVD::Image<CVD::Rgb<CVD::byte>>& out) {
    cv::Mat clone = in.clone();
    cv::Mat_<cv::Vec3b>& frame_p = (cv::Mat_<cv::Vec3b>&)clone;
    for (int i = 0; i < in.size().height; i++) {
        for (int j = 0; j < in.size().width; j++) {
            out[i][j].red = frame_p(i,j)[2];
            out[i][j].green = frame_p(i,j)[1];
            out[i][j].blue = frame_p(i,j)[0];
        }
    }
}

}

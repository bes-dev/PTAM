#ifndef __PTAM_H__
#define __PTAM_H__

#include "Tracker.h"
#include "ATANCamera.h"
#include "Map.h"
#include "MapMaker.h"
#include "opencv2/core/core.hpp"

namespace ptam {

class PTAM {
public:
    PTAM(cv::Size frameSize);
    ~PTAM();

    void process(const cv::Mat& frame);
    void startTracking();
    void stopTracking();

    std::vector<float> getPose();
    std::vector<cv::Point2f> getCorners();
private:
    Map *mMap;
    MapMaker *mMapMaker;
    Tracker *mTracker;
    ATANCamera *mCamera;
    CVD::Image<CVD::byte> mCvdFrame;
    cv::Size mSize;

    void convertOpenCVToCvdByte(const cv::Mat& in, CVD::Image<CVD::byte>& out);
    void convertOpenCVToCvdRGB(const cv::Mat& in, CVD::Image<CVD::Rgb<CVD::byte>>& out);
};

}

#endif /*__PTAM_H__*/

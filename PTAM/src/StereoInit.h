#ifndef __STEREO_INIT_H__
#define __STEREO_INIT_H__

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"

#include <sstream>
#include <vector>
#include <list>
#include "globals.h"
#include <opencv2/core/core.hpp>

// This struct is used for initial correspondences of the first stereo pair.
struct Trail {
    MiniPatch mPatch;
    CVD::ImageRef irCurrentPos;
    CVD::ImageRef irInitialPos;
};

class StereoInit {
public:
    StereoInit(MapMaker& mm);
    int process(KeyFrame& mCurrentKF, SE3<>& mse3CamFromWorld, bool& mbUserPressedSpacebar);
protected:
    MapMaker &mMapMaker;            // The class which maintains the map
    bool mIsFirstFrame;

    void Reset();

    // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
    int TrackForInitialMap(KeyFrame &mCurrentKF, SE3<>& mse3CamFromWorld, bool& mbUserPressedSpacebar);      // This is called by TrackFrame if there is not a map yet.
    enum {TRAIL_TRACKING_NOT_STARTED, TRAIL_TRACKING_STARTED, TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
    void TrailTracking_Start(KeyFrame &mCurrentKF);     // First frame of initial trail tracking. Called by TrackForInitialMap.
    int  TrailTracking_Advance(KeyFrame &mCurrentKF);   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
    std::list<Trail> mlTrails;      // Used by trail tracking
    KeyFrame mFirstKF;              // First of the stereo pair
    KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches

};

#endif /*__STEREO_INIT_H__*/

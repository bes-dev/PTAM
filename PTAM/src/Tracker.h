//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"

#include <sstream>
#include <vector>
#include <list>
#include "globals.h"
#include <opencv2/core/core.hpp>
#include "StereoInit.h"

class TrackerData;

class Tracker {
public:
    Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm);
    ~Tracker();

    // TrackFrame is the main working part of the tracker: call this every frame.
    void TrackFrame(CVD::Image<CVD::byte> &imFrame);

    inline SE3<> GetCurrentPose() {
        return mse3CamFromWorld;
    }

    // Gets messages to be printed on-screen for the user.
    void StartTracking();
    void StopTracking();

    KeyFrame& getCurrentKeyFrame();
    std::vector<cv::Point2f> cvPoints;

protected:
    KeyFrame mCurrentKF;            // The current working frame as a keyframe struct

    // The major components to which the tracker needs access:
    Map &mMap;                      // The map, consisting of points and keyframes
    MapMaker &mMapMaker;            // The class which maintains the map
    ATANCamera mCamera;             // Projection model
    Relocaliser mRelocaliser;       // Relocalisation module

    CVD::ImageRef mirSize;          // Image size of whole image

    void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.


    // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
    StereoInit *mStereoInit;

    // Methods for tracking the map once it has been made:
    void TrackMap();                // Called by TrackFrame if there is a map.
    void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
    void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
    void UpdateMotionModel();       // Motion model is updated after TrackMap
    int SearchForPoints(std::vector<TrackerData*> &vTD, int nRange, int nFineIts);  // Finds points in the image
    Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD, double dOverrideSigma = 0.0, bool bMarkOutliers = false); // Updates pose from found points.
    SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
    SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
    Vector<6> mv6CameraVelocity;    // Motion model
    double mdVelocityMagnitude;     // Used to decide on coarse tracking
    double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
    bool mbDidCoarse;               // Did tracking use the coarse tracking stage?

    // Interface with map maker:
    int mnFrame;                    // Frames processed since last reset
    int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
    void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe

    // Tracking quality control:
    int manMeasAttempted[LEVELS];
    int manMeasFound[LEVELS];
    enum {BAD, DODGY, GOOD} mTrackingQuality;
    int mnLostFrames;

    // Relocalisation functions:
    bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
    bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

    // Frame-to-frame motion init:
    SmallBlurryImage *mpSBILastFrame;
    SmallBlurryImage *mpSBIThisFrame;
    void CalcSBIRotation();
    Vector<6> mv6SBIRot;
    bool mbUseSBIInit;

    // User interaction for initial tracking:
    bool mbUserPressedSpacebar;
};

#endif

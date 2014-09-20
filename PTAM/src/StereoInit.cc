#include "StereoInit.h"
#include "Log.h"

StereoInit::StereoInit(MapMaker& mm): mMapMaker(mm), mnInitialStage(TRAIL_TRACKING_NOT_STARTED) {
}

void StereoInit::Reset() {
    mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
    mlTrails.clear();
}

int StereoInit::process(KeyFrame& mCurrentKF, SE3<>& mse3CamFromWorld, bool& mbUserPressedSpacebar) {
    return TrackForInitialMap(mCurrentKF, mse3CamFromWorld, mbUserPressedSpacebar);
}

// Routine for establishing the initial map. This requires two spacebar presses from the user
// to define the first two key-frames. Salient points are tracked between the two keyframes
// using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
// break it.) The salient points are stored in a list of `Trail' data structures.
// What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
int StereoInit::TrackForInitialMap(KeyFrame &mCurrentKF, SE3<>& mse3CamFromWorld, bool& mbUserPressedSpacebar) {
    // MiniPatch tracking threshhold.
    static int gvnMaxSSD = TrackerMiniPatchMaxSSD;
    MiniPatch::mnMaxSSD = gvnMaxSSD;

    // What stage of initial tracking are we at?
    if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED) {
        if(mbUserPressedSpacebar) { // First spacebar = this is the first keyframe
            mbUserPressedSpacebar = false;
            TrailTracking_Start(mCurrentKF);
            mnInitialStage = TRAIL_TRACKING_STARTED;
        } else {
            LOG("Point camera at planar scene and press spacebar to start tracking for initial map.");
        }
        return 0;
    }

    if(mnInitialStage == TRAIL_TRACKING_STARTED) {
        int nGoodTrails = TrailTracking_Advance(mCurrentKF);  // This call actually tracks the trails
        if(nGoodTrails < 10) {// if most trails have been wiped out, no point continuing.
            Reset();
            return 1;
        }

        // If the user pressed spacebar here, use trails to run stereo and make the intial map..
        if(mbUserPressedSpacebar) {
            mbUserPressedSpacebar = false;
            std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
            for(std::list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
                vMatches.push_back(std::pair<CVD::ImageRef, CVD::ImageRef>(i->irInitialPos,
                                                            i->irCurrentPos));
            mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);  // This will take some time!
            mnInitialStage = TRAIL_TRACKING_COMPLETE;
        } else {
            LOG("Translate the camera slowly sideways, and press spacebar again to perform stereo init.");
        }
    }
    return 0;
}

// The current frame is to be the first keyframe!
void StereoInit::TrailTracking_Start(KeyFrame &mCurrentKF) {
    mCurrentKF.MakeKeyFrame_Rest();  // This populates the Candidates list, which is Shi-Tomasi thresholded.
    mFirstKF = mCurrentKF;
    std::vector<std::pair<double, CVD::ImageRef> > vCornersAndSTScores;
    // Copy candidates into a trivially sortable vector.
    // So that we can choose the image corners with max ST score
    for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCandidates.size(); i++) {
        Candidate &c = mCurrentKF.aLevels[0].vCandidates[i];
        if(!mCurrentKF.aLevels[0].im.in_image_with_border(c.irLevelPos, MiniPatch::mnHalfPatchSize)) {
            continue;
        }
        vCornersAndSTScores.push_back(std::pair<double, CVD::ImageRef>(-1.0 * c.dSTScore, c.irLevelPos)); // negative so highest score first in sorted list
    }
    sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end());  // Sort according to Shi-Tomasi score
    int nToAdd = MaxInitialTrails;
    for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++) {
        if(!mCurrentKF.aLevels[0].im.in_image_with_border(vCornersAndSTScores[i].second, MiniPatch::mnHalfPatchSize)) {
            continue;
        }
        Trail t;
        t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF.aLevels[0].im);
        t.irInitialPos = vCornersAndSTScores[i].second;
        t.irCurrentPos = t.irInitialPos;
        mlTrails.push_back(t);
        nToAdd--;
    }
    mPreviousFrameKF = mFirstKF;  // Always store the previous frame so married-matching can work.
}

// Steady-state trail tracking: Advance from the previous frame, remove duds.
int StereoInit::TrailTracking_Advance(KeyFrame &mCurrentKF) {
    int nGoodTrails = 0;
    MiniPatch BackwardsPatch;
    Level &lCurrentFrame = mCurrentKF.aLevels[0];
    Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

    for(std::list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();) {
        std::list<Trail>::iterator next = i; next++;
        Trail &trail = *i;
        CVD::ImageRef irStart = trail.irCurrentPos;
        CVD::ImageRef irEnd = irStart;
        bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
        if(bFound) {
            // Also find backwards in a married-matches check
            BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
            CVD::ImageRef irBackWardsFound = irEnd;
            bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
            if((irBackWardsFound - irStart).mag_squared() > 2) {
                bFound = false;
            }

            trail.irCurrentPos = irEnd;
            nGoodTrails++;
        } else { // Erase from list of trails if not found this frame.
            mlTrails.erase(i);
        }
        i = next;
    }
    mPreviousFrameKF = mCurrentKF;
    return nGoodTrails;
}

#include "StereoInit.h"
#include "LevelHelpers.h"
#include "MapPoint.h"
#include "Log.h"

StereoInit::StereoInit(MapMaker& mm, ATANCamera &mc, Map &map): mMapMaker(mm), mCamera(mc), mMap(map), mnInitialStage(TRAIL_TRACKING_NOT_STARTED) {
    mOldKF.dSceneDepthMedian = 1;
    mOldKF.se3CfromW = SE3<>::exp(TooN::makeVector(0,0,1,M_PI,0,0));
}

void StereoInit::Reset() {
    mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
    mlTrails.clear();
}

int StereoInit::process(KeyFrame& mCurrentKF, SE3<>& mse3CamFromWorld, bool& mbUserPressedSpacebar) {
    int level = 1;
    //const ptam::PtamParamsConfig& pPars = PtamParameters::varparams();
    bool AutoInit = false;
    if (AutoInit)
    {
        if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED)
            mbUserPressedSpacebar=true;
        else if(mnInitialStage == TRAIL_TRACKING_STARTED)
        {
            // calc rotation aid of SBI
            SmallBlurryImage* pSBIsecond = new SmallBlurryImage(mCurrentKF);
            Vector<3> rotvec = CalcSBIRotation(mFirstKF.pSBI, pSBIsecond);
            SO3<> rotmat = SO3<>::exp(rotvec);
            std::vector<double> mediandistvec;

            unsigned short k=0;
            std::vector<double> medianvec;
            medianvec.resize(mlTrails.size());
            for(std::list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();++i)
            {
                Trail &trail = *i;
                // compensate pixel disparity with rotation!!
                Vector<2> firstvec = LevelZeroPos(trail.irInitialPos, level);
                Vector<2> z1vec =  mCamera.UnProject(LevelZeroPos(trail.irCurrentPos, level));
                Vector<3> dirvec = makeVector(z1vec[0],z1vec[1],1);
                dirvec = rotmat.inverse()*dirvec;
                z1vec = mCamera.Project(makeVector(dirvec[0]/dirvec[2],dirvec[1]/dirvec[2]));
                mediandistvec.push_back(sqrt((z1vec[0]-firstvec[0])*(z1vec[0]-firstvec[0])+(z1vec[1]-firstvec[1])*(z1vec[1]-firstvec[1])));
                ++k;
            }
            delete pSBIsecond;

            if(k>0)	// otherwise "if (mediandist>pPars.AutoInitPixel)" segfaults...
            {
                std::vector<double>::iterator first = mediandistvec.begin();
                std::vector<double>::iterator last = mediandistvec.end();
                std::vector<double>::iterator middle = first + std::floor((last - first) / 2);
                std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
                double mediandist = *middle;
                if (mediandist > 20)  //TODO: 20 - AutoInitPixel, should be extracted as parameters
                    mbUserPressedSpacebar=true;
            }
        }
    }

    return TrackForInitialMap(mCurrentKF, mse3CamFromWorld, mbUserPressedSpacebar);
}

//Weiss{ returns a rodriguez vector of an estimated rotation between 2 SBIs
Vector<3> StereoInit::CalcSBIRotation(SmallBlurryImage *SBI1, SmallBlurryImage *SBI2)
{
    SBI1->MakeJacs();
    std::pair<SE2<>, double> result_pair;
    result_pair = SBI2->IteratePosRelToTarget(*SBI1, 6);
    SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
    return se3Adjust.ln().slice<3,3>();
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



    //Implemetation of the AutoInit mode for stereo initialization
//    //const ptam::PtamParamsConfig& pPars = PtamParameters::varparams();
//    int gvnMaxSSD = 100000; //TODO: pPars.MiniPatchMaxSSD, should be extracted as parameter
//    //static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
//    int level = 1; //TODO 1 - InitLevel should be extracted as parameter
//    CVD::ImageRef tmpInit, tmpCurr;
//    MiniPatch::mnMaxSSD = gvnMaxSSD;

//    // What stage of initial tracking are we at?
//    if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED)
//    {
//        if(mbUserPressedSpacebar)  // First spacebar = this is the first keyframe
//        {
//            mbUserPressedSpacebar = false;
//            TrailTracking_Start(mCurrentKF);
//            mnInitialStage = TRAIL_TRACKING_STARTED;
//        }
//        else
//            LOG("Point camera at planar scene and press spacebar to start tracking for initial map.");
//        return 0;
//    };

//    if(mnInitialStage == TRAIL_TRACKING_STARTED)
//    {
//        int nGoodTrails = TrailTracking_Advance(mCurrentKF);  // This call actually tracks the trails
//        if(nGoodTrails < 10) // if most trails have been wiped out, no point continuing.
//        {
//            Reset();
//            return 1;
//        }

//        // If the user pressed spacebar here, use trails to run stereo and make the intial map..
//        if(mbUserPressedSpacebar)
//        {
//            mbUserPressedSpacebar = false;
//            std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
//            for(std::list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++){
//                tmpInit.x = LevelZeroPos(i->irInitialPos.x, level);
//                tmpInit.y = LevelZeroPos(i->irInitialPos.y, level);
//                tmpCurr.x = LevelZeroPos(i->irCurrentPos.x, level);
//                tmpCurr.y = LevelZeroPos(i->irCurrentPos.y, level);

//                vMatches.push_back(std::pair<CVD::ImageRef, CVD::ImageRef>(tmpInit, tmpCurr));
//            }

//            bool initret=false;
//            initret=mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);  // This will take some time!

//            bool mAutoreset = true;
//            if(initret)
//            {
//                if (mAutoreset)
//                {
//                    std::vector<double> medianvec1, medianvec2;


//                    for(unsigned int ip=0; ip<mMap.vpPoints.size(); ip++)
//                    {
//                        //						if(mMap.vpPoints[ip]->pPatchSourceKF==mMap.vpKeyFrames[0])
//                        medianvec1.push_back((mMap.vpKeyFrames[0]->se3CfromW*mMap.vpPoints[ip]->v3WorldPos)[2]);
//                        //						else //if(mMap.vpPoints[ip]->pPatchSourceKF==mMap.vpKeyFrames[1])
//                        medianvec2.push_back((mMap.vpKeyFrames[1]->se3CfromW*mMap.vpPoints[ip]->v3WorldPos)[2]);
//                    }
//                    std::vector<double>::iterator first = medianvec1.begin();
//                    std::vector<double>::iterator last = medianvec1.end();
//                    std::vector<double>::iterator middle = first + std::floor((last - first) / 2);
//                    std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
//                    mMap.vpKeyFrames[0]->dSceneDepthMedian = *middle;

//                    first = medianvec2.begin();
//                    last = medianvec2.end();
//                    middle = first + std::floor((last - first) / 2);
//                    std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
//                    mMap.vpKeyFrames[1]->dSceneDepthMedian = *middle;

//                    double scale = mOldKF.dSceneDepthMedian / mMap.vpKeyFrames[0]->dSceneDepthMedian;
//                    mMapMaker.ApplyGlobalScaleToMap(scale);
//                    SE3<> transf = mOldKF.se3CfromW.inverse()*mMap.vpKeyFrames[0]->se3CfromW;
//                    mMapMaker.ApplyGlobalTransformationToMap(transf);
//                    mse3CamFromWorld.get_translation() = mse3CamFromWorld.get_translation()*scale;
//                    mse3CamFromWorld = mse3CamFromWorld*transf.inverse();

//                    mAutoreset=false;
//                }
//                mnInitialStage = TRAIL_TRACKING_COMPLETE;

//                LOG("Initialized map on level %d", level);

//            }
//            else
//            {
//                Reset();
//                return 1;
//            }
//            //}
//        }
//        else
//            LOG("Translate the camera slowly sideways, and press spacebar again to perform stereo init.");
//    }

//    return 0;
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

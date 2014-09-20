#ifndef GLOBAL_VARS
#define GLOBAL_VARS
#include "image.h"
#include "TooN.h"

#define NUMTRACKERCAMPARAMETERS 5
const TooN::Vector<NUMTRACKERCAMPARAMETERS> CameraParameters;

const double MapMakerMaxKFDistWiggleMult =2;
const int MapMakerPlaneAlignerRansacs = 100;
const double Reloc2MaxScore = 9e6;
const int TrackerDrawFASTCorners =0;
const int MaxInitialTrails = 100;
const int BundleMaxIterations = 20;
const double BundleUpdateSquaredConvergenceLimit = 1e-06;
const int BundleCout = 240;

const double MapMakerWiggleScale = 0.1;
const std::string BundleMEstimator = "Tukey";
const double BundleMinTukeySigma = 0.4;

const double TrackerRotationEstimatorBlur = 0.75;
const int TrackerUseRotationEstimator = 1;
const int TrackerMiniPatchMaxSSD = 100000;
const int TrackerCoarseMin = 10; // Min number of large-scale features for coarse stage
const int TrackerCoarseMax = 60;    // Max number of large-scale features for coarse stage
const int TrackerCoarseRange = 30;  // Pixel search radius for coarse features
const int TrackerCoarseSubPixIts = 8;   // Max sub-pixel iterations for coarse features

const int TrackerDisableCoarse = 0; // Set this to 1 to disable coarse stage (except after recovery)
const double TrackerCoarseMinVelocity = 0.006;  // Speed above which coarse stage is used.
const int TrackerMaxPatchesPerFrame = 1000;
const std::string TrackerMEstimator = "Tukey";
const double TrackerTrackingQualityGood = 0.3;
const double TrackerTrackingQualityLost = 0.13;
const double MapMakerCandidateMinShiTomasiScore = 70;
const int nMaxSSDPerPixel  = 500;

#endif

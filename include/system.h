#ifndef SFO_SYSTEM_H
#define SFO_SYSTEM_H

#include <thread>
#include "libviso2/matrix.h"
#include "libviso2/viso_stereo.h"
#include "oxts.h"

#include "orbExtractor.h"
#include "loopDetector.h"

#include "DBoW2/include/TemplatedVocabulary.h"
#include "DBoW2/include/FClass.h"

namespace SFO {
    class System {
    public:

        System(const std::string &strSettingsFile, const std::string &strVocabularyFile);
        ~System();
        void trackStereo(const cv::Mat &imLeft, const cv::Mat &imRight);
        void shutdown();

    private:
        void loadSettings(const std::string &strSettingsFile);

        cv::Size mImgSize;

        unsigned int mnFrame;

        libviso2::Matrix mPose;

        std::vector<libviso2::Matrix> *mpvPoses;

        std::vector<int32_t> mvInliers;
        std::vector<libviso2::Matcher::p_match> mvMatches;

        std::int32_t mDims[3];

        libviso2::VisualOdometryStereo::parameters mParam;
        GtsamTracker *mpGtsamTracker;
        libviso2::VisualOdometryStereo *mpTracker;
        FrameDrawer *mpFrameDrawer;
        std::thread mtFrameDrawer;
        MapDrawer *mpMapDrawer;
        std::thread mtMapDrawer;

        float mFps, mT;

        LoopDetector *mpLoopDetector;
    };

} // namespace SFO

#endif //SFO_SYSTEM_H

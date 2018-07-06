#ifndef SFO_SYSTEM_H
#define SFO_SYSTEM_H

namespace SFO {
    class System {
    public:

        System(const std::string &strSettingsFile);
        System(const std::string &strSettingsFile, const std::vector<libviso2::Matrix> &vGtPoses);
        ~System();
        void trackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);
        void shutdown();

    private:
        void loadSettings(const std::string &strSettingsFile);

        cv::Size mImgSize;

        int mFrame;

        libviso2::Matrix mPose;

        std::vector<libviso2::Matrix> *mpvPoses;;

        std::vector<int32_t> mvInliers;
        std::vector<libviso2::Matcher::p_match> mvMatches;

        std::int32_t mDims[3];

        libviso2::VisualOdometryStereo::parameters mParam;
        GtsamTracker *mpGtsamTracker;
        libviso2::VisualOdometryStereo *mpTracker;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;
        std::thread mtMapDrawer;

        float mFps, mT;
    };

} // namespace SFO

#endif //SFO_SYSTEM_H

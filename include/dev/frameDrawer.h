#ifndef SFO_FRAMEDRAWER_H
#define SFO_FRAMEDRAWER_H


namespace SFO {
    class FrameDrawer {
    public:
        FrameDrawer();

        void update();

        cv::Mat drawFrame();

    protected:
        cv::Mat mIm;

    };
}




















#endif //SFO_FRAMEDRAWER_H

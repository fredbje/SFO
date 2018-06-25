#ifndef SFO_VIEWER_H
#define SFO_VIEWER_H

#include<drawer.h>


#include <mutex>

namespace SFO {
    class Viewer {
    public:
        Viewer(Drawer *pDrawer, const std::string &strSettingsPath);

        ~Viewer();

        void run();

        void requestFinish();

        void requestStop();

        bool isFinished();

        bool isStopped();

        void release();

    private:
        bool stop();
        bool checkFinishRequest();
        void setFinish();

        Drawer *mpDrawer;

        double mT;

        float mImageWidth, mImageHeight;
        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool mbFinished, mbFinishRequested, mbStopped, mbStopRequested;

        std::mutex mMutexFinish, mMutexStop;
    };
}



#endif //SFO_VIEWER_H

#include<mutex>
#include<thread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/persistence.hpp>


#include "viewer.h"


namespace SFO {
    Viewer::Viewer(Drawer *pDrawer, const std::string &strSettingPath) : mpDrawer(pDrawer) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        fps = (fps < 1) ? 30 : fps;
        mT = 1e3/fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];

        if(mImageWidth < 1 || mImageHeight < 1) {
            std::cerr << "Image size in settings is not valid!" << std::endl;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
    }

    Viewer::~Viewer() { }

    void Viewer::run() {
        mbFinished = false;
        mbStopped = false;

        pangolin::CreateWindowAndBind("SFO: Map Viewer", 1024, 768);

        // 3D mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGL we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);

        // Define camera render object (for view/scene browsing)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        cv::namedWindow("SFO: Current Frame");

        bool bFollow = true;

        while(1) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpDrawer->getCurrentOpenGLCameraMatrix(Twc);

            if(menuFollowCamera && bFollow) {
                s_cam.Follow(Twc);
            } else if (menuFollowCamera && !bFollow) {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            } else if(!menuFollowCamera && bFollow) {
                bFollow = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Draw white screen
            mpDrawer->drawCurrentCamera(Twc);

            pangolin::FinishFrame();

            cv::Mat im = mpDrawer->drawFrame();
            cv::imshow("SFO: Current Frame", im);
            cv::waitKey(mT);

            if(stop()) {
                while(isStopped()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(3));
                }
            }

            if(checkFinishRequest()) {
                break;
            }
        }

        setFinish();
    }

    void Viewer::requestFinish() {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::checkFinishRequest() {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::setFinish() {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished() {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::requestStop() {
        std::unique_lock<std::mutex> lock(mMutexStop);
        if(!mbStopped) {
            mbStopRequested = true;
        }
    }

    bool Viewer::isStopped() {
        std::unique_lock<std::mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::stop() {
        std::unique_lock<std::mutex> lock1(mMutexStop);
        std::unique_lock<std::mutex> lock2(mMutexFinish);

        if(mbFinishRequested) {
            return false;
        } else if(mbStopRequested) {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }
        return false;
    }

    void Viewer::release() {
        std::unique_lock<std::mutex> lock(mMutexStop);
        mbStopped = false;
    }
}
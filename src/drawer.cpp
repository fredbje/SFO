#include "drawer.h"
#include <opencv2/core/persistence.hpp>
//#include <libviso2/matrix.h>

namespace SFO {

    Drawer::Drawer(const std::string &strSettingsFile) {
        cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];

        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

        fSettings.release();
    }

    Drawer::Drawer(const std::string &strSettingsFile, const std::vector<libviso2::Matrix> &vGtPoses)
            : Drawer(strSettingsFile) {
        mvGtPoses = vGtPoses; // Copy content, not reference
    }

    Drawer::~Drawer() = default;


    void Drawer::updateGtsamPoses(const std::vector<libviso2::Matrix> &vGtsamPoses) {
        // TODO: Lock the variable
        std::unique_lock<std::mutex> lock(mMutexUpdateGtsam);
        mvGtsamPoses.insert(mvGtsamPoses.end(), vGtsamPoses.begin() + mvGtsamPoses.size(), vGtsamPoses.end());
    }


    void Drawer::updateLibviso2Poses(const std::vector<libviso2::Matrix> &vLibviso2Poses) {
        std::unique_lock<std::mutex> lock(mMutexUpdateLibviso2);
        mvLibviso2Poses.insert(mvLibviso2Poses.end(), vLibviso2Poses.begin() + mvLibviso2Poses.size(), vLibviso2Poses.end());
    }



    void Drawer::drawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
        float CameraSize = 0.5;
        float mCameraLineWidth = 1;
        const float &w = CameraSize;
        const auto h = (float)(w*0.75);
        const auto z = (float)(w*0.6);

        glPushMatrix();
        glMultMatrixd(Twc.m);

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }

    void Drawer::drawCurrentCameraRed(pangolin::OpenGlMatrix &Twc) {
        float CameraSize = 0.5;
        float mCameraLineWidth = 1;
        const float &w = CameraSize;
        const auto h = (float)(w*0.75);
        const auto z = (float)(w*0.6);

        glPushMatrix();
        glMultMatrixd(Twc.m);

        glLineWidth(mCameraLineWidth);
        glColor3f(1.0f,0.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }

    void Drawer::drawCurrentCameraBlue(pangolin::OpenGlMatrix &Twc) {
        float CameraSize = 0.5;
        float mCameraLineWidth = 1;
        const float &w = CameraSize;
        const auto h = (float)(w*0.75);
        const auto z = (float)(w*0.6);

        glPushMatrix();
        glMultMatrixd(Twc.m);

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }

    pangolin::OpenGlMatrix Drawer::getOpenGlMatrix(libviso2::Matrix pose) {
        pangolin::OpenGlMatrix Twc;

        Twc.m[ 0] = pose.val[0][0];
        Twc.m[ 1] = pose.val[1][0];
        Twc.m[ 2] = pose.val[2][0];
        Twc.m[ 3] = pose.val[3][0];

        Twc.m[ 4] = pose.val[0][1];
        Twc.m[ 5] = pose.val[1][1];
        Twc.m[ 6] = pose.val[2][1];
        Twc.m[ 7] = pose.val[3][1];

        Twc.m[ 8] = pose.val[0][2];
        Twc.m[ 9] = pose.val[1][2];
        Twc.m[10] = pose.val[2][2];
        Twc.m[11] = pose.val[3][2];

        Twc.m[12] = pose.val[0][3];
        Twc.m[13] = pose.val[1][3];
        Twc.m[14] = pose.val[2][3];
        Twc.m[15] = pose.val[3][3];
        
        return Twc;
    }


    void Drawer::requestFinish() {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Drawer::checkFinish() {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Drawer::start() {

        pangolin::CreateWindowAndBind("Stereo Visual Odometry: Pose Viewer", 1024, 768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        //glEnable (GL_BLEND);
        //glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        //pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        //pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
        //pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        //pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
        //pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
        //pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
        //pangolin::Var<bool> menuReset("menu.Reset",false,false);

        // Define Camera Render Object (for view / scene browsing)
        /*
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
        );
        */
        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 320, 240, 0.1, 1000),
                pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 1.0, 0.0, 0.0)
        );




        // Add named OpenGL viewport to window and provide 3D Handler
        auto *handler3D = new pangolin::Handler3D(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(handler3D);

        // Matrix that changes where the camera is
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        //bool bFollow = true;

        while(true) {

            // Set boot interrupt point
            boost::this_thread::interruption_point();

            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


            // Skip if there are no poses
            if(!mvGtsamPoses.empty()) {
                continue;
            }

            // Get the current transformation
            //Twc = getOpenGlMatrix(manager->get_state());
            Twc = getOpenGlMatrix(mvLibviso2Poses.at(mvLibviso2Poses.size()-1));
            drawCurrentCameraBlue(Twc);

            /*
            if(menuFollowCamera && bFollow) {
                s_cam.Follow(Twc);
            } else if(menuFollowCamera && !bFollow) {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            } else if(!menuFollowCamera && bFollow) {
                bFollow = false;
            }
            */

            // Update the view camera
            //s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ,0,0,0,pangolin::AxisZ));
            //s_cam.Follow(Twc);
            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            // The two poses we are drawing lines between
            pangolin::OpenGlMatrix Tw1;
            pangolin::OpenGlMatrix Tw2;
            pangolin::OpenGlMatrix Twtruth;

            //Twtruth = getOpenGlMatrix(mptrPoseTruth->at(mptrPose->size()-1));
            //DrawCurrentCameraBlue(Twtruth);

            // Draw current frame
            for(std::size_t i = 1; i < mvGtsamPoses.size(); i++) {
                // Get new pose
                Tw1 = getOpenGlMatrix(mvGtsamPoses.at(i-1));
                Tw2 = getOpenGlMatrix(mvGtsamPoses.at(i));
                // Draw the camera that is not drawn yet
                drawCurrentCamera(Tw2);

                Twtruth = getOpenGlMatrix(mvGtPoses.at(i));
                drawCurrentCameraRed(Twtruth);

                Twc = getOpenGlMatrix(mvLibviso2Poses.at(i));
                drawCurrentCameraBlue(Twc);
                // Draw line between the two
                float mLineSize = 3;
                glLineWidth(mLineSize);
                glColor3f(1.0, 1.0, 0.0);
                glBegin(GL_LINES);
                glVertex3f((float) Tw1.m[12], (float) Tw1.m[13], (float) Tw1.m[14]);
                glVertex3f((float) Tw2.m[12], (float) Tw2.m[13], (float) Tw2.m[14]);
                glEnd();
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();

            if(checkFinish()) {
                break;
            }
        }
        delete handler3D;
        std::cout << "Deleted handler3D" << std::endl;
    }
}





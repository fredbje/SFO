#include "drawer.h"
//#include <libviso2/matrix.h>

namespace SFO {

    Drawer::Drawer() {
        mpvGtsamPoses = new std::vector<libviso2::Matrix>();
        mpvLibviso2Poses = new std::vector<libviso2::Matrix>();
        mpvGtPoses = new std::vector<libviso2::Matrix>();
    }


    Drawer::Drawer(const std::vector<libviso2::Matrix> &vGtPoses) : Drawer() {
        *mpvGtPoses = vGtPoses;
    }

    Drawer::~Drawer() {
        delete mpvGtsamPoses;
        delete mpvLibviso2Poses;
        delete mpvGtPoses;
    }

    void Drawer::updateGtsamPoses(std::vector<libviso2::Matrix> *pvGtsamPoses) {
        // TODO: Lock the variable
        mpvGtsamPoses->insert(mpvGtsamPoses->end(), pvGtsamPoses->begin() + mpvGtsamPoses->size(), pvGtsamPoses->end());
    }

    void Drawer::updateLibviso2Poses(std::vector<libviso2::Matrix> *pvLibviso2Poses) {
        mpvLibviso2Poses->insert(mpvLibviso2Poses->end(), pvLibviso2Poses->begin() + mpvLibviso2Poses->size(), pvLibviso2Poses->end());
    }
    
    void Drawer::drawCamera(pangolin::OpenGlMatrix &Twc, Color color) {
        float CameraSize = 0.5;
        float mCameraLineWidth = 1;
        const float &w = CameraSize;
        const auto h = (float)(w*0.75);
        const auto z = (float)(w*0.6);

        glPushMatrix();
        glMultMatrixd(Twc.m);

        glLineWidth(mCameraLineWidth);
        if(color == red) {
            glColor3f(1.0f, 0.0f, 0.0f);
        } else if(color == green) {
            glColor3f(0.0f, 1.0f, 0.0f);
        }else if (color == blue) {
            glColor3f(0.0f, 0.0f, 1.0f);
        }

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

    // TODO Make it possible to option out viewing different poses and graphs.
    void Drawer::start() {
        double mViewpointX = 10.0;
        double mViewpointY = 10.0;
        double mViewpointZ = 100.0;
        double mViewpointF = 400;

        pangolin::CreateWindowAndBind("Pose Viewer", 1024, 768);
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        auto *handler3D = new pangolin::Handler3D(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(handler3D);

        // Matrix that changes where the camera is
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        bool bFollow = true;

        // TODO Fix so that viewer stops when pressing ESC.
        while(true /*!pangolin::ShouldQuit()*/) {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Skip if there are no poses
            if(mpvGtsamPoses->empty()) {
                continue;
            }

            Twc = getOpenGlMatrix(mpvGtsamPoses->back());
            if(menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if(menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            // The two poses we are drawing lines between
            pangolin::OpenGlMatrix Tw1Gtsam;
            pangolin::OpenGlMatrix Tw2Gtsam;
            pangolin::OpenGlMatrix Tw1Libviso2;
            pangolin::OpenGlMatrix Tw2Libviso2;
            pangolin::OpenGlMatrix Tw1Gt;
            pangolin::OpenGlMatrix Tw2Gt;

            // Draw current frame
            for(std::size_t i = 1; i < mpvGtsamPoses->size(); i++) {
                Tw1Gtsam    = getOpenGlMatrix(mpvGtsamPoses->at(i-1));
                Tw2Gtsam    = getOpenGlMatrix(mpvGtsamPoses->at(i));
                Tw1Libviso2 = getOpenGlMatrix(mpvLibviso2Poses->at(i-1));
                Tw2Libviso2 = getOpenGlMatrix(mpvLibviso2Poses->at(i));
                Tw1Gt       = getOpenGlMatrix(mpvGtPoses->at(i-1));
                Tw2Gt       = getOpenGlMatrix(mpvGtPoses->at(i));

                if(i == 1) {
                    drawCamera(Tw1Gtsam, green);
                    drawCamera(Tw1Libviso2, blue);
                    drawCamera(Tw2Gt, red);
                }

                drawCamera(Tw2Gtsam, green);
                drawCamera(Tw2Libviso2, blue);
                drawCamera(Tw2Gt, red);

                drawLines(Tw1Gtsam, Tw2Gtsam, green);
                drawLines(Tw1Libviso2, Tw2Libviso2, blue);
                drawLines(Tw1Gt, Tw2Gt, red);
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();

            if(checkFinish()) {
                break;
            }
        }
        pangolin::DestroyWindow("Pose Viewer");
        delete handler3D;
        std::cout << "Deleted handler 3D" << std::endl;
    }
    
    void Drawer::drawLines(pangolin::OpenGlMatrix T1, pangolin::OpenGlMatrix T2, Color color) {
        float mLineSize = 3;
        glLineWidth(mLineSize);
        if(color == red) {
            glColor3f(1.0f, 0.0f, 0.0f);
        } else if(color == green) {
            glColor3f(0.0f, 1.0f, 0.0f);
        }else if (color == blue) {
            glColor3f(0.0f, 0.0f, 1.0f);
        }
        glBegin(GL_LINES);
        glVertex3f((float) T1.m[12], (float) T1.m[13], (float) T1.m[14]);
        glVertex3f((float) T2.m[12], (float) T2.m[13], (float) T2.m[14]);
        glEnd();
    }
} // namespace SFO





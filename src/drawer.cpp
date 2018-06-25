#include <opencv2/core/persistence.hpp> // cv::FileStorage
#include <opencv/cv.hpp>
#include <libviso2/matcher.h>

#include "drawer.h"

namespace SFO {

    Drawer::Drawer(libviso2::VisualOdometryStereo *pViso, const std::string &strSettingPath) : mpViso(pViso) {

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

        // Set default values
        this->mpvPoseGtsam = new std::vector<libviso2::Matrix>();
        this->mpvPoseLibviso2 = new std::vector<libviso2::Matrix>();
        this->mpvPoseGT = new std::vector<libviso2::Matrix>();
        //this->mPose = vptrPose->at(0);

        // Start the render thread
        //boost::thread t(boost::bind(&drawer::start, this));
    }

/*
    Drawer::Drawer(std::string gtPosesFileName) : Drawer() {
        libviso2::Matrix tempPose(4,4);
        double r11,r12,r13,r21,r22,r23,r31,r32,r33;
        double t1,t2,t3;

        std::ifstream gtPosesFile;
        std::string temp_state;
        
        gtPosesFile.open(gtPosesFileName.c_str(), std::ifstream::in);
        if(!gtPosesFile.is_open()){
            std::cerr << "Error, could not open ground truth pose file!" << std::endl;
        }

        std::cout << "Loading ground truth poses.." << std::endl;
        while(!gtPosesFile.eof()) {
            // Read in next line
            std::getline(gtPosesFile, temp_state);
            std::sscanf(temp_state.c_str(), "%le %le %le %le %le %le %le %le %le %le %le %le",
                   &r11,&r12,&r13,&t1,&r21,&r22,&r23,&t2,&r31,&r32,&r33,&t3);

            if(temp_state.empty()){break;}

            // Create clone state
            tempPose.val[0][0] = r11;
            tempPose.val[0][1] = r12;
            tempPose.val[0][2] = r13;
            tempPose.val[0][3] = t1;
            tempPose.val[1][0] = r21;
            tempPose.val[1][1] = r22;
            tempPose.val[1][2] = r23;
            tempPose.val[1][3] = t2;
            tempPose.val[2][0] = r31;
            tempPose.val[2][1] = r32;
            tempPose.val[2][2] = r33;
            tempPose.val[2][3] = t3;
            tempPose.val[3][0] = 0;
            tempPose.val[3][1] = 0;
            tempPose.val[3][2] = 0;
            tempPose.val[3][3] = 1;
            mptrPoseTruth->push_back(tempPose);
        }
        std::cout << "Finished loading GT poses. Number of poses loaded: " << mptrPoseTruth->size() << std::endl;
    }
    */

    Drawer::~Drawer() {
        //delete this->mptrPose;
        //delete this->mptrPoselib;
        //delete this->mptrPoseTruth;
    }

    void Drawer::setCurrentCameraPose(const cv::Mat &Tcw) {
        std::unique_lock<std::mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    cv::Mat Drawer::drawFrame(){
        // Vector of structs p_match for storing matches.
        std::vector<libviso2::Matcher::p_match> vMatches = mpViso->getMatches();
        std::vector<int32_t> vInliers = mpViso->getInlierIndices();

        // output some statistics
        double nMatches = mpViso->getNumberOfMatches();
        double nInliers = mpViso->getNumberOfInliers();
        std::cout << ", Matches: " << std::fixed << std::setprecision(0) << nMatches;
        std::cout << ", Inliers: " << std::fixed << std::setprecision(2)
                  << 100.0 * nInliers / nMatches << " %" << std::endl;

        // Create combined matrix
        cv::Mat im3(2*mImLeft.size().height + 5, mImLeft.size().width, CV_8UC1);
        cv::Mat left(im3, cv::Rect(0, 0, mImLeft.size().width, mImLeft.size().height));
        mImLeft.copyTo(left);
        cv::Mat right(im3, cv::Rect(0, mImRight.size().height + 5, mImRight.size().width, mImRight.size().height));
        mImRight.copyTo(right);

        // Convert to color type
        cv::cvtColor(im3, im3, CV_GRAY2RGB);

        // ==============================
        // Add extracted features to mat
        // ==============================
        for (std::size_t ki = 0; ki < vMatches.size(); ki++) {
            // Get match data
            libviso2::Matcher::p_match match;
            match = vMatches.at(ki);
            // Point color object
            cv::Scalar color;
            // Check to see if inlier, if so color it as one
            // The outliers are red
            if (std::find(vInliers.begin(), vInliers.end(), ki) != vInliers.end()) {
                color = CV_RGB(0, 255, 255);
            } else {
                color = CV_RGB(255, 0, 0);
            }
            // Add to image
            cv::Point2f pt_left(match.u1c, match.v1c);
            cv::circle(im3, pt_left, 2, color);
            cv::Point2f pt_right(match.u2c, match.v2c + (mImLeft.size().height + 5));
            cv::circle(im3, pt_right, 2, color);
        }
        return im3;
    }


   /* void Drawer::updatePoses(std::vector<libviso2::Matrix>* gtsamPoseVec) {

        // TODO: Lock the variable

        // Debug
        //cout << "Array before: " << mptrPose->size() << endl;

        // Next copy over the new poses

        mptrPose->insert(mptrPose->end(), gtsamPoseVec->begin() + mptrPose->size(), gtsamPoseVec->end());

        // Debug
        //cout << "Array after: " << mptrPose->size() << endl;

    }
*/
    void Drawer::updatePoseslib(std::vector<libviso2::Matrix>* libviso2PoseVec) {
        mpvPoseLibviso2->insert(mpvPoseLibviso2->end(), libviso2PoseVec->begin() + mpvPoseLibviso2->size(), libviso2PoseVec->end());
    }

    void Drawer::updateImages(const cv::Mat &imLeft, const cv::Mat &imRight) {
        imLeft.copyTo(mImLeft);
        imRight.copyTo(mImRight);
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

    pangolin::OpenGlMatrix Drawer::get_matrix(libviso2::Matrix pose) {


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

    void Drawer::start() {

        pangolin::CreateWindowAndBind("Stereo Visual Odometry: Pose Viewer", 1024, 768);
        glEnable(GL_DEPTH_TEST);

        // Defind vars
        double mViewpointX = 10.0;
        double mViewpointY = 10.0;
        double mViewpointZ = 100.0;
        double mViewpointF = 400;

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 320, 240, 0.1, 1000),
                pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 1.0, 0.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        // Matrix that changes where the camera is
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        while(!pangolin::ShouldQuit()) {

            // Set boot interrupt point
            boost::this_thread::interruption_point();

            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


            // Skip if there are no poses
            if(mpvPoseLibviso2->empty()) {
                continue;
            }

            // Get the current transformation
            //Twc = get_matrix(manager->get_state());
            ////Twc = get_matrix(mptrPoselib->at(mptrPoselib->size()-1));
            ///drawCurrentCameraBlue(Twc);


            // Update the view camera
            //s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ,0,0,0,pangolin::AxisZ));
            //s_cam.Follow(Twc);
            d_cam.Activate(s_cam);

            // Make our background white
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            // The two poses we are drawing lines between
            pangolin::OpenGlMatrix Tw1;
            pangolin::OpenGlMatrix Tw2;
            pangolin::OpenGlMatrix Twtruth;

            //Twtruth = get_matrix(mptrPoseTruth->at(mptrPose->size()-1));
            //DrawCurrentCameraBlue(Twtruth);

            // Draw current frame
            for(std::size_t i = 1; i < mpvPoseLibviso2->size(); i++) {
                // Get new pose
               // Tw1 = get_matrix(mptrPose->at(i-1));
                //Tw2 = get_matrix(mptrPose->at(i));
                // Draw the camera that is not drawn yet
               // drawCurrentCamera(Tw2);

                //Twtruth = get_matrix(mptrPoseTruth->at(i));
                //drawCurrentCameraRed(Twtruth);

                Twc = get_matrix(mpvPoseLibviso2->at(i));
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
        }

    }


    void Drawer::getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
    {
        if(!mCameraPose.empty())
        {
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            {
                std::unique_lock<std::mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
                twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
            }

            M.m[0] = Rwc.at<float>(0,0);
            M.m[1] = Rwc.at<float>(1,0);
            M.m[2] = Rwc.at<float>(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Rwc.at<float>(0,1);
            M.m[5] = Rwc.at<float>(1,1);
            M.m[6] = Rwc.at<float>(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Rwc.at<float>(0,2);
            M.m[9] = Rwc.at<float>(1,2);
            M.m[10] = Rwc.at<float>(2,2);
            M.m[11]  = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15]  = 1.0;
        }
        else
            M.SetIdentity();
    }
}





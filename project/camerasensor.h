#ifndef CAMERASENSOR_H
#define CAMERASENSOR_H

#include "prerequisites.h"
#include "FreeImageHelper.h"
#include <experimental/filesystem>
#include <regex>
#include <iostream>

/**
 * @brief The CameraSensor class
 *
 * Left camera, also referred to as camera 1 is in world coordinates (reference frame).
 * The right camera, also referred to as camera 2 is translated to the right.
 * Depth values: Should be relative to the reference frame (left camera, camera 1).
 */

typedef unsigned char BYTE;

// Filesystem namespace as abbreveation
namespace fs = std::experimental::filesystem;
const std::regex IMAGE_NAME_REGEX (".*im[0-9].png");

using namespace Eigen;

class CameraSensor {
public:
    CameraSensor() {
    }

    ~CameraSensor() {
        SAFE_DELETE_ARRAY(m_leftFrame);
        SAFE_DELETE_ARRAY(m_rightFrame);
    }

    /**
     * @brief Init
     * initialize sensor, set all image properties
     * @param datasetDir : path to image-directory
     * @return false if initialization wasn't possible
     */
    bool Init(const std::string& datasetDir) {

        m_dataDir = datasetDir;
        if (!pushImageNamesToVector()) return false;

        /*
         * from here on: recycle-bin data set
         *
        m_leftIntrinsics << 2945.377f, 0.f, 1284.862f,
                0.0f, 2945.377f, 954.52f,
                0.0f, 0.0f, 1.f;
        m_leftExtrinsics.setIdentity();

        m_rightIntrinsics << 2945.377f, 0.0f, 1455.543f,
                0.0f, 2945.377f, 954.52f,
                0.0f, 0.0f, 1.0f;

        m_rightExtrinsics <<    0.7071f, 0.0f, 0.7071f, 178.232f, // translated along x axis by 178.232mm, or maybe doffs= 170.681?
                0.0f, 1.f, 0.0f, 0.0f,
                -0.7071f, 0.0f, 0.7071f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;
        */

        /*
         * from here on: blender data set
         * */

        m_focalLength = 700.0f;
        m_baseline = 100.0f;
        m_doffs = 100.0f;

        // width/height needed for homographies
        m_leftImageWidth = 640;
        m_leftImageHeight = 480;
        m_rightImageWidth = 640;
        m_rightImageHeight = 480;

        m_leftIntrinsics << 700.0f, 0.f, 320.0f,
                            0.0f, 700.0f, 240.0f,
                            0.0f, 0.0f, 1.0f;

        m_rightIntrinsics << 700.0f, 0.f, 320.0f,
                             0.0f, 700.0f, 240.0f,
                             0.0f, 0.0f, 1.0f;

        m_leftExtrinsics.setIdentity();
        m_rightExtrinsics <<    0.966f, 0.0f, 0.259f, 100.0f, // 15 degree of rotation
                                0.0f, 1.0f, 0.0f, 0.0f,
                                -0.259f, 0.0f, 0.966f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f;

        /*
         * ( cos 0 sin)
         * ( 0   1 0  )
         * (-sin 0 cos)
         *
         * more rotations:

        m_rightExtrinsics <<    1.0f, 0.0f, 0.0f, 100.0f, // 0 degree of rotation
                                0.0f, 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f;

        m_rightExtrinsics <<    0.999f, 0.0f, 0.035f, 100.0f, // 2 degree of rotation
                                0.0f, 1.0f, 0.0f, 0.0f,
                                -0.035f, 0.0f, 0.999f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f;

        m_rightExtrinsics <<    0.7071f, 0.0f, 0.7071f, 100.0f, // 45 degree of rotation
                                0.0f, 1.0f, 0.0f, 0.0f,
                                -0.7071f, 0.0f, 0.7071f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f;
        */

        // set index to start with
        m_arraySet = false;
        m_currentIdx = -1;

        // compute homographies once
        computeHomographies();

        return true;

    }

    Matrix3f hat(Vector3f v) {
        Matrix3f v_hat;
        v_hat <<    0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
        return v_hat;
    }

    bool computeHomographies() {
        Matrix3f K = m_leftIntrinsics;
        Matrix4f E = m_rightExtrinsics;

        Vector3f T = E.block<3,1>(0,3);
        Matrix3f R = E.block<3,3>(0,0);
        Matrix3f T_hat = hat(T);
        Matrix3f F = K.inverse().transpose() * T_hat * R * K.inverse();
        Vector3f e = K * R.transpose() * T;
        // TODO: Implement special case of NO rotation between frames (slides internet)

        // estimation of z
        // left frame
        Matrix3f PP;
        PP <<   (m_leftImageWidth-1)*(m_leftImageWidth-1), (m_leftImageWidth-1)*(m_leftImageHeight-1), 0,
                (m_leftImageWidth-1)*(m_leftImageHeight-1), (m_leftImageHeight-1)*(m_leftImageHeight-1), 0,
                0, 0, 1;
        PP *= 0.25f;

        Matrix3f pcpc;
        pcpc << m_leftImageWidth*m_leftImageWidth-1, 0, 0,
                0, m_leftImageHeight*m_leftImageHeight-1, 0,
                0, 0, 1;
        pcpc *= 0.25f;

        Matrix3f e_hat = hat(e);
        Matrix3f A = e_hat.transpose() * PP * e_hat;
        Matrix3f B = e_hat.transpose() * pcpc * e_hat;
        LLT<Matrix3f> cholesky_A(A);
        Matrix3f D = cholesky_A.matrixL();
        EigenSolver<Matrix3f> EV(D.inverse().transpose() * B * D.inverse());
        Vector3f y = EV.pseudoEigenvectors().col(0);
        Vector3f z = D.inverse() * y;
        z(2) = 0;

        // right frame
        Matrix3f PP_;
        PP_ <<  (m_rightImageWidth-1)*(m_rightImageWidth-1), (m_rightImageWidth-1)*(m_rightImageHeight-1), 0,
                (m_rightImageWidth-1)*(m_rightImageHeight-1), (m_rightImageHeight-1)*(m_rightImageHeight-1), 0,
                0, 0, 1;
        PP_ *= 0.25f;

        Matrix3f pcpc_;
        pcpc_ <<    m_rightImageWidth*m_rightImageWidth-1, 0, 0,
                0, m_rightImageHeight*m_rightImageHeight-1, 0,
                0, 0, 1;
        pcpc_ *= 0.25f;

        Matrix3f A_ = F.transpose() * PP_ * F;
        Matrix3f B_ = F.transpose() * pcpc_ * F;
        LLT<Matrix3f> cholesky_A_(A_);
        Matrix3f D_ = cholesky_A_.matrixL();
        EigenSolver<Matrix3f> EV_(D_.inverse().transpose() * B_ * D_.inverse());
        Vector3f y_ = EV_.pseudoEigenvectors().col(0);
        Vector3f z_ = D_.inverse() * y_;
        z_(2) = 0;

        // average vector, not further optimized here
        // TODO: non-linear optimization of initial guess z_avg
        Vector3f z_avg = (z/z.norm() + z_/z_.norm())/2;

        Vector3f w = e_hat * z_avg;
        w = w / w.z();
        Vector3f w_ = F * z_avg;
        w_ = w_ / w_.z();

        Matrix3f Hp, Hp_;
        Hp <<   1, 0, 0,
                0, 1, 0,
                w.x(), w.y(), 1;
        Hp_ <<  1, 0, 0,
                0, 1, 0,
                w_.x(), w_.y(), 1;

        Vector3f v_ = Vector3f::Zero();
        Matrix3f Hr, Hr_;
        Hr <<   F(2,1) - w.y()*F(2,2), w.x()*F(2,2) - F(2,0), 0,
                F(2,0) - w.x()*F(2,2), F(2,1) - w.y()*F(2,2), F(2,2) + v_.z(),
                0, 0, 1;
        // The abs() of the Hr_p(0,0) and Hr_p(1,1) are necessary to rotate in the right direction. Should be OK, since this is a rotation matrix and cos is axis-symmetric.
        Hr_ <<  abs(F(1,2) - w_.y()*F(2,2)), w_.x()*F(2,2) - F(0,2), 0,
                F(0,2) - w_.x()*F(2,2), abs(F(1,2) - w_.y()*F(2,2)), v_.z(),
                0, 0, 1;
        /*
        Hr_ <<  F(1,2) - w_.y()*F(2,2), w_.x()*F(2,2) - F(0,2), 0,
                F(0,2) - w_.x()*F(2,2), F(1,2) - w_.y()*F(2,2), v_.z(),
                0, 0, 1;
        */

        m_H << Hr * Hp;
        m_H_ << Hr_ * Hp_;
        m_S = determineScaleMatrix();
    }

    /**
     * @brief determineScaleMatrix - determine an approximate scale factor by comparing transformed image areas and determine their scale factor compared to the original image.
     * @return
     */
    Matrix3f determineScaleMatrix() {
        float imageArea = (m_leftImageWidth * m_leftImageHeight + m_rightImageWidth * m_rightImageHeight) / 2;

        // left image approx area
        Vector3f lowerLeftCorner = m_H * Vector3f(0, 0, 0);
        Vector3f lowerRightCorner = m_H * Vector3f(0, m_leftImageWidth, 0);
        Vector3f upperLeftCorner = m_H * Vector3f(m_leftImageHeight, 0, 0);

        Vector3f firstEdge = lowerRightCorner - lowerLeftCorner;
        Vector3f secondEdge = upperLeftCorner - lowerLeftCorner;
        float approxArea = firstEdge(1) * secondEdge(0);

        // right image approx area
        Vector3f lowerLeftCorner_ = m_H_ * Vector3f(0, 0, 0);
        Vector3f lowerRightCorner_ = m_H_ * Vector3f(0, m_rightImageWidth, 0);
        Vector3f upperLeftCorner_ = m_H_ * Vector3f(m_rightImageHeight, 0, 0);

        Vector3f firstEdge_ = lowerRightCorner_ - lowerLeftCorner_;
        Vector3f secondEdge_ = upperLeftCorner_ - lowerLeftCorner_;
        float approxArea_ = firstEdge_(1) * secondEdge_(0);

        // determine the average area
        float approxAreaAvg = (approxArea + approxArea_) / 2;
        float scale = std::sqrt(imageArea / approxAreaAvg);

        Matrix3f result;
        result << scale, 0, 0, 0, scale, 0, 0, 0, 1;
        return result;
    }

    bool setArrays(int l_w, int l_h, int r_w, int r_h) {
        // create arrays and read in dummy values
        if (m_arraySet) {
            SAFE_DELETE_ARRAY(m_leftFrame);
            SAFE_DELETE_ARRAY(m_rightFrame);
        }

        m_arraySet = true;

        m_leftImageWidth = l_w;
        m_leftImageHeight = l_h;

        m_rightImageWidth = r_w;
        m_rightImageHeight = r_h;

        // TODO
        // we also need to update the intrinsics then!
        // we probably also have to adapt the focal length in the intrinsics.. ???


        m_leftFrame = new BYTE[4 * m_leftImageWidth * m_leftImageHeight];
        for (unsigned int i = 0; i < 4 * m_leftImageWidth * m_leftImageHeight; i++) m_leftFrame[i] = 255;
        m_rightFrame = new BYTE[4 * m_rightImageWidth * m_rightImageHeight];
        for (unsigned int i = 0; i < 4 * m_rightImageWidth * m_rightImageHeight; i++) m_rightFrame[i] = 255;
    }

    /**
     * @brief pushImageNamesToVector
     * create vector of two image name pairs, traverses filesystem and compares image names to regex
     * @return false if no images found, else true
     */
    bool pushImageNamesToVector() {
        bool pair = false;
        std::string temp_imageName;
        int samples = 0;

        // traverse filesystem at given directory path
        for(auto & file : fs::directory_iterator(m_dataDir)) {

            std::string imageName = file.path();

            /// Uncomment for file system debugging
            // std::cout << "File name:\t" << imageName << std::endl;

            // Check for correct image names - compare to const REGEX at file start
            if(std::regex_match(imageName, IMAGE_NAME_REGEX)) {

                // Helper to group every 2 consecutive images to pairs
                // --> in case of better database should be able to make this better
                if(pair) {
                    m_imagePairNameVector.push_back(std::pair<std::string, std::string>(imageName, temp_imageName));
                    pair = !pair;
                    samples++;
                }
                else {
                    temp_imageName = imageName;
                    pair = !pair;
                }
            }
        }

        return samples > 0;
    }

    /**
     * @brief ProcessNextFrame
     * called every step, reads in image of next filename to BYTE array
     * @return false if no more frames, true if next frame possible
     */
    bool ProcessNextFrame()
    {
        m_currentIdx++;

        if ((unsigned int)m_currentIdx >= (unsigned int)m_imagePairNameVector.size()) return false;

        std::cout << "ProcessNextFrame [" << m_currentIdx << " | " << m_imagePairNameVector.size() << "]" << std::endl;

        // read in both images as BYTE arrays

        FreeImageB leftImage, rightImage;
        leftImage.LoadImageFromFile(m_imagePairNameVector[m_currentIdx].first);
        rightImage.LoadImageFromFile(m_imagePairNameVector[m_currentIdx].second);

        // (re)initialize arrays in case needed
        if(!m_arraySet ||
                leftImage.w != m_leftImageWidth || leftImage.h != m_leftImageHeight ||
                rightImage.w != m_rightImageWidth || rightImage.h != m_rightImageHeight) {

            setArrays(leftImage.w, leftImage.h, rightImage.w, rightImage.h);
        }

        memcpy(m_leftFrame, leftImage.data, 4 * m_leftImageWidth * m_leftImageHeight);
        memcpy(m_rightFrame, rightImage.data, 4 * m_rightImageWidth * m_rightImageHeight);

        return true;
    }

    BYTE* getLeftFrame() {
        return m_leftFrame;
    }
    BYTE* getRightFrame() {
        return m_rightFrame;
    }
    Eigen::Matrix3f getLeftIntrinsics() {
        return m_leftIntrinsics;
    }
    Eigen::Matrix3f getRightIntrinsics() {
        return m_leftIntrinsics;
    }
    Eigen::Matrix4f getLeftExtrinsics(){
        return m_leftExtrinsics;
    }
    Eigen::Matrix4f getRightExtrinsics(){
        return m_rightExtrinsics;
    }
    unsigned int getLeftImageWidth() {
        return m_leftImageWidth;
    }
    unsigned int getLeftImageHeight() {
        return m_leftImageHeight;
    }
    unsigned int getRightImageWidth() {
        return m_leftImageWidth;
    }
    unsigned int getRightImageHeight() {
        return m_leftImageHeight;
    }
    Matrix3f getH() {
        return m_H;
    }
    Matrix3f getH_() {
        return m_H_;
    }
    Matrix3f getS() {
        return m_S;
    }
    float getFocalLength() {
        return m_focalLength;
    }
    float getBaseline() {
        return m_baseline;
    }
    float getDoffs() {
        return m_doffs;
    }

private:
    // directory
    std::string m_dataDir;
    std::vector<std::pair<std::string, std::string>> m_imagePairNameVector;

    // current frame index
    int m_currentIdx;

    bool m_arraySet;

    // color images
    BYTE* m_leftFrame;
    BYTE* m_rightFrame;

    // Properties of both cameras
    float m_focalLength;
    float m_baseline;
    float m_doffs;

    Eigen::Matrix3f m_leftIntrinsics;
    Eigen::Matrix4f m_leftExtrinsics;

    Eigen::Matrix3f m_rightIntrinsics;
    Eigen::Matrix4f m_rightExtrinsics;

    unsigned int m_leftImageWidth;
    unsigned int m_leftImageHeight;
    unsigned int m_rightImageWidth;
    unsigned int m_rightImageHeight;

    // homographies
    Matrix3f m_H, m_H_, m_S;
};

#endif // CAMERASENSOR_H

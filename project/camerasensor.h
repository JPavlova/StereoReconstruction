#ifndef CAMERASENSOR_H
#define CAMERASENSOR_H

#include "prerequisites.h"
#include "FreeImageHelper.h"
#include <experimental/filesystem>
#include <regex>

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

const std::regex IMAGE_NAME_REGEX (".*im[0-9]+[.]png");

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

        //// PARAMETERS ADAPTED FOR EASY DATASET

        m_leftImageWidth = 2864;
        m_leftImageHeight = 1924;
        m_rightImageWidth = 2864;
        m_rightImageHeight = 1924;

        m_leftIntrinsics << 2945.377f, 0.f, 1284.862f,
                            0.0f, 2945.377f, 954.52f,
                            0.0f, 0.0f, 1.f;
        m_leftExtrinsics.setIdentity();

        m_rightIntrinsics << 2945.377f, 0.0f, 1455.543f,
                             0.0f, 2945.377f, 954.52f,
                             0.0f, 0.0f, 1.0f;
        m_rightExtrinsics <<    1.0f, 0.0f, 0.0f, 178.232f, // translated along x axis by 178.232mm
                                0.0f, 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f;

        // create arrays and read in dummy values
        m_leftFrame = new BYTE[4 * m_leftImageWidth * m_leftImageHeight];
        for (unsigned int i = 0; i < 4 * m_leftImageWidth * m_leftImageHeight; i++) m_leftFrame[i] = 255;
        m_rightFrame = new BYTE[4 * m_rightImageWidth * m_rightImageHeight];
        for (unsigned int i = 0; i < 4 * m_rightImageWidth * m_rightImageHeight; i++) m_rightFrame[i] = 255;

        // set index to start with

        m_currentIdx = -1;
        return true;

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
                    m_imagePairNameVector.push_back(std::pair<std::string, std::string>(temp_imageName, imageName));
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

        FreeImageB leftImage;
        leftImage.LoadImageFromFile(m_imagePairNameVector[m_currentIdx].first);
        memcpy(m_leftFrame, leftImage.data, 4 * m_leftImageWidth * m_leftImageHeight);

        FreeImageB rightImage;
        rightImage.LoadImageFromFile(m_imagePairNameVector[m_currentIdx].second);
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

private:
    // directory
    std::string m_dataDir;
    std::vector<std::pair<std::string, std::string>> m_imagePairNameVector;

    // current frame index
    int m_currentIdx;

    // color images
    BYTE* m_leftFrame;
    BYTE* m_rightFrame;

    // Properties of both cameras
    Eigen::Matrix3f m_leftIntrinsics;
    Eigen::Matrix4f m_leftExtrinsics;

    Eigen::Matrix3f m_rightIntrinsics;
    Eigen::Matrix4f m_rightExtrinsics;

    unsigned int m_leftImageWidth;
    unsigned int m_leftImageHeight;
    unsigned int m_rightImageWidth;
    unsigned int m_rightImageHeight;

};

#endif // CAMERASENSOR_H

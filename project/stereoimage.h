#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include <vector>
#include "prerequisites.h"
#include "camerasensor.h"
#include <iostream>
#include <random>
#include <fstream>

using namespace Eigen;

// start of class

class StereoImage {
    public:
        StereoImage(CameraSensor *sensor);
        ~StereoImage()
        {
            SAFE_DELETE_ARRAY(m_leftImage);
            SAFE_DELETE_ARRAY(m_rightImage);
            SAFE_DELETE_ARRAY(m_leftImageRectified);
            SAFE_DELETE_ARRAY(m_rightImageRectified);
            SAFE_DELETE_ARRAY(m_depthImage);
        }
        void rectify();
        void derectifyDepthMap();
        void patchmatch();
        void disparityToDepth();

        bool backproject_frame(Vertex *vertices);

        // GETTERS

        Pixel *getLeftImage() const;
        Pixel *getRightImage() const;
        int getLeftImageWidth() const;
        int getLeftImageHeight() const;
        int getRightImageWidth() const;
        int getRightImageHeight() const;
        std::optional<Pixel> *getLeftImageRectified() const;
        std::optional<Pixel> *getRightImageRectified() const;
        Pixel *getLeftImageRectifiedUnoptional() const;
        Pixel *getRightImageRectifiedUnoptional() const;
        int *getLookup() const;
        float *getDepthImage();
        float *getRectifiedDepthImage();
        float *getDisparity();
        void writeDepthMapToFile();

        CameraSensor *sensor;

        void setDepthImage(float *depthImage);

private:
        // raw data
        Pixel* m_leftImage;
        Pixel* m_rightImage;

        int m_leftImageWidth;
        int m_leftImageHeight;
        int m_rightImageWidth;
        int m_rightImageHeight;

        // empty at beginning, filled with a rectified copy of the image
        std::optional<Pixel>* m_leftImageRectified;
        std::optional<Pixel>* m_rightImageRectified;

        // index lookup tables
        int* m_lookUpTransformedIndices;

        // disparities as floats
        float* m_disparity;

        // final output, empty at the beginning
        float* m_depthImage;
        float* m_depthImageRectified;

};

#endif // STEREOIMAGE_H

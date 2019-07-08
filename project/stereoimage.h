#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include <vector>
#include "prerequisites.h"
#include "camerasensor.h"
#include <iostream>

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
        void patchmatch();

        bool backproject_frame(Vertex *vertices);

        // GETTERS

        Pixel *getLeftImage() const;
        Pixel *getRightImage() const;
        int getLeftImageWidth() const;
        int getLeftImageHeight() const;
        int getRightImageWidth() const;
        int getRightImageHeight() const;
        Pixel *getLeftImageRectified() const;
        Pixel *getRightImageRectified() const;
        int *getLeftImageLookup() const;
        int *getRightImageLookup() const;
        float *getDepthImage();

        // SETTERS
        void setLeftImageRectified(Pixel* value);
        void setRightImageRectified(Pixel* value);

private:
        CameraSensor *sensor;

        // raw data
        Pixel* m_leftImage;
        Pixel* m_rightImage;

        int m_leftImageWidth;
        int m_leftImageHeight;
        int m_rightImageWidth;
        int m_rightImageHeight;

        // empty at beginning, filled with a rectified copy of the image
        Pixel* m_leftImageRectified;
        Pixel* m_rightImageRectified;

        // index lookup tables
        int* m_leftImageLookup;
        int* m_rightImageLookup;

        // final output, empty at the beginning
        float* m_depthImage;

};

#endif // STEREOIMAGE_H

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
        StereoImage(BYTE * leftImage, BYTE * rightImage, CameraSensor *sensor);
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
        std::vector<Feature> getLeftFeatures() const;
        std::vector<Feature> getRightFeatures() const;
        std::vector<std::pair<int, int> > getFeatureMatches() const;
        float *getDepthImage();

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

        // individual features
        std::vector<Feature> m_leftFeatures;
        std::vector<Feature> m_rightFeatures;

        // tuple list of features that match
        std::vector<std::pair<int, int>> m_featureMatches;

        // final output, empty at the beginning
        float* m_depthImage;

};

#endif // STEREOIMAGE_H

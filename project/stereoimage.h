#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include <vector>
#include "prerequisites.h"

using namespace Eigen;

// start of class

class StereoImage {
    public:
        StereoImage(unsigned char * image_left, unsigned char * image_right);
        ~StereoImage()
        {
            SAFE_DELETE_ARRAY(image_left);
            SAFE_DELETE_ARRAY(image_right);
            SAFE_DELETE_ARRAY(image_left_rect);
            SAFE_DELETE_ARRAY(image_right_rect);
            SAFE_DELETE_ARRAY(image_depth);
        }
        void rectify();
        void patchmatch();
        void reconstruct();


    private:
        // raw data
        Pixel* image_left;
        Pixel* image_right;

        // empty at beginning, filled with a rectified copy of the image
        Pixel* image_left_rect;
        Pixel* image_right_rect;

        // individual features
        std::vector<Feature> features_left;
        std::vector<Feature> features_right;

        // tuple list of features that match
        std::vector<std::pair<int, int>> feature_matches;

        // final output, empty at the beginning
        float* image_depth;

};

#endif // STEREOIMAGE_H

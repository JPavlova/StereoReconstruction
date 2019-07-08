#ifndef PATCHMATCH_H
#define PATCHMATCH_H

#include "stereoimage.h"
#include <optional>

class PatchMatch
{

public:
    /*Images must be rectified
    Patch size is the distance from the center to the border of each patch, i.e. for 3x3 patches patch size would be 1 (must be odd!)*/
    PatchMatch(std::optional<Pixel> *leftImage, std::optional<Pixel> *rightImage, int width, int height, int patchSize);
    int *computeDisparity();

private:
    int evalNeighborhood(int center_left, int center_right);
    void propagate(int idx);
    void randomSearch(int row, int idx);

    std::optional<Pixel> *m_leftImage;
    std::optional<Pixel> *m_rightImage;
    int *m_disparity;
    int *m_neighborhood;
    int m_width, m_height, m_patchSize;
};

#endif

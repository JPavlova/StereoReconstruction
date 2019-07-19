#ifndef MATCHER_H
#define MATCHER_H

#include "stereoimage.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

class Matcher
{
public:
    Matcher(StereoImage *stereoImage, int patchSize, int searchWindow, float alpha);

    void runBlockMatch();
    void runPatchMatch(int iterations);
    void runOpenCVMatch();
    int *getDisparityMap();
    float *getDepthMap();
    void reset();

    void setPatchSize(int size);

private:
    void propagate(int x, int y, int mode);
    void randomSearch(int x, int y);
    float patchDistance(int posX, int posY, int offsetX);

    void color(std::optional<Pixel> *image, int x, int y, int *c);
    float diff(int *c1, int *c2);
    bool validPixel(int x, int y);
    int idx(int x, int y);

    float m_alpha;
    int m_searchWindow;
    int m_patchSize;
    int m_width;
    int m_height;

    StereoImage *m_stereoImage;
    std::optional<Pixel> *m_leftImage, *m_rightImage;

    int *m_nearestNeighborField;
    float *m_patchDistances;
    float *m_depthMap;
};

#endif // MATCHER_H

#ifndef MATCHER_H
#define MATCHER_H

#include "stereoimage.h"

class Matcher
{
public:
    Matcher(StereoImage *stereoImage, int patchSize, int searchWindow, float alpha);

    void runBlockMatch();
    void runPatchMatch(int iterations);
    int *getDisparityMap();
    float *getDepthMap();

private:
    void propagate(int x, int y, int mode);
    void randomSearch(int x, int y);
    float patchDistance(int posX, int posY, int offsetX);

    void color(unsigned char *image, int x, int y, int *c);
    float diff(int *c1, int *c2);
    bool validPixel(int x, int y);
    int idx(int x, int y);

    float m_alpha;
    int m_searchWindow;
    int m_patchSize;
    int m_width;
    int m_height;

    StereoImage *m_stereoImage;
    unsigned char *m_leftImage, *m_rightImage;
    int *m_nearestNeighborField;
    float *m_patchDistances;
    float *m_depthMap;
};

#endif // MATCHER_H

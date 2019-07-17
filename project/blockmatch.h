#ifndef BLOCKMATCH_H
#define BLOCKMATCH_H

#include "stereoimage.h"

class BlockMatch
{
public:
    BlockMatch(StereoImage *stereoImage, int blockSize, int searchWindow);

    void run();
    int *getDisparityMap();
    float *getDepthMap();

private:
    float patchDistance(int posX, int posY, int offsetX);
    void color(unsigned char *image, int x, int y, int *c);
    float diff(int *c1, int *c2);
    bool validPixel(int x, int y);
    int idx(int x, int y);

    float m_alpha;
    int m_searchWindow;
    int m_blockSize;
    int m_width;
    int m_height;

    StereoImage *m_stereoImage;
    unsigned char *m_leftImage, *m_rightImage;
    int *m_nearestNeighborField;
    float *m_patchDistances;
    float *m_depthMap;
};

#endif // BLOCKMATCH_H

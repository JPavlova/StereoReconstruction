#include "blockmatch.h"

BlockMatch::BlockMatch(StereoImage *stereoImage, int blockSize, int searchWindow) :
    m_leftImage((unsigned char *) stereoImage->getLeftImageRectified()),
    m_rightImage((unsigned char *) stereoImage->getRightImageRectified()),
    m_width(stereoImage->getLeftImageWidth()),
    m_height(stereoImage->getLeftImageHeight()),
    m_blockSize(blockSize),
    m_searchWindow(searchWindow)
{
    m_nearestNeighborField = new int[m_width * m_height]();
    m_patchDistances = new float[m_width * m_height]();

    int i = 0;
    for (int y = 0; y < m_height; y++) {
        for (int x = 0; x < m_width; x++) {
            i = idx(x, y);
            m_nearestNeighborField[i] = 0;
            m_patchDistances[i] = INF;
        }
    }
}

void BlockMatch::run()
{
#pragma omp parallel for
    for (int y = m_blockSize/2; y < (m_height-m_blockSize/2); y++) {
        std::cout << "process: " << (y * 100)/m_height << " %" << std::endl;
        for (int x = m_blockSize/2; x < m_width-m_blockSize/2; x++) {
            int i = idx(x, y);
            for (int x2 = x; x2 >= std::max(m_blockSize/2, x-m_searchWindow); x2--) {
                int offset = x2 - x;
                float dist = patchDistance(x, y, offset);

                if (dist < m_patchDistances[i]) {
                    m_patchDistances[i] = dist;
                    m_nearestNeighborField[i] = -offset;
                }
            }
        }
    }
}

int *BlockMatch::getDisparites()
{
    return m_nearestNeighborField;
}

float BlockMatch::patchDistance(int posX, int posY, int offsetX) {
    float result = 0.0f;
    int *left_color = new int[4]();
    int *right_color = new int[4]();
    int pixelCounter = 0;

    for (int y = -m_blockSize/2; y < m_blockSize/2; y++) {
        for (int x = -m_blockSize/2; x < m_blockSize/2; x++) {
            if (validPixel(posX + x, posY + y) && validPixel(posX + offsetX + x, posY + y)) {
                color(m_leftImage, posX + x, posY + y, left_color);
                color(m_rightImage, posX + x + offsetX, posY + y, right_color);
                result += diff(left_color, right_color);
                pixelCounter++;
            }
            else {
                return INF;
            }
        }
    }
    delete(left_color);
    delete(right_color);
    return result / pixelCounter;
}

bool BlockMatch::validPixel(int x, int y) {
    return (x >= 0) && (y >= 0) && (x < m_width) && (y < m_height);
}

void BlockMatch::color(unsigned char *image, int x, int y, int *c)
{
    int i = idx(x, y) * 4;
    c[0] = (int) image[i];
    c[1] = (int) image[i+1];
    c[2] = (int) image[i+2];
    c[3] = (int) image[i+3];
}

float BlockMatch::diff(int *c1, int *c2)
{
    float result = 0.0f;
    for (int i = 0; i < 3; i++) {
        result += std::abs(c1[i] - c2[i]);
    }
    return result;
}

int BlockMatch::idx(int x, int y)
{
    return y * m_width + x;
}

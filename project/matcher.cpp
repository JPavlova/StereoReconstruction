#include "matcher.h"

/**
 * @brief Matcher::Matcher this class encapsulates both matching methods we are considering: BlockMatch and PatchMatch.
 * @param stereoImage
 * @param blockSize
 * @param searchWindow
 * @param alpha
 */
Matcher::Matcher(StereoImage *stereoImage, int patchSize, int searchWindow, float alpha) :
    m_stereoImage(stereoImage),
    m_leftImage((unsigned char *) stereoImage->getRightImage()), // because is wrong somewhere else in the code...
    //TODO: actually use unrectified images here.. however only working with original images.
    m_rightImage((unsigned char *) stereoImage->getLeftImage()),
    m_width(stereoImage->getLeftImageWidth()),
    m_height(stereoImage->getLeftImageHeight()),
    m_patchSize(patchSize),
    m_searchWindow(searchWindow),
    m_alpha(alpha)
{
    m_nearestNeighborField = new int[m_width * m_height]();
    m_patchDistances = new float[m_width * m_height]();
    m_depthMap = new float[m_width * m_height]();

    int i = 0;
    for (int y = 0; y < m_height; y++) {
        for (int x = 0; x < m_width; x++) {
            i = idx(x, y);
            m_nearestNeighborField[i] = 0;
            m_patchDistances[i] = INF;
        }
    }
}

/**
 * @brief Matcher::runPatchMatch implements the standard PatchMatch algorithm.
 * @param iterations
 */
void Matcher::runPatchMatch(int iterations) {
    for (int i = 0; i < iterations; i++) {
#pragma omp parallel for
        for (int y = m_patchSize/2; y < m_height-m_patchSize/2; y++) {
            for (int x = m_patchSize/2; x < m_width-m_patchSize/2; x++) {
                if (i % 2 == 0)
                    propagate(x, y, 0);
                else
                    propagate(x, y, 1);

                randomSearch(x, y);
            }
        }
    }
}

/**
 * @brief Matcher::randomSearch allows only one directional offsets in a certain window.
 * @param x
 * @param y
 */
void Matcher::randomSearch(int x, int y) {
    int i = idx(x, y);
    int cutoff = std::max(x-m_searchWindow, 0);
    float radius = m_alpha * (m_width-m_patchSize);

    while (radius > 1) {
        int randomOffset = -(std::rand() % (x - cutoff));
        float dist = patchDistance(x, y, randomOffset);
        if (dist < m_patchDistances[i]) {
            m_patchDistances[i] = dist;
            m_nearestNeighborField[i] = -randomOffset;
        }

        radius *= m_alpha;
    }
}

/**
 * @brief Matcher::propagate works exactly according to the paper, switches lookup direction according to current iteration
 * @param x
 * @param y
 * @param mode
 */
void Matcher::propagate(int x, int y, int mode) {
    int i = idx(x, y);
    int offset;

    if (mode == 0)
        offset = -1;
    else
        offset = 1;

    if (validPixel(x+offset, y)) {
        int iH = idx(x+offset, y);
        float distH = patchDistance(x, y, m_nearestNeighborField[iH]);
        if (distH < m_patchDistances[i]) {
            m_patchDistances[i] = distH;
            m_nearestNeighborField[i] = m_nearestNeighborField[iH];
        }
    }

    int iV = idx(x, y+offset);
    if (validPixel(x, y+offset)) {
        float distV = patchDistance(x, y, m_nearestNeighborField[iV]);
        if (distV < m_patchDistances[i]) {
            m_patchDistances[i] = distV;
            m_nearestNeighborField[i] = m_nearestNeighborField[iV];
        }
    }
}

/**
 * @brief Matcher::runBlockMatch implements the standard BlockMatch algorithm, but searches only in one direction and only in a certain window.
 */
void Matcher::runBlockMatch()
{
#pragma omp parallel for
    for (int y = m_patchSize/2; y < (m_height-m_patchSize/2); y++) {
        for (int x = m_patchSize/2; x < m_width-m_patchSize/2; x++) {
            int i = idx(x, y);
            for (int x2 = x; x2 >= std::max(m_patchSize/2, x-m_searchWindow); x2--) {
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

int *Matcher::getDisparityMap()
{
    return m_nearestNeighborField;
}

float *Matcher::getDepthMap()
{
    float z_inPixel = 0.0f;
    float result = MINF;
    float focalLength = m_stereoImage->sensor->getFocalLength();
    float baseline = m_stereoImage->sensor->getBaseline();
    std::cout << "f: " << focalLength << std::endl;
    std::cout << "baseline: " << baseline << std::endl;

    for (int i = 0; i < m_width * m_height; i++) {
        if (m_nearestNeighborField[i] > 0) {
            z_inPixel = (baseline * focalLength) / m_nearestNeighborField[i];
            m_depthMap[i] = z_inPixel;
        }
        else {
            m_depthMap[i] = MINF;
        }
    }

    return m_depthMap;
}

/**
 * @brief Matcher::patchDistance calculates the summed absolute pixel intensity differences of a patch.
 * @param posX
 * @param posY
 * @param offsetX
 * @return
 */
float Matcher::patchDistance(int posX, int posY, int offsetX) {
    float result = 0.0f;
    int *left_color = new int[4]();
    int *right_color = new int[4]();
    int pixelCounter = 0;

    for (int y = -m_patchSize/2; y < m_patchSize/2; y++) {
        for (int x = -m_patchSize/2; x < m_patchSize/2; x++) {
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

bool Matcher::validPixel(int x, int y) {
    return (x >= 0) && (y >= 0) && (x < m_width) && (y < m_height);
}

/**
 * @brief Matcher::color retrieves the color of a pixel in the provided image.
 * @param image
 * @param x
 * @param y
 * @param c
 */
void Matcher::color(unsigned char *image, int x, int y, int *c)
{
    int i = idx(x, y) * 4;
    c[0] = (int) image[i];
    c[1] = (int) image[i+1];
    c[2] = (int) image[i+2];
    c[3] = (int) image[i+3];
}

float Matcher::diff(int *c1, int *c2)
{
    float result = 0.0f;
    for (int i = 0; i < 3; i++) {
        result += std::abs(c1[i] - c2[i]);
    }
    return result;
}

int Matcher::idx(int x, int y)
{
    return y * m_width + x;
}

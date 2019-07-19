#include "matcher.h"

#define MIN_OVERLAP 0.01f

/**
 * @brief Matcher::Matcher this class encapsulates both matching methods we are considering: BlockMatch and PatchMatch.
 * @param stereoImage
 * @param blockSize
 * @param searchWindow
 * @param alpha
 */
Matcher::Matcher(StereoImage *stereoImage, int patchSize, int searchWindow, float alpha) :
    m_stereoImage(stereoImage),
    m_leftImage(stereoImage->getLeftImageRectified()),
    m_rightImage(stereoImage->getRightImageRectified()),
    m_width(stereoImage->getLeftImageWidth()),
    m_height(stereoImage->getLeftImageHeight()),
    m_patchSize(patchSize),
    m_searchWindow(searchWindow),
    m_alpha(alpha)
{
    m_nearestNeighborField = new int[m_width * m_height]();
    m_matchIndex = new int[m_width * m_height]();
    m_patchDistances = new float[m_width * m_height]();
    m_depthMap = new float[m_width * m_height]();

    int i = 0;
    for (int y = 0; y < m_height; y++) {
        for (int x = 0; x < m_width; x++) {
            i = idx(x, y);
            m_nearestNeighborField[i] = INT_MIN;
            m_matchIndex[i] = INT_MIN;
            m_patchDistances[i] = 255.f*4.f*(float)m_patchSize*(float)m_patchSize*MIN_OVERLAP;
        }
    }
}

void Matcher::reset() {
    int i = 0;
    for (int y = 0; y < m_height; y++) {
        for (int x = 0; x < m_width; x++) {
            i = idx(x, y);
            m_nearestNeighborField[i] = INT_MIN;
            m_matchIndex[i] = INT_MIN;
            m_patchDistances[i] = 255.f*4.f*(float)m_patchSize*(float)m_patchSize*MIN_OVERLAP;
        }
    }
}

void Matcher::runOpenCVMatch() {
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16*5, m_patchSize);
    cv::Mat img1(m_height, m_width, CV_8UC1);
    cv::Mat img2(m_height, m_width, CV_8UC1);
    cv::Mat disp(m_height, m_width, CV_16SC1);

    for (int col = 0; col < m_width; ++col) {
        for (int row = 0; row < m_height; ++row) {
            int idx = row * m_width + col;
            if(m_leftImage[idx].has_value()){
                img1.at<unsigned char>(row, col) = 0.299 * m_leftImage[idx].value()[0] +  0.587 * m_leftImage[idx].value()[1] + 0.114 * m_leftImage[idx].value()[2];
            }
            if(m_rightImage[idx].has_value()){
                img2.at<unsigned char>(row, col) = 0.299 * m_rightImage[idx].value()[0] +  0.587 * m_rightImage[idx].value()[1] + 0.114 * m_rightImage[idx].value()[2];
            }
        }
    }

    bm->compute(img1, img2, disp);

    cv::Mat1d easyDisp(disp);
    for (int row = 0; row < m_height; ++row) {
        for (int col = 0; col < m_width; ++col) {
            int idx = row * m_width + col;
            int disp_ = easyDisp[row][col];
            //            std::cout << "Disparity at (" << row << ", " << col << "):\t" << disp_ << std::endl;
            m_nearestNeighborField[idx] = disp_ == -16 ? INT_MIN : disp_;
        }
    }
}

/**
 * @brief Matcher::runPatchMatch implements the standard PatchMatch algorithm.
 * @param iterations
 */
void Matcher::runPatchMatch(int iterations) {
    for (int i = 0; i < iterations; i++) {
        for (int y = m_patchSize/2; y < m_height-m_patchSize/2; y++) {
#pragma omp parallel for
            for (int x = m_patchSize/2; x < m_width-m_patchSize/2; x++) {

                if(m_leftImage[idx(x, y)].has_value()){
                    if (i % 2 == 0)
                        propagate(x, y, 0);
                    else
                        propagate(x, y, 1);

                    randomSearch(x, y);
                }
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
    float current_alpha = m_alpha;
    for(int iterations = 0; iterations < 5; iterations++) {
        int randomOffset = 1.0 * rand() / RAND_MAX * (m_width-m_patchSize);
        int clippedOffset = current_alpha * (randomOffset - x) + m_patchSize/2;
        //        std::cout << "Clipped offset " << clippedOffset << " for " << x << std::endl;
        float dist = patchDistance(x, y, clippedOffset);
        if (dist < m_patchDistances[i]) {
            m_patchDistances[i] = dist;
            m_nearestNeighborField[i] = clippedOffset;
            m_matchIndex[i] = i + clippedOffset;
            //            std::cout << "Found a better one!" << std::endl;
        }
        current_alpha *= m_alpha;
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
 * @brief Matcher::runBlockMatch implements the standard BlockMatch algorithm, but searches in both directions and only in a certain window.
 */
void Matcher::runBlockMatch()
{
#pragma omp parallel for collapse(2)
    for (int y = m_patchSize/2; y < (m_height-m_patchSize/2); y++) {
        for (int x = m_patchSize/2; x < m_width-m_patchSize/2; x++) {
            if(!m_leftImage[idx(x, y)].has_value()){
                continue;
            }
#pragma omp parallel for
            for (int x2 = std::max(m_patchSize/2, x-m_searchWindow); x2 <= std::max(m_width-m_patchSize/2, x+m_searchWindow); x2++) {
                int i = idx(x, y);
                int offset = x2 - x;
                float dist = patchDistance(x, y, offset);

                if (dist < m_patchDistances[i]) {
                    m_patchDistances[i] = dist;
                    m_nearestNeighborField[i] = offset;
                    m_matchIndex[i] = i + offset;
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
        if (m_nearestNeighborField[i] == INT_MIN && m_matchIndex[i] == INT_MIN) {
            m_depthMap[i] = MINF;
        }
        else if (m_matchIndex[i] == INT_MIN){
            z_inPixel = (baseline * focalLength) / abs(m_nearestNeighborField[i]);
            m_depthMap[i] = z_inPixel;
        }
        else {
            int uL = i % m_width - m_width/2;
            int uR = m_width/2 - m_matchIndex[i] % m_width;
            if (uL + uR == 0) {
                m_depthMap[i] = MINF;
                continue;
            }
            result = (baseline * focalLength) / (float)(uL + uR);
            m_depthMap[i] = result;
            //            std::cout << "UL: " << uL << ", UR: " << uR << " (compared to disparity: " << m_nearestNeighborField[i] << ") --> Z: " << m_depthMap[i] << std::endl;
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
            if (validPixel(posX + x, posY + y) && validPixel(posX + offsetX + x, posY + y) &&
                    m_leftImage[idx(posX + x, posY + y)].has_value() && m_rightImage[idx(posX + x + offsetX, posY + y)].has_value()) {

                color(m_leftImage, posX + x, posY + y, left_color);
                color(m_rightImage, posX + x + offsetX, posY + y, right_color);
                result += diff(left_color, right_color);
                pixelCounter++;
            }
            else {
                delete(left_color);
                delete(right_color);
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
void Matcher::color(std::optional<Pixel> *image, int x, int y, int *c)
{
    int i = idx(x, y);
    c[0] = (int) image[i].value()[0];
    c[1] = (int) image[i].value()[1];
    c[2] = (int) image[i].value()[2];
    c[3] = (int) image[i].value()[3];
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

void Matcher::setPatchSize(int size) {
    m_patchSize = size;
}

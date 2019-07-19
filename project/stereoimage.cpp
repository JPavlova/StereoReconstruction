#include "stereoimage.h"

#define BILINEAR_INTERPOLATION false

/**
 * @brief StereoImage::StereoImage
 * create StereoImage from left/right BYTE * arrays and camera sensor, save in Pixel* array for left/right images
 * @param leftImage : BYTE * array+
 * @param rightImage : BYTE * array
 * @param sensor : CameraSensor * object
 */
StereoImage::StereoImage(CameraSensor * sensor) : sensor(sensor)
{

    BYTE * leftImage = sensor->getLeftFrame();
    BYTE * rightImage = sensor->getRightFrame();

    m_leftImageWidth = sensor->getLeftImageWidth();
    m_leftImageHeight = sensor->getLeftImageHeight();
    m_rightImageWidth = sensor->getRightImageWidth();
    m_rightImageHeight = sensor->getRightImageHeight();

    m_leftImage = new Pixel[m_leftImageWidth * m_leftImageHeight];
    m_rightImage = new Pixel[m_rightImageWidth * m_rightImageHeight];
    m_depthImageRectified = new float[m_leftImageWidth * m_leftImageHeight];
    m_depthImage = new float[m_leftImageWidth * m_leftImageHeight];
    m_disparity = new float[m_leftImageWidth * m_leftImageHeight];

    // read in BYTE pixel values as Pixel (Vectur4uc)
    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_leftImage[idx] = Pixel(leftImage[4*idx], leftImage[4*idx+1], leftImage[4*idx+2], leftImage[4*idx+3]);
    for(unsigned int idx = 0; idx < m_rightImageWidth * m_rightImageHeight; idx++) m_rightImage[idx] = Pixel(rightImage[4*idx], rightImage[4*idx+1], rightImage[4*idx+2], rightImage[4*idx+3]);
    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_depthImage[idx] = MINF;
    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_depthImageRectified[idx] = MINF;
    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_disparity[idx] = MINF;

    // set up arrays for left/right rectified image, fill with optional
    m_leftImageRectified = new std::optional<Pixel>[m_leftImageWidth * m_leftImageHeight];
    m_rightImageRectified = new std::optional<Pixel>[m_rightImageWidth * m_rightImageHeight];

    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_leftImageRectified[idx] = std::nullopt;
    for(unsigned int idx = 0; idx < m_rightImageWidth * m_rightImageHeight; idx++) m_rightImageRectified[idx] = std::nullopt;

    // set up lookup arrays
    m_lookUpTransformedIndices = new int[m_leftImageWidth * m_leftImageHeight];

    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_lookUpTransformedIndices[idx] = -1;
}

/**
 * @brief StereoImage::backproject_frame
 * @param vertices Output argument, array of vertices is filled in this method. HAS to be in the right dimensions!
 * @return
 */

bool StereoImage::backproject_frame(Vertex *vertices)
{
    Pixel *colorMap = m_leftImage;
    float *depthMap = m_depthImage;

    Matrix3f leftIntrinsicsInv = sensor->getLeftIntrinsics().inverse();

#pragma omp parallel for
    for (int v = 0; v < sensor->getLeftImageHeight(); v++) {
        for (int u = 0; u < sensor->getLeftImageWidth(); u++) {

            int i = v * sensor->getLeftImageWidth() + u;
            float w = depthMap[i];

            if (w == MINF || w == INF) {
                vertices[i].position = Vector4f(MINF, MINF, MINF, MINF);
                vertices[i].color = Pixel(0, 0, 0, 0);
            }
            else {
                Vector3f p_image = Vector3f(u * w, v * w, w);
                Vector3f p_camera = leftIntrinsicsInv * p_image; // already in world coordinates, no need for extrinsics-mult.

                // color retrieval
                Pixel rgb = colorMap[i];

                vertices[i].position = Vector4f(p_camera.x(), p_camera.y(), p_camera.z(), 1);
                vertices[i].color = rgb;
            }
        }
    }

    return true;
}

void StereoImage::derectifyDepthMap() {
    // OLD CODE WITH ADDITIONAL TRANSFORMATION NEEDED COMMENTED

    Matrix3f H = sensor->getH();
    Matrix3f S = sensor->getS();
    Matrix3f inverse = (S * H).inverse();

#pragma omp parallel for collapse(2)
    for (int row = 0; row < m_leftImageHeight; row++) {
        for (int col = 0; col < m_leftImageWidth; col++) {

            Vector3f position, transformed_l;
            int x_left, y_left;

            int i = row * m_leftImageWidth + col;

            position << col, row, 1;

            transformed_l = inverse * position;
            transformed_l /= transformed_l(2);

            x_left = std::round(transformed_l(0));
            y_left = std::round(transformed_l(1));

            if ((x_left >= 0) && (y_left >= 0) && (x_left < m_leftImageWidth) && (y_left < m_leftImageHeight)) {
                int idx_l = y_left * m_leftImageWidth + x_left;
                m_depthImage[idx_l] = m_depthImageRectified[i];
            }
        }
    }
}

void StereoImage::rectify() {
    Matrix3f H = sensor->getH();
    Matrix3f H_ = sensor->getH_();
    Matrix3f S = sensor->getS();
    Matrix3f S_H_inverse = (S * H).inverse();
    Matrix3f S_H__inverse = (S * H_).inverse();

#pragma omp parallel for collapse(2)
    for (int row = 0; row < m_leftImageHeight; row++) {
        for (int col = 0; col < m_leftImageWidth; col++) {

            Vector3f position, transformed_l, transformed_r;
            int x_left, y_left, x_right, y_right;
            int i = row * m_leftImageWidth + col;
            position << col, row, 1;

            transformed_l = S_H_inverse * position;
            transformed_l /= transformed_l(2);
            transformed_r = S_H__inverse * position;
            transformed_r /= transformed_r(2);

            x_left = std::round(transformed_l(0));
            y_left = std::round(transformed_l(1));
            x_right= std::round(transformed_r(0));
            y_right= std::round(transformed_r(1));

            if(!BILINEAR_INTERPOLATION) {
                // pick corresponding color from untransformed image, nearest neighbour right now and not interpolated
                if ((x_left >= 0) && (y_left >= 0) && (x_left < m_leftImageWidth) && (y_left < m_leftImageHeight)) {
                    int idx_l = y_left * m_leftImageWidth + x_left;
                    m_leftImageRectified[i] = m_leftImage[idx_l];
                    m_lookUpTransformedIndices[y_left * m_leftImageWidth + x_left] = i;
                }

                if ((x_right >= 0) && (y_right >= 0) && (x_right < m_rightImageWidth) && (y_right < m_rightImageHeight)) {
                    int idx_l = y_right * m_rightImageWidth + x_right;
                    m_rightImageRectified[i] = m_rightImage[idx_l];
                }
            }
            else {
                // bilinear interpolation, check time differences?
                auto bilinearInterpolation = [&](Vector3f& transform, Pixel* image, std::optional<Pixel>* rectifiedImage, int width, int height) {
                    if(0 <= transform[0] && transform[0] <= (float)width && 0 <= transform[1] && transform[1] <= (float)height) {
                        int newRow = (int) transform[1];
                        int newCol = (int) transform[0];
                        m_lookUpTransformedIndices[newRow * m_leftImageWidth + newCol] = i;

                        // Bilinear interpolation for values (TODO check if just that simple in RGB)
                        float r = 0.f, g = 0.f, b = 0.f, a = 0.f;
                        float a_ = transform[1] - (float)(int)transform[1]; // dist to up
                        float b_ = transform[0] - (float)(int)transform[0]; // dist to left

                        for (int row_ = -1; row_ <= 1; ++row_) {
                            for (int col_ = -1; col_ <= 1; ++col_) {

                                if( fabs((float)row_ + 0.5f - a_) < 1.f && fabs((float)col_ + 0.5f - b_) < 1.f && // check if closer pixel
                                        0 < newCol + col_ && newCol + col_ < width && 0 < newRow + row_ && newRow + row_ < height){

                                    float f = fabs((float)row_ + 0.5f - a_) * fabs((float)col_ + 0.5f - b_);

                                    r += f * image[(newRow + row_) * width + newCol + col_][0];
                                    g += f * image[(newRow + row_) * width + newCol + col_][1];
                                    b += f * image[(newRow + row_) * width + newCol + col_][2];
                                    a += f * image[(newRow + row_) * width + newCol + col_][3];
                                }
                            }
                        }
                        rectifiedImage[i] = Pixel((int)r, (int)g, (int)b, (int)a);
                    }
                };

                bilinearInterpolation(transformed_l, m_leftImage, m_leftImageRectified, m_leftImageWidth, m_leftImageHeight);
                bilinearInterpolation(transformed_r, m_rightImage, m_rightImageRectified, m_rightImageWidth, m_rightImageHeight);
            }
        }
    }
}


/**
 * @brief StereoImage::disparityToDepth
 * calculate depth
 * z = baseline * focallength / (disparity + doffs)
 * OR: z = focallength * baseline / disparity
 */
void StereoImage::disparityToDepth()
{
    //    int* recL = StereoImage::m_leftImageLookup;
    //    int *recR = StereoImage::m_rightImageLookup;
    //    Pixel* imgL =  getLeftImage();
    //    Pixel* imgR = getRightImage();

    float focalLength = sensor->getFocalLength();
    float baseline = sensor->getBaseline();
    float doffs = sensor->getDoffs();

    // compute depth values
#pragma omp parallel for
    for(int i = 0; i< m_leftImageWidth * m_leftImageHeight; i++)
    {
        if(m_disparity[i] == 0 || m_disparity[i] == MINF){
            m_depthImageRectified[i] = MINF;
        } else {
            m_depthImageRectified[i] = baseline * focalLength / m_disparity[i];
        }
    }
}

// Needed to write out rectified images
static Pixel* unOptionalize(std::optional<Pixel>* image, int size) {
    Pixel* n_image = new Pixel[size];
#pragma omp parallel for
    for(int i = 0; i < size; i++) {
        if(!image[i].has_value()) {
            n_image[i] = Pixel(0, 0, 0, 0);
            continue;
        }
        n_image[i] = image[i].value();
    }
    return n_image;
}

void StereoImage::writeDepthMapToFile() {
    std::ofstream myfile;
    myfile.open("depth.txt");
    for (int row = 0; row < m_leftImageHeight; row++) {
        for (int col = 0; col < m_leftImageWidth; col++) {
            myfile << m_depthImage[row*m_leftImageWidth+col] << "\t";
        }
        myfile << "\n";
    }
    myfile.close();
}


// GETTERS

Pixel *StereoImage::getLeftImage() const
{
    return m_leftImage;
}

Pixel *StereoImage::getRightImage() const
{
    return m_rightImage;
}

int StereoImage::getLeftImageWidth() const
{
    return m_leftImageWidth;
}

int StereoImage::getLeftImageHeight() const
{
    return m_leftImageHeight;
}

int StereoImage::getRightImageWidth() const
{
    return m_rightImageWidth;
}

int StereoImage::getRightImageHeight() const
{
    return m_rightImageHeight;
}

std::optional<Pixel> *StereoImage::getLeftImageRectified() const
{
    return m_leftImageRectified;
}

std::optional<Pixel> *StereoImage::getRightImageRectified() const
{
    return m_rightImageRectified;
}

Pixel *StereoImage::getLeftImageRectifiedUnoptional() const
{
    return unOptionalize(m_leftImageRectified, m_leftImageWidth * m_leftImageHeight);
}

Pixel *StereoImage::getRightImageRectifiedUnoptional() const
{
    return unOptionalize(m_rightImageRectified, m_rightImageWidth * m_rightImageHeight);
}

int *StereoImage::getLookup() const
{
    return m_lookUpTransformedIndices;
}

float* StereoImage::getDepthImage() {
    return m_depthImage;
}

float* StereoImage::getRectifiedDepthImage() {
    return m_depthImageRectified;
}

float* StereoImage::getDisparity() {
    return m_disparity;
}

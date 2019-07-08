#include "stereoimage.h"
#include <random>
//#include <opencv4/opencv2/core.hpp>
//#include <opencv4/opencv2/highgui.hpp>
//#include <opencv4/opencv2/features2d.hpp>
//#include <opencv4/opencv2/xfeatures2d.hpp>
//#include <opencv4/opencv2/xfeatures2d/nonfree.hpp>
//#include <opencv4/opencv2/cvconfig.h>
//#include <opencv4/opencv2/imgproc.hpp>




/**
 * @brief StereoImage::StereoImage
 * create StereoImage from left/right BYTE * arrays and camera sensor, save in Pixel* array for left/right images
 * @param leftImage : BYTE * array
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
    m_depthImage = new float[m_leftImageWidth * m_leftImageHeight];

    // read in BYTE pixel values as Pixel (Vectur4uc)
    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_leftImage[idx] = Pixel(leftImage[4*idx], leftImage[4*idx+1], leftImage[4*idx+2], leftImage[4*idx+3]);
    for(unsigned int idx = 0; idx < m_rightImageWidth * m_rightImageHeight; idx++) m_rightImage[idx] = Pixel(rightImage[4*idx], rightImage[4*idx+1], rightImage[4*idx+2], rightImage[4*idx+3]);
    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_depthImage[idx] = (std::rand() % 100) / 10.0f; // random initialization

    // set up arrays for left/right rectified image, fill with Pixel(-1,-1,-1,-1)
    m_leftImageRectified = new Pixel[m_leftImageWidth * m_leftImageHeight];
    m_rightImageRectified = new Pixel[m_rightImageWidth * m_rightImageHeight];

    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_leftImageRectified[idx] = Pixel(-1, -1, -1, -1);
    for(unsigned int idx = 0; idx < m_rightImageWidth * m_rightImageHeight; idx++) m_rightImageRectified[idx] = Pixel(-1, -1, -1, -1);

    // set up lookup arrays
    m_leftImageLookup = new int[m_leftImageWidth * m_leftImageHeight];
    m_rightImageLookup = new int[m_rightImageWidth * m_rightImageHeight];

    for(unsigned int idx = 0; idx < m_leftImageWidth * m_leftImageHeight; idx++) m_leftImageLookup[idx] = -1;
    for(unsigned int idx = 0; idx < m_rightImageWidth * m_rightImageHeight; idx++) m_rightImageLookup[idx] = -1;


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

    Matrix3f leftIntrinsics = sensor->getLeftIntrinsics();
    Matrix3f leftIntrinsicsInv = sensor->getLeftIntrinsics().inverse();
    float fovX = leftIntrinsics(0, 0);
    float fovY = leftIntrinsics(1, 1);
    float cX = leftIntrinsics(0, 2);
    float cY = leftIntrinsics(1, 2);

    int i = 0;
    float w;

    for (int v = 0; v < sensor->getLeftImageHeight(); v++) {
        for (int u = 0; u < sensor->getLeftImageWidth(); u++) {
            i = v * sensor->getLeftImageWidth() + u;
            w = depthMap[i];

            if (w == MINF) {
                vertices[i].position = Vector4f(MINF, MINF, MINF, MINF);
                vertices[i].color = Pixel(0, 0, 0, 0);
            }
            else {
                Vector3f uvw = Vector3f(u*w, v*w, w);
                Vector3f xyz = leftIntrinsics * uvw; // already in world coordinates, no need for extrinsics-mult.

                // color retrieval
                Pixel rgb = colorMap[i];

                vertices[i].position = Vector4f(xyz.x(), xyz.y(), xyz.z(), 1);
                vertices[i].color = rgb;
            }
        }
    }

    return true;
}

Matrix3f decomposeMatrix(Matrix3f D)
{
    float sum; //Hilfsvariablen

    for(int i = 0; i<3; i++)
    {
        for(int j = 0; j<i; j++)
        {
            sum = D(i,j);
            for(int k = 0; k < j-1; k++)
            {
                sum = sum - D(i,k)*D(j,k);
            }
            if(i > j)
            {
                D(i,j) = sum/ D(j,j);
            }
            else if(sum > 0)
            {
                D(i,i) = sqrt(sum);
            }
        }
    }

    D(0,1) = 0.f;
    D(0,2) = 0.f;
    D(1,2) = 0.f;
    return D;
}

Matrix3f skewSymmetricMatrix(Vector3f& vector) {
    Matrix3f m;
    m << 0.f, -vector.z(), vector.y(),
            vector.z(), 0.f, -vector.x(),
            -vector.y(), vector.x(), 0.f;
    return m;
}

void StereoImage::rectify()
{
    int width = sensor -> getLeftImageHeight();
    int height = sensor->getLeftImageWidth();


    Pixel* leftImage =  getLeftImage();
    Pixel* rightImage =  getRightImage();

    /*Berechne Foundation Matrix F*/

    Matrix3f K1 = sensor->getLeftIntrinsics(); //linke Kamera
    Matrix3f K2 = sensor->getRightIntrinsics(); //rechte Kamera
    Matrix4f Rt = sensor->getRightExtrinsics();
    Matrix3f R = Rt.block<3,3>(0,0);                                //Rotationsmatrix abgeleitet von der Extrinsic von der rechten der Kamera
    // R << Rt(0,0), 0.f, 0.f,
    //        0.f, Rt(1,1) ,0.f,
    //         0.f, 0.f, Rt(2,2);
    Vector3f t = Rt.block<3, 1>(0, 3);                                //Transformationsvector
    //    t << Rt(0,3), Rt(1,3), Rt(2,3);

    Matrix3f F;

    //Berechne die asymmetrische Matrix zur Kreuzproduktdarstellung [x]_x

    Vector3f tmpv = K1*R.transpose()*t;
    Matrix3f tmpm = skewSymmetricMatrix(tmpv);

    F = K2.inverse().transpose()*R*K1.transpose()*tmpm;

    /*Berechne Epipolare Linien*/
    Vector3f e1,e2; // not sure if e2 is even needed
    e1 = (-1.f)*K1.transpose()*t;
    e2 = K2*t;

    /*Berechne z*/
    /*Berechne A,B bzw A',B'*/
    Matrix3f A1,B1,A2,B2;
    Matrix3f pp, ppc;

    pp << ((width*height)/12)* width*width - 1 , 0.f, 0.f,
            0.f, ((width*height)/12)*height*height-1, 0.f,
            0.f, 0.f, 0.f;

    ppc << 1/4* powf(width-1,2), 1/4*(width-1)*(height-1), 0.f,
            1/4*(width-1)*(height-1), 1/4*(powf(height-1,2)), 0.f,
            0.f, 0.f, 0.f;

    Matrix3f esx1 = skewSymmetricMatrix(e1);
    Matrix3f esx2 = skewSymmetricMatrix(e2); // is this even needed?

    A1 = esx1.transpose()*pp*esx1;
    B1 = esx1.transpose()*ppc*esx1;

    A2 = F.transpose()*pp*F;
    B2 = F.transpose()*ppc*F;

    /*A = D*D.traspose()*/

    Matrix3f D = decomposeMatrix(A1);

    /*y=Eigenvector of D.inv.trans*B*D.inv*/
    EigenSolver<Matrix3f> y(D.inverse().transpose()*B1*D.inverse());

    /*z = D.inv*y*/
    Vector3f z = D.inverse() * y.pseudoEigenvectors().col(0); // TODO: decide if complex EV needed

    Vector3f w1, w2;
    w1 = esx1*z;
    w2 = F*z;

    float vc = 0; // minimum v-coordinate of a pixel in any image, set to zero so far

    Matrix3f Hr1, Hr2, Hp1, Hp2;
    Hr1 << F(2,1)-w1.y()*F(2,2), w1.x()*F(2,2)-F(2,1), 0.f,
            F(2,0)-w1.x()*F(2,2), F(2,1)-w1.y()*F(2,2), F(2,2)+vc,
            0.f, 0.f, 1.f;
    Hr2 << F(1,2)-w2.y()*F(2,2), w2.x()*F(2,2)-F(0,2), 0.f,
            F(0,2)-w2.x()*F(2,2), F(1,2)-w2.y()*F(2,2), vc,
            0.f, 0.f, 1.f;

    Hp1 << 1.f, 0.f, 0.f,
            0.f, 1.f, 0.f,
            w1.x(), w1.y(), 1.f;
    Hp2 << 1.f, 0.f, 0.f,
            0.f, 1.f, 0.f,
            w2.x(), w2.y(), 1.f;

    Matrix3f inverseTransform1 = (Hr1 * Hp1).inverse();
    Matrix3f inverseTransform2 = (Hr2 * Hp2).inverse();

    // Pixel* v1;
    // Pixel* v2;

    //Output: try projection without opencv
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {

            Vector3f outputPixel = Vector3f((float)col - (float)width/2, (float)row - (float)height/2, 1.f); // shifted so (0,0) in middle

            auto affineTrans = [&](Vector3f& transform, int* lookup, Pixel* image, Pixel* rectified){
                if(0 <= transform[0] && transform[0] <= (float)width && 0 <= transform[1] && transform[1] <= (float)height) {
                    int newRow = (int) transform[1];
                    int newCol = (int) transform[0];
                    lookup[row * width + col] = newRow * width + newCol;

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

                    rectified[row * width + col] = Pixel((int)r, (int)g, (int)b, (int)a);

                }
            };

            Vector3f L = inverseTransform1 * outputPixel;
            L[0] += (float)width/2; // shift back so (0,0) top left
            L[1] += (float)height/2;

            affineTrans(L, m_leftImageLookup, m_leftImage, m_leftImageRectified);


            Vector3f R = inverseTransform2 * outputPixel;
            R[0] += (float)width/2;
            R[1] += (float)height/2;

            affineTrans(R, m_rightImageLookup, m_rightImage, m_rightImageRectified);
        }
    }

    //  cv::warpPerspective( leftImage->matrix(), v1->matrix(), Hp1, width*height );
    //  cv::warpPerspective( rightImage->matrix(), v2->matrix(), Hp2, width*height );

    //  setLeftImageRectified(v1);
    //  setRightImageRectified(v2);

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

Pixel *StereoImage::getLeftImageRectified() const
{
    return m_leftImageRectified;
}

Pixel *StereoImage::getRightImageRectified() const
{
    return m_rightImageRectified;
}

int *StereoImage::getLeftImageLookup() const
{
    return m_leftImageLookup;
}

int *StereoImage::getRightImageLookup() const
{
    return m_rightImageLookup;
}

float* StereoImage::getDepthImage() {
    return m_depthImage;
}

//SETTERS
void StereoImage::setLeftImageRectified(Pixel* value)
{
    m_leftImageRectified = value;
}
void StereoImage::setRightImageRectified(Pixel *value)
{
    m_rightImageRectified = value;
}


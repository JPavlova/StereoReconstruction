#include "stereoimage.h"
#include <random>
//#include <opencv4/opencv2/core.hpp>
//#include <opencv4/opencv2/highgui.hpp>
//#include <opencv4/opencv2/features2d.hpp>
//#include <opencv4/opencv2/xfeatures2d.hpp>
//#include <opencv4/opencv2/xfeatures2d/nonfree.hpp>



/**
 * @brief StereoImage::StereoImage
 * create StereoImage from left/right BYTE * arrays and camera sensor, save in Pixel* array for left/right images
 * @param leftImage : BYTE * array
 * @param rightImage : BYTE * array
 * @param sensor : CameraSensor * object
 */
StereoImage::StereoImage(BYTE * leftImage, BYTE * rightImage, CameraSensor * sensor) : sensor(sensor)
{
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
void StereoImage::rectify()
{
 int w = sensor -> getLeftImageHeight();
 int h = sensor->getLeftImageWidth();


 Pixel* il =  getLeftImage();
 Pixel* iR = getRightImage();

 /*Berechne Foundation Matrix F*/

 Matrix3f K1 = sensor->getLeftIntrinsics(); //linke Kamera
 Matrix3f K2 = sensor->getRightIntrinsics(); //rechte Kamera
 Matrix4f Rt = sensor->getRightExtrinsics();
 Matrix3f R;                                //Rotationsmatrix abgeleitet von der Extrinsic von der rechten der Kamera
 R << Rt(0,0), 0.f, 0.f,
        0.f, Rt(1,1) ,0.f,
         0.f, 0.f, Rt(2,2);
 Vector3f t;                                //Transformationsvector
 t << Rt(0,3), Rt(1,3), Rt(2,3);

 Matrix3f F;

 //Berechne die asymmetrische Matrix zur Kreuzproduktdarstellung [x]_x
 Vector3f tmpv = K1*R.transpose()*t;
 Matrix3f tmpm;
 tmpm << 0.f, -tmpv.z(), tmpv.y(),
         tmpv.z(), 0.f, -tmpv.x(),
         -tmpv.y(), tmpv.x(), 0.f;

 F = K2.inverse().transpose()*R*K1.transpose()*tmpm;

 /*Berechne Epipolare Linien*/
 Vector3f e1,e2;
 e1 = (-1.f)*K1.transpose()*t;
 e2 = K2*t;

 /*Berechne z*/
    /*Berechne A,B bzw A',B'*/
 Matrix3f A1,B1,A2,B2;
 Matrix3f pp, ppc;

 pp << ((w*h)/12)* w*w - 1 , 0.f, 0.f,
         0.f, ((w*h)/12)*h*h-1, 0.f,
         0.f, 0.f, 0.f;

 ppc << 1/4* powf(w-1,2), 1/4*(w-1)*(h-1), 0.f,
         1/4*(w-1)*(h-1), 1/4*(powf(h-1,2)), 0.f,
         0.f, 0.f, 0.f;

 Matrix3f esx1, esx2;
 esx1 << 0.f, -e1.z(), e1.y(),
         e1.z(), 0.f, -e1.x(),
         -e1.y(), e1.x(), 0.f;

 esx2 << 0.f, -e2.z(), e2.y(),
         e2.z(), 0.f, -e2.x(),
         -e2.y(), e2.x(), 0.f;

 A1 = esx1.transpose()*pp*esx1;
 B1 = esx1.transpose()*ppc*esx1;

 A2 = F.transpose()*pp*F;
 B2 = F.transpose()*ppc*F;

    /*A = D*D.traspose()*/

 Matrix3f D = decomposeMatrix(A1);

 /*y=Eigenvector of D.inv.trans*B*D.inv*/
 EigenSolver<Matrix3f> y1(D.inverse().transpose()*B1*D.inverse());

 D = decomposeMatrix(A2);
 EigenSolver<Matrix3f> y2(D.inverse().transpose()*B2*D.inverse());


 /*z = D.inv*y*/
 Vector3f z1,z2;
 z1 = D.inverse()*y1.eigenvectors().col(0);
 z1 = D.inverse()*y2.eigenvectors().col(0);

 Vector3f w1, w2;
 w1 = esx1*z1;
 w2 = F*z2;

 float vc = 0 - il->bottomLeftCorner(0,h).value(); //??

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
 Hp1 << 1.f, 0.f, 0.f,
         0.f, 1.f, 0.f,
         w2.x(), w2.y(), 1.f;

 //Output
 //m_leftImageRectified;
 //m_rightImageRectified;


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

std::vector<Feature> StereoImage::getLeftFeatures() const
{
    return m_leftFeatures;
}

std::vector<Feature> StereoImage::getRightFeatures() const
{
    return m_rightFeatures;
}

std::vector<std::pair<int, int> > StereoImage::getFeatureMatches() const
{
    return m_featureMatches;
}

float* StereoImage::getDepthImage() {
    return m_depthImage;
}


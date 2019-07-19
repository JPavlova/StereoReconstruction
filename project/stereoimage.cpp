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

#pragma omp parallel for
    for (int v = 0; v < sensor->getLeftImageHeight(); v++) {
        for (int u = 0; u < sensor->getLeftImageWidth(); u++) {

            int i = v * sensor->getLeftImageWidth() + u;
            float w = depthMap[i];

            if (w == MINF) {
                vertices[i].position = Vector4f(MINF, MINF, MINF, MINF);
                vertices[i].color = Pixel(0, 0, 0, 0);
            }
            else {
                Vector3f uvw = Vector3f(u*w, v*w, w);
                Vector3f xyz = leftIntrinsicsInv* uvw; // already in world coordinates, no need for extrinsics-mult.

                // color retrieval
                Pixel rgb = colorMap[i];

                vertices[i].position = Vector4f(xyz.x(), xyz.y(), xyz.z(), 1);
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

#pragma omp parallel for collapse(2)
    for (int row = 0; row < m_leftImageHeight; row++) {
        for (int col = 0; col < m_leftImageWidth; col++) {

            int i = row * m_leftImageWidth + col;
            Vector3f position, transformed_l, transformed_r;
            int x_left, y_left, x_right, y_right;

            position << col, row, 1;

            transformed_l = (S * H).inverse() * position;
            transformed_l /= transformed_l(2);
            transformed_r = (S * H_).inverse() * position;
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
        m_depthImageRectified[i] = log(baseline * focalLength / m_disparity[i]);
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

/*
Matrix3f skewSymmetricMatrix(Vector3f& vector) {
    Matrix3f m;
    m << 0.f, -vector.z(), vector.y(),
            vector.z(), 0.f, -vector(0),
            -vector.y(), vector(0), 0.f;
    return m;
}

Matrix3f decomposeMatrix(Matrix3f D)
{
    float sum; //Hilfsvariablen

    for(int i = 0; i<3; i++)
    {
        for(int j = 0; j<i; j++)
        {
            sum = D(i,j);
            for(int k = 0; k < j; k++)
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
    int width = sensor->getLeftImageWidth();
    int height = sensor->getLeftImageHeight();

    Pixel* leftImage = getLeftImage();
    Pixel* rightImage = getRightImage();

    // Berechne Foundation Matrix F

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

    // remark: other source F = K^(-T) * ssM(T) * R * K'^(-1)
//    F = K2.inverse().transpose()*R*K1.transpose()*tmpm;
    F = K1.inverse().transpose() * skewSymmetricMatrix(t) * R * K2.inverse();

    // Berechne Epipolare Linien
    Vector3f e1 = K1 * R.transpose() * t;
    Vector3f e2 = K2 * t;

    Vector3f a = F * e1;

    // Berechne z
    // Berechne A,B bzw A',B'
    Matrix3f A1,B1,A2,B2;
    Matrix3f PP, ppc;

    auto fsquare = [](int a){return (float)a*(float)a;};

    PP << (width*height)/12.f * fsquare(width) - 1.f , 0.f, 0.f,
          0.f, (width*height)/12.f * fsquare(height) - 1.f, 0.f,
          0.f, 0.f, 0.f;

    ppc << 0.25f*fsquare(width-1), 0.25f*(width-1)*(height-1), 0.f,
           0.25f*(width-1)*(height-1), 0.25f*fsquare(height-1), 0.f,
           0.f, 0.f, 0.f;

    Matrix3f esx1 = skewSymmetricMatrix(e1);

    A1 = esx1.transpose()*PP*esx1;
    B1 = esx1.transpose()*ppc*esx1;

    A2 = F.transpose()*PP*F;
    B2 = F.transpose()*ppc*F;

    / A = D*D.transpose()

    Matrix3f D = decomposeMatrix(A1);
    Matrix3f aaaaaaaa = D.transpose() * D;

    // y=Eigenvector of D.inv.trans*B*D.inv
    EigenSolver<Matrix3f> y(D.inverse().transpose()*B1*D.inverse());

    // z = D.inv*y
    Matrix3f vals = y.pseudoEigenvalueMatrix();
    Matrix3f eigs = y.pseudoEigenvectors();
    Vector3f z = D.inverse() * y.pseudoEigenvectors().col(0);

    Vector3f w1, w2;
    w1 = esx1*z;
    w2 = F*z;

    float vc = 0; // minimum v-coordinate of a pixel in any image, set to zero so far

    Matrix3f Hr1, Hr2, Hp1, Hp2;
    Hr1 << F(2,1)-w1.y()*F(2,2), w1.x()*F(2,2)-F(2,0), 0.f,
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

    std::cout << "z: \n" << z << std::endl;
    std::cout << "w1: \n" << w1 << std::endl;
    std::cout << "F: \n" << F << std::endl;
    std::cout << "Hp1: \n" << Hp1 << std::endl;
    std::cout << "Hr1: \n" << Hr1 << std::endl;

//    Matrix3f h2, h1;
//    h2 << 1, 0, 0, 0, 1, 0, -1, 0, 1;
//    h1 = R * t;
//    Matrix3f test = h2*h1;


    // Pixel* v1;
    // Pixel* v2;

    //Output: try projection without opencv
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {

            Vector3f outputPixel = Vector3f((float)col - (float)width/2, (float)row - (float)height/2, 1.f); // shifted so (0,0) in middle

            auto affineTrans = [&](Vector3f& transform, int* lookup, Pixel* image, std::optional<Pixel>* rectified){
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
*/

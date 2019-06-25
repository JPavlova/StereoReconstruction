#include "stereoimage.h"
#include <iostream>

StereoImage::StereoImage(Pixel *image_left, Pixel *image_right, CameraSensor *sensor) : image_left(image_left), image_right(image_right), sensor(sensor)
{
    image_depth = new float[sensor->getLeftImageWidth() * sensor->getLeftImageHeight()];
}

/**
 * @brief StereoImage::backproject_frame
 * @param vertices Output argument, array of vertices is filled in this method. HAS to be in the right dimensions!
 * @return
 */

bool StereoImage::backproject_frame(Vertex *vertices)
{
    Pixel *colorMap = image_left;
    float *depthMap = image_depth;

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

float* StereoImage::getDepthImage() {
    return image_depth;
}

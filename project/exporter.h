#ifndef EXPORTER_H
#define EXPORTER_H

#include "prerequisites.h"
#include <fstream>
#include <FreeImage.h>

int computeIndex(int x, int y, int width);
float distance(Vector4f u, Vector4f v);
bool triangleValid(Vector4f u, Vector4f v, Vector4f w);


float roundToDecimal(float number, int decimalPlace) {
    return std::round(std::pow(10, decimalPlace) * number) / std::pow(10, decimalPlace);
}

/**
 * @brief writeRGBImage
 * @param image         : image BYTE array, a format of RGBA is assumed, hence 4 BYTE per pixel.
 * @param width         : in pixel
 * @param height        : in pixel
 * @param filename
 * @return
 */
bool writeRGBImage(BYTE *image, int width, int height, const std::string& filename) {
    FIBITMAP *bitmap = FreeImage_Allocate(width, height, 24);
    RGBQUAD color;
    if (!bitmap) {
        exit(1);
        std::cerr << "could not allocate memory for rgb image" << std::endl;
    }

    int i;
    BYTE current;

    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            i = ((height - h) * width + w) * 4; // Considered RGBA format of input, weird indexing in order to not have a mirrored image.
            //std::cout << "width: " << w << ", height: " << h << std::endl;
            current = image[i];
            color.rgbRed = image[i];
            color.rgbGreen = image[i+1];
            color.rgbBlue = image[i+2];
            FreeImage_SetPixelColor(bitmap, w, h, &color);
        }
    }

    if (FreeImage_Save(FIF_PNG, bitmap, filename.c_str(), 0))
        std::cout << "RGB image successfully saved!" << std::endl;

    FreeImage_DeInitialise();

    return true;
}

bool writeDisparityImageRaw(int *depthImage, int width, int height, DEPTH_MODE mode, const std::string& filename) {
    // using freeimage ideally
    FreeImage_Initialise();
    FIBITMAP * bitmap = FreeImage_Allocate(width, height, 24);
    RGBQUAD color;

    if (!bitmap) {
        exit(1);
        std::cerr << "could not allocate memory for depth image" << std::endl;
    }
    // minimal, maximal depth values
    float minimum = INF;
    float maximum =MINF;
    float current;
    for (int i = 0; i < width * height; i++) {
        current = depthImage[i];
        if ((current != MINF) && (current != INF)){
            if (current > maximum) {
                maximum = current;
            }
            if (current < minimum) {
                minimum = current;
            }
        }
    }

    float factor = 0.0f;
    if ((maximum - minimum) > EPSILON) {
        factor = 1.0f / (maximum - minimum);
    }

    int i = 0;
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            i = (height - h) * width + w;
            current = depthImage[i];
            color.rgbRed = (BYTE) current * factor * 255;
            color.rgbGreen = (BYTE) current * factor * 255;
            color.rgbBlue = (BYTE) current * factor * 255;
            FreeImage_SetPixelColor(bitmap, w, h, &color);
        }
    }

    if (FreeImage_Save(FIF_PNG, bitmap, filename.c_str(), 0))
        std::cout << "Disparity image successfully saved!" << std::endl;

    FreeImage_DeInitialise();

    return true;
}

bool writeDepthImageRaw(float *depthImage, int width, int height, const std::string path) {
    FreeImage_Initialise();
    FIBITMAP * bitmap = FreeImage_Allocate(width, height, 24);
    RGBQUAD color;

    if (!bitmap) {
        exit(1);
        std::cerr << "could not allocate memory for depth image" << std::endl;
    }
    // minimal, maximal depth values
    float minimum = INF;
    float maximum = MINF;
    float avg = 0;
    float current;
    for (int i = 0; i < width * height; i++) {
        current = depthImage[i];
        if ((current != MINF) && (current != INF)){
            avg += current;
            if (current > maximum) {
                maximum = current;
            }
            if (current < minimum) {
                minimum = current;
            }
        }
    }
    avg /= (width * height);
    std::cout << "maximum: " << maximum << std::endl;
    std::cout << "minimum: " << minimum << std::endl;
    std::cout << "average: " << avg << std::endl;

    float factor = 0.0f;
    if ((maximum - minimum) > EPSILON) {
        factor = 1.0f / (maximum - minimum);
    }

    int i = 0;
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            i = (height - h) * width + w; // does this work now?
            current = depthImage[i];
            color.rgbRed = (BYTE) (current * factor * 255);
            color.rgbGreen = (BYTE) (current * factor * 255);
            color.rgbBlue = (BYTE) (current * factor * 255);
            FreeImage_SetPixelColor(bitmap, w, h, &color);
        }
    }

    if (FreeImage_Save(FIF_PNG, bitmap, path.c_str(), 0))
        std::cout << "Depth image successfully saved!" << std::endl;

    FreeImage_DeInitialise();

    return true;
}


/**
 * @brief writeDepthImage
 * @param depthImage
 * @param width
 * @param height
 * @param mode      Not implemented yet!
 * @param filename
 * @return
 */
bool writeDepthImage(float *depthImage, int width, int height, DEPTH_MODE mode, const std::string& filename, float max) {
    // using freeimage ideally
    FreeImage_Initialise();
    FIBITMAP * bitmap = FreeImage_Allocate(width, height, 24);

    if (!bitmap) {
        exit(1);
        std::cerr << "could not allocate memory for depth image" << std::endl;
    }

    float* logDepth = (float*) malloc(width * height * sizeof(float));

#pragma omp parallel for
    for(int i = 0; i < width * height; i++){
        logDepth[i] = log(depthImage[i]);
    }

    // minimal, maximal depth values
    float minimum = std::numeric_limits<float>::infinity();
    float maximum =-std::numeric_limits<float>::infinity();
    for (int i = 0; i < width * height; i++) {
        float current = logDepth[i];
        if ((current != MINF) && (current != INF)){
            if (current > maximum) {
                maximum = current;
            }
            if (current < minimum) {
                minimum = current;
            }
        }
    }
    std::cout << "maximum: " << maximum << std::endl;
    std::cout << "minimum: " << minimum << std::endl;
    if (minimum < 0)
        minimum = 0;
    if (maximum > max) {
        maximum = max;
    }

    float factor = 0.0f;
    if ((maximum - minimum) > 0.000001f) {
        factor = 1.0f / (maximum - minimum);
    }

    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {

            int i = (height - h) * width + w;

            float current;
            if(logDepth[i] > max){
                current = max;
            } else {
                current = logDepth[i] - minimum;
            }

            RGBQUAD color;
            color.rgbRed = (BYTE) current * factor * 255;
            color.rgbGreen = (BYTE) current * factor * 255;
            color.rgbBlue = (BYTE) current * factor * 255;
            FreeImage_SetPixelColor(bitmap, w, h, &color);
        }
    }

    if (FreeImage_Save(FIF_PNG, bitmap, filename.c_str(), 0))
        std::cout << "Depth image successfully saved!" << std::endl;

    FreeImage_DeInitialise();
    free(logDepth);

    return true;
}

// code mostly copied from my homework assignment
bool writeMesh(Vertex *vertices, unsigned int width, unsigned int height, const std::string& filename) {

    int nVertices = 0;
    int *meshIndices = new int[width * height];

    std::vector<Vector3i> faces;

    for(int i = 0; i < width * height; i++){
        if(vertices[i].position[0] != MINF){
            meshIndices[i] = nVertices;
            nVertices++;
        } else {
            meshIndices[i] = -1;
        }
    }

    for (int y = 0; y < height-1; y++) {

        for (int x = 0; x < width-1; x++) {

            int origin = computeIndex(x, y, width);
            int right = computeIndex(x + 1, y, width);
            int bottom = computeIndex(x, y + 1, width);

            if(triangleValid(vertices[origin].position, vertices[right].position, vertices[bottom].position)){
                faces.push_back(Vector3i(meshIndices[origin], meshIndices[bottom], meshIndices[right]));
            }
        }

        for(int x = 1; x < width; x++){

            int origin = computeIndex(x, y, width);
            int bottomLeft = computeIndex(x -1, y + 1, width);
            int bottom = computeIndex(x, y + 1, width);

            if(triangleValid(vertices[origin].position, vertices[bottomLeft].position, vertices[bottom].position)){
                faces.push_back(Vector3i(meshIndices[origin], meshIndices[bottomLeft], meshIndices[bottom]));
            }
        }
    }

    int nFaces = faces.size();

    // Write off file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    // write header
    outFile << "COFF" << std::endl;
    outFile << nVertices << " " << nFaces << " 0" << std::endl;

    for(int i = 0; i < width * height; i++){

        if(meshIndices[i] != -1) {

            for(int j = 0; j < 3; j++){
                outFile << vertices[i].position[j] << " ";
            }

            for(int j = 0; j < 4; j++){
                outFile << (int) (vertices[i].color[j]) << " ";
            }

            outFile << std::endl;
        }
    }

    for(Vector3i face:faces){
        outFile << "3 " << face[0] << " " << face[1] << " " << face[2] << std::endl;
    }

    // close file
    outFile.close();

    return true;
}

int computeIndex(int x, int y, int width){
    return y * width + x;
}

float distance(Vector4f u, Vector4f v){
    return (u-v).norm();
}

bool triangleValid(Vector4f u, Vector4f v, Vector4f w){
    float edgeThreshold = 1e3;
    return u[0] != MINF && v[0] != MINF && w[0] != MINF &&
            distance(u,v) < edgeThreshold && distance(u,w) < edgeThreshold && distance(v,w) < edgeThreshold;
}

#endif // EXPORTER_H

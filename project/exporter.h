#ifndef EXPORTER_H
#define EXPORTER_H

#include "prerequisites.h"
#include <fstream>
#include <FreeImage.h>

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
        std::cout << "Depth image successfully saved!" << std::endl;

    FreeImage_DeInitialise();

    return true;
}

bool writeDepthImageRaw(float *depthImage, int width, int height, DEPTH_MODE mode, const std::string& filename) {
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
    RGBQUAD color;

    if (!bitmap) {
        exit(1);
        std::cerr << "could not allocate memory for depth image" << std::endl;
    }
    // minimal, maximal depth values
    float minimum = std::numeric_limits<float>::infinity();
    float maximum =-std::numeric_limits<float>::infinity();
    for (int i = 0; i < width * height; i++) {
        float current = depthImage[i];
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

    int i = 0;
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            i = (height - h) * width + w;
            float current;
            if(depthImage[i] > max){
                current = max;
            } else {
                current = depthImage[i] - minimum;
            }
            color.rgbRed = (BYTE) current * factor * 255;
            color.rgbGreen = (BYTE) current * factor * 255;
            color.rgbBlue = (BYTE) current * factor * 255;
            FreeImage_SetPixelColor(bitmap, w, h, &color);
        }
    }

    if (FreeImage_Save(FIF_PNG, bitmap, filename.c_str(), 0))
        std::cout << "Depth image successfully saved!" << std::endl;

    FreeImage_DeInitialise();

    return true;
}

// code mostly copied from my homework assignment
bool writeMesh(Vertex *vertices, unsigned int width, unsigned int height, const std::string& filename) {
    float edgeThreshold = 0.01f; // 1cm

    unsigned int nVertices = 0;
    int *indices = new int[height * width];

    for (int v = 0; v < width * height; v++) {
        if (!(vertices[v].position.x() == MINF)) {
            indices[v] = nVertices;
            nVertices++;
        }
        else {
            indices[v] = -1; // should not be referenced, hence also a test
        }
    }

    unsigned nFaces = 0;

    int i = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            i = y * width + x;

            if ((vertices[i].position.x() != MINF) && (vertices[i + width].position.x() != MINF) && (vertices[i + 1].position.x() != MINF)) {
                if (((vertices[i].position - vertices[i + width].position).norm() <= edgeThreshold) &&
                    ((vertices[i].position - vertices[i + 1].position).norm() <= edgeThreshold) &&
                    ((vertices[i + 1].position - vertices[i + width].position).norm() <= edgeThreshold)) {
                    nFaces++;
                }
            }
            if ((vertices[i + 1].position.x() != MINF) && (vertices[i + width].position.x() != MINF) && (vertices[i + width + 1].position.x() != MINF)) {
                if (((vertices[i + 1].position - vertices[i + width].position).norm() <= edgeThreshold) &&
                    ((vertices[i + 1].position - vertices[i + width + 1].position).norm() <= edgeThreshold) &&
                    ((vertices[i + width].position - vertices[i + width + 1].position).norm() <= edgeThreshold)) {
                    nFaces++;
                }
            }
        }
    }

    // Write off file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    // write header
    outFile << "COFF" << std::endl;
    outFile << nVertices << " " << nFaces << " 0" << std::endl;

    // save vertices
    for (int v = 0; v < width * height; v++) {
        if (!(vertices[v].position.x() == MINF)) {
            outFile << roundToDecimal(vertices[v].position[0], 3) << " " << roundToDecimal(vertices[v].position[1], 3) << " " << roundToDecimal(vertices[v].position[2], 3) << " "
                    << (int) vertices[v].color[0] << " " << (int) vertices[v].color[1] << " " << (int) vertices[v].color[2] << " " << 255 << std::endl;
        }
    }

    // save faces
    i = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            i = y * width + x;
            if ((vertices[i].position.x() != MINF) && (vertices[i + width].position.x() != MINF) && (vertices[i + 1].position.x() != MINF)) {
                if (((vertices[i].position - vertices[i + width].position).norm() <= edgeThreshold) &&
                    ((vertices[i].position - vertices[i + 1].position).norm() <= edgeThreshold) &&
                    ((vertices[i + 1].position - vertices[i + width].position).norm() <= edgeThreshold)) {
                    outFile << 3 << " " << indices[i] << " " << indices[i + width] << " " << indices[i + 1] << std::endl;
                }
            }
            if ((vertices[i + 1].position.x() != MINF) && (vertices[i + width].position.x() != MINF) && (vertices[i + width + 1].position.x() != MINF)) {
                if (((vertices[i + 1].position - vertices[i + width].position).norm() <= edgeThreshold) &&
                    ((vertices[i + 1].position - vertices[i + width + 1].position).norm() <= edgeThreshold) &&
                    ((vertices[i + width].position - vertices[i + width + 1].position).norm() <= edgeThreshold)) {
                    outFile << 3 << " " << indices[i + 1] << " " << indices[i + width] << " " << indices[i + width + 1] << std::endl;
                }
            }
        }
    }

    // close file
    outFile.close();

    return true;
}

#endif // EXPORTER_H

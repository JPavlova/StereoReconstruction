#include <iostream>
#include <vector>
#include <random>

#include "stereoimage.h"
#include "prerequisites.h"
#include "exporter.h"
#include <FreeImage.h>
#include "patchmatch.h"
#include "matcher.h"

#define PATCH_SIZE 5

// MAIN FILE TO RUN PIPELINE

/**
 * 1. READ IN IMAGES
 *      - convert .png format into array
 *      - read in ground truth, store in array with same indices
 *      -> would propose a "StereoImage" class containing left, right, depth information in arrays
 *      - Input: Path to database
 *      - Output: StereoImage object
 *
 * 2. RECTIFY IMAGES
 *      - rectification algorithm by paper
 *      - Input: StereoImage object
 *      - Output: StereoImage rectified (inherited class?)
 *
 * 3. PATCHMATCH ALGORITHM
 *      - maybe start with simple Block matching algorithm and expand later?
 *      - randomized approach, thus needs few iterations?
 *      -> epipolar geometry at end or in an intermediate step between 3/4
 *      - Input: StereoImage rectified
 *      - Output: Array of Depth values -> Could also be added to StereoImage object
 *
 *      --> at this step comparison to ground truth possible
 *
 * 4. OUTPUT REPRESENTATION
 *      - represent depth values in HSV / greyvalue depthmap
 *      -> optional: create PointCloud mesh
 *      - Input: StereoImage rectified + Array of depth values (easier in one object)
 *      - Output: Image like format, either to save as .png again or stream?
 *
 * **/

int main(int argc, char *argv[])
{
    // Read in data
//    std::string dataDir = "../project/data/Recycle-perfect/Recycle-perfect";
    std::string dataDir = "../project/data/blender";
    CameraSensor sensor;

    if(!sensor.Init(dataDir)) {
        std::cout << "Failed to initialize sensor!\n Check data directory, path and image name regex!" << std::endl;
        return -1;
    }

    // Sensor loop over all frames
    while(sensor.ProcessNextFrame()) {

        // create StereoImage from current sensor frames
        StereoImage testImage(&sensor);
        int width = testImage.getLeftImageWidth();
        int height = testImage.getLeftImageHeight();

        // Rectify
        testImage.rectify();
        writeRGBImage((BYTE *) testImage.getLeftImageRectifiedUnoptional(), width, height, "./rect_l.png");
        writeRGBImage((BYTE *) testImage.getRightImageRectifiedUnoptional(), width, height, "./rect_r.png");

        int iterations = 5;
        float alpha = 0.8;
        int blockSize = 5;
        int searchWindow = width/4;
        Matcher m(&testImage, blockSize, searchWindow, alpha);
        for (int i = 0; i < 5; ++i) {
            int bSize = 2 * i + 3;
            std::string sbSize = std::to_string(bSize);

            m.setPatchSize(bSize);

            if(bSize > 4) {
                // RUN OPENCV
                std::cout << "+++++++++++++++" << std::endl;
                std::cout << "STARTING OPENCV PATCHSIZE " << bSize << std::endl;
                std::cout << "+++++++++++++++" << std::endl;
                m.reset();
                m.runOpenCVMatch();
                writeDisparityImageRaw(m.getDisparityMap(), width, height, DEPTH_MODE::GRAY, "./opencv_disparity" + sbSize + ".png");
                writeDepthImageRaw(m.getDepthMap(), width, height, "./opencv_depth" + sbSize + ".png");
                writeRawDepthFile("./opencv_depth_values" + sbSize + ".txt", m.getDepthMap(), width, height);
                writeRawDisparityFile("./opencv_disparity_values" + sbSize + ".txt", m.getDisparityMap(), width, height);
            }
            // RUN PATCHMATCH
            std::cout << "+++++++++++++++" << std::endl;
            std::cout << "STARTING PATCHMATCH PATCHSIZE " << bSize << std::endl;
            std::cout << "+++++++++++++++" << std::endl;
            m.reset();
            m.runPatchMatch(iterations);
            writeDisparityImageRaw(m.getDisparityMap(), width, height, DEPTH_MODE::GRAY, "./patchmatch_disparity" + sbSize + ".png");
            writeDepthImageRaw(m.getDepthMap(), width, height, "./patchmatch_depth" + sbSize + ".png");
            writeRawDepthFile("./patchmatch_depth_values" + sbSize + ".txt", m.getDepthMap(), width, height);
            writeRawDisparityFile("./patchmatch_disparity_values" + sbSize + ".txt", m.getDisparityMap(), width, height);

            // RUN BLOCKMATCH
            std::cout << "+++++++++++++++" << std::endl;
            std::cout << "STARTING BLOCKMATCH PATCHSIZE " << bSize << std::endl;
            std::cout << "+++++++++++++++" << std::endl;
            m.reset();
            m.runBlockMatch();
            writeDisparityImageRaw(m.getDisparityMap(), width, height, DEPTH_MODE::GRAY, "./blockmatch_disparity" + sbSize + ".png");
            writeDepthImageRaw(m.getDepthMap(), width, height, "./blockmatch_depth" + sbSize + ".png");
            writeRawDepthFile("./blockmatch_depth_values" + sbSize + ".txt", m.getDepthMap(), width, height);
            writeRawDisparityFile("./blockmatch_disparity_values" + sbSize + ".txt", m.getDisparityMap(), width, height);

        }

        // point cloud/ backprojection
        /*
        Vertex *vertices = new Vertex[width * height];
        testImage.backproject_frame(vertices);
        writeMesh(vertices, width, height, "./pointcloud.off");
        */
    }

    return 0;
}

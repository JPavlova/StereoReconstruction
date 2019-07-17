#include <iostream>
#include <vector>
#include <random>

#include "stereoimage.h"
#include "prerequisites.h"
#include "exporter.h"
#include <FreeImage.h>
#include "patchmatch.h"
#include "blockmatch.h"

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
    //std::string dataDir = "../project/data/Recycle-perfect/Recycle-perfect";
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
//        writeRGBImage((BYTE *) testImage.getLeftImageRectifiedUnoptional(), width, height, "./rect_l.png");
//        writeRGBImage((BYTE *) testImage.getRightImageRectifiedUnoptional(), width, height, "./rect_r.png");

        // Patchmatch
//        PatchMatch patchMatch(&testImage,width,height,PATCH_SIZE);
//        patchMatch.computeDisparity();
//        writeDepthImage(testImage.getDisparity(), width, height, DEPTH_MODE::GRAY, "./disparity.png", 640.f);
//        testImage.disparityToDepth();
//        testImage.derectifyDepthMap();
//        writeDepthImage(testImage.getRectifiedDepthImage(), width, height, DEPTH_MODE::GRAY, "./depth.png", 10000.f);

        // Blockmatch
        // . CURRENTLY ONLY WORK WITH NON-RECTIFIED IMAGES, FOR WHATEVER REASONS
        // . IN BLOCKMATCH, THE LEFT AND RIGHT IMAGES ARE SWITCHED BECAUSE I THINK THE MAIN CODE SWITCHES LEFT AND RIGHT IMAGES ALSO, WE HAVE TO CHANGE THAT.
        int blockSize = 5;
        int searchWindow = 128;
        BlockMatch bm(&testImage, blockSize, searchWindow);
        bm.run();
        writeDisparityImageRaw(bm.getDisparityMap(), width, height, DEPTH_MODE::GRAY, "./blockmatch_disparity.png");
        writeDepthImageRaw(bm.getDepthMap(), width, height, "./blockmatch_depth.png");

        // point cloud/ backprojection
        /*
        Vertex *vertices = new Vertex[width * height];
        testImage.backproject_frame(vertices);
        writeMesh(vertices, width, height, "./pointcloud.off");
        */
    }

    return 0;
}

#include <iostream>
#include <vector>

#include "stereoimage.h"
#include "prerequisites.h"
#include "exporter.h"
#include <FreeImage.h>

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
    std::string dataDir = "../project/data/Recycle-perfect/Recycle-perfect";
    CameraSensor sensor;

    if(!sensor.Init(dataDir)) {
        std::cout << "Failed to initialize sensor!\n Check data directory, path and image name regex!" << std::endl;
        return -1;
    }

    // Sensor loop over all frames
    while(sensor.ProcessNextFrame()) {

        // Create StereoImage from current sensor frames
        StereoImage testImage(sensor.getLeftFrame(), sensor.getRightFrame(), &sensor);


        // -- Rectify
        // -- Patchmatch


        // part 4:

        // point cloud/ backprojection

        int width = testImage.getLeftImageWidth();
        int height = testImage.getLeftImageHeight();

        Vertex vertices[width * height];
        testImage.backproject_frame(vertices);

        // export point cloud to .off
        writeMesh(vertices, width, height, "./pointcloud.off");
        writeDepthImage(testImage.getDepthImage(), width, height, DEPTH_MODE::GRAY, "./depth.png");

    }



    return 0;
}

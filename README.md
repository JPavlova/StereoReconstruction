# StereoReconstruction
Project for "3D Scanning and Motioncapture"

# TODO

- after rectification, the actual distances between matching pixels can not be taken by comparing the rectified images, but we have to un-rectify the matching indices so we can compute actual disparity
-> rework patchmatch to store indices (?) of matching pixel from left to right, then use a lookup table computed in stereo-image to un-rectify
--> only then apply epipolar geometry


### EASY DATASET
http://vision.middlebury.edu/stereo/data/scenes2014/
### DOWNLOAD LINK
http://vision.middlebury.edu/stereo/data/scenes2014/zip/Recycle-perfect.zip

###############################################

### KITTI STEREO VISION DATASET : Stereo Evaluation 2012
#### 194 training image pairs and 195 test image pairs -> png format

### DOWNLOAD LINK (Sonst neuen download-link auf der Webseite requesten)

https://s3.eu-central-1.amazonaws.com/avg-kitti/data_stereo_flow.zip

### ABOUT THE DATASET

http://www.cvlibs.net/datasets/kitti/eval_stereo_flow.php?benchmark=stereo

### CITATION

@INPROCEEDINGS{Geiger2012CVPR,
  author = {Andreas Geiger and Philip Lenz and Raquel Urtasun},
  title = {Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite},
  booktitle = {Conference on Computer Vision and Pattern	Recognition (CVPR)},
  year = {2012}
}

import numpy as np
import matplotlib.pyplot as plt


def trya():
    threshold = 50000
    a = np.arange(0, 100000)
    b = 255*np.exp(-2/threshold*a)
    print(b[threshold])
    plt.plot(a, b)
    plt.vlines(threshold, ymin=0, ymax=255, colors="red")
    plt.show()

def disparity(threshold, filename, title, savef=False):

    min_ = 0
    # opencv 3500, pm/bm 40000
    max_ = threshold

    with open(filename, "r") as file:
        xml = file.read().splitlines()

    disparity_map = []
    for element in xml:
        temp = []
        # print(element)
        for el in element.split(sep="\t"):
            # print(el)
            if el in ["\n", ""]:
                continue
            if el != "-inf" and el != "inf":
                temp.append(abs(float(el)))
            else:
                temp.append(np.nan)
        disparity_map.append(temp)

    disparity_map = np.array(disparity_map)

    histogram_array = disparity_map[~np.isnan(disparity_map)].reshape(-1)
    #print(histogram_array, histogram_array.shape)
    #print(np.min(histogram_array))
    #print(np.max(histogram_array))
    #plt.hist(histogram_array, bins=np.arange(min_, max_, (max_ - min_) / 100))
    #plt.show()


    img = disparity_map
    #print(disparity_map[~np.isnan(disparity_map)])
    img[np.isnan(disparity_map)] = 0
    img[img < min_] = 0
    img[img > max_] = 0

    factor = 255 / (max_ - min_)

    img = factor * img
    temp = np.zeros([img.shape[0], img.shape[1], 3])
    temp[:,:,0] = img
    temp[:,:,1] = img
    temp[:,:,2] = img
    rgbimg = np.array(temp, dtype=np.uint8)

    #print(rgbimg)
    #plt.hist(img, bins=np.arange(0, 255, 255/10))
    #plt.show()
    plt.axis("off")
    plt.title(title)
    plt.imshow(rgbimg)
    if savef: plt.savefig(fname=str(title) + ".png")
    plt.show()

def depth(threshold, filename, title, savef=False):

    min_ = 0
    # opencv 3500, pm/bm 40000
    max_ = threshold

    with open(filename, "r") as file:
        xml = file.read().splitlines()

    depth_values = []
    for element in xml:
        temp = []
        # print(element)
        for el in element.split(sep="\t"):
            # print(el)
            if el in ["\n", ""]:
                continue
            if el != "-inf" and el != "inf":
                temp.append(float(el))
            else:
                temp.append(np.nan)
        depth_values.append(temp)

    depth_values = np.array(depth_values)

    histogram_array = depth_values[~np.isnan(depth_values)].reshape(-1)
    #print(histogram_array, histogram_array.shape)
    #print(np.min(histogram_array))
    #print(np.max(histogram_array))
    #plt.hist(histogram_array, bins=np.arange(min_, max_, (max_ - min_) / 100))
    #plt.show()


    img = depth_values
    #print(depth_values[~np.isnan(depth_values)])
    img[np.isnan(depth_values)] = 0
    img[img < min_] = 0
    img[img > max_] = 0

    factor = 255 / (max_ - min_)

    img = factor * img
    temp = np.zeros([img.shape[0], img.shape[1], 3])
    temp[:,:,0] = img
    temp[:,:,1] = img
    temp[:,:,2] = img
    rgbimg = np.array(temp, dtype=np.uint8)

    #print(rgbimg)
    #plt.hist(img, bins=np.arange(0, 255, 255/10))
    #plt.show()
    plt.axis("off")
    plt.title(title)
    plt.imshow(rgbimg)
    if savef: plt.savefig(fname=str(title) + ".png")
    plt.show()


if __name__ == "__main__":

    bsize = 3
    savef = False
    path="../../build-project-Desktop-Debug/classroom_1deg/"

    # need to manually adjust scales so images look similar!
    # opencv is for whatever reason always smaller by some factor 10-20!

    #depth(6000, path + f"opencv_depth_values{bsize}.txt", f"OPENCV DEPTHMAP BLOCKSIZE {bsize}", savef)
    #depth(100000, path + f"patchmatch_depth_values{bsize}.txt", f"PATCHMATCH DEPTHMAP BLOCKSIZE {bsize}", savef)
    #depth(100000, path + f"blockmatch_depth_values{bsize}.txt", f"BLOCKMATCH DEPTHMAP BLOCKSIZE {bsize}", savef)

    #disparity(1400, path + f"opencv_disparity_values{bsize}.txt", f"OPENCV DISPARITY MAP BLOCKSIZE {bsize}", savef)
    disparity(100, path + f"patchmatch_disparity_values{bsize}.txt", f"PATCHMATCH DISPARITY MAP BLOCKSIZE {bsize}", savef)
    disparity(100, path + f"blockmatch_disparity_values{bsize}.txt", f"BLOCKMATCH DISPARITY MAP BLOCKSIZE {bsize}", savef)
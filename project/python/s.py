import numpy as np
import matplotlib.pyplot as plt
import os, fnmatch


def trya():
    threshold = 50000
    a = np.arange(0, 100000)
    b = 255*np.exp(-2/threshold*a)
    print(b[threshold])
    plt.plot(a, b)
    plt.vlines(threshold, ymin=0, ymax=255, colors="red")
    plt.show()

def read_file(filename, invalid):
    with open(filename, "r") as file:
        xml = file.read().splitlines()

    map = []
    for element in xml:
        temp = []
        # print(element)
        for el in element.split(sep="\t"):
            # print(el)
            if el in ["\n", ""]:
                continue
            if el in invalid:
                temp.append(np.nan)
            else:
                temp.append(abs(float(el)))
        map.append(temp)

    return np.array(map)

def disparity(threshold, filename, title, savef=False):

    min_ = 0
    # opencv 3500, pm/bm 40000
    max_ = threshold

    disparity_map = read_file(filename, ["-2147483648", "2147483648"])

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

    depth_values = read_file(filename, ["-inf", "inf"])

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

def h_fun_disp(hist_array, min_, max_, title, save=False, filename=None):
    hist, bins = np.histogram(hist_array, bins=np.arange(min_, max_, (max_ - min_) / 50))
    hist_normalized = hist / hist_array.shape[0] * 100
    centers = (bins[:-1] + bins[1:]) / 2
    bin_size = bins[1]-bins[0]

    plt.bar(centers, hist_normalized, width=bin_size)

    highest = np.argmax(hist_normalized)
    second = highest + 1 if highest == 0 else highest - 1
    plt.hlines(hist_normalized[highest], centers[second], centers[highest], color='orange')
    plt.text(centers[second], hist_normalized[highest], '{:.2f}'.format(hist_normalized[highest]),
             ha="left" if highest == 0 else "right", va='center', color='orange')

    plt.ylim(0, 100)
    plt.title(title, y=1.03)
    plt.xlabel(f"Disparity differences (in pixels), bin size: {max(int(bin_size), 1)}")
    plt.ylabel("Percent pixels contained in bin")
    if(save): plt.savefig(filename + f"_{int(min_)}_{int(max_)}.png")
    plt.show()

def h_fun_depth(hist_array, min_, max_, title, save=False, filename=None):
    hist, bins = np.histogram(hist_array, bins=np.arange(min_, max_, (max_ - min_) / 50))
    hist_normalized = hist / hist_array.shape[0] * 100
    centers = (bins[:-1] + bins[1:]) / 2
    bin_size = bins[1]-bins[0]

    plt.bar(centers, hist_normalized, width=bin_size)

    highest = np.argmax(hist_normalized)
    second = highest + 1 if highest==0 else highest - 1
    plt.hlines(hist_normalized[highest], centers[second], centers[highest], color='orange')
    plt.text(centers[second], hist_normalized[highest], '{:.2f}'.format(hist_normalized[highest]), ha="left" if highest==0 else "right", va='center',color='orange')

    plt.ylim(0, 100)
    plt.title(title, y=1.03)
    plt.xlabel(f"Depth differences (in mm), bin size: {max(int(bin_size), 1)}")
    plt.ylabel("Percent pixels contained in bin")
    if(save): plt.savefig(filename + f"_{int(min_)}_{int(max_)}.png")
    plt.show()

def compare(path, bsize, type_, title, save, filename):
    histogram_ = False
    invalid = ["-inf", "inf"] if type_ == "depth" else ["-2147483648", "2147483648"]
    for root, dir, files in os.walk(path):
        if f"patchmatch_{type_}_values{bsize}.txt" in files and f"blockmatch_{type_}_values{bsize}.txt" in files:
            print("Reading in...\t" + root)
            depth_vals_1 = read_file(root + f"/patchmatch_{type_}_values{bsize}.txt", invalid)
            depth_vals_2 = read_file(root + f"/blockmatch_{type_}_values{bsize}.txt", invalid)
            print("Depths", depth_vals_1, depth_vals_2)

            overlap = ((~np.isnan(depth_vals_1)).astype(int) + (~np.isnan(depth_vals_2)).astype(int)) > 0
            print("OVERLAP", overlap)
            print("VALUES:", depth_vals_1[overlap], depth_vals_2[overlap])
            diff = depth_vals_1[overlap == True] - depth_vals_2[overlap == True]
            diff = diff[~np.isnan(diff)]
            diff = np.abs(diff[np.abs(diff) < 10000]) if type_ == "depth" else diff
            print("Individual values: " + str(diff.reshape(-1).shape[0]))

            if not(histogram_):
                histogram_array = diff.reshape(-1)
                histogram_ = True
            else:
                if diff.shape[0] > 0:
                    histogram_array = np.concatenate([histogram_array, diff.reshape(-1)], axis=0)

    print(histogram_array.shape[0])

    min_ = np.min(histogram_array)
    max_ = np.max(histogram_array)
    print(np.sum(np.isnan(histogram_array)))

    if type_ == "depth":
        h_fun_depth(histogram_array, min_, max_, title, save, filename)
        h_fun_depth(histogram_array, 0, 2000, title, save, filename)
        h_fun_depth(histogram_array, 0, 100, title, save, filename)

    else:
        h_fun_disp(histogram_array, min_, max_, title, save, filename)
        h_fun_disp(histogram_array, -50, 50, title, save, filename)
        h_fun_disp(histogram_array, -25, 25, title, save, filename)

if __name__ == "__main__":

    bsize = 11
    savef = True
    path="../../build-project-Desktop-Debug/"


    #compare(path, bsize, "depth",
    #        f"Histogram: Depth Map Comparison between\n Patchmatch and Blockmatch (Blocksize: {bsize})",
    #        savef, f"HISTOGRAM_Depth_COMPARISON_{bsize}")

    compare(path, bsize, "disparity", f"Histogram: Disparity Map Comparison between\n Patchmatch and Blockmatch (Blocksize: {bsize})",
               savef, f"HISTOGRAM_DISPARITY_COMPARISON_{bsize}")

    # need to manually adjust scales so images look similar!
    # opencv is for whatever reason always smaller by some factor 10-20!

    #depth(6000, path + f"opencv_depth_values{bsize}.txt", f"OPENCV DEPTHMAP BLOCKSIZE {bsize}", savef)
    #depth(100000, path + f"patchmatch_depth_values{bsize}.txt", f"PATCHMATCH DEPTHMAP BLOCKSIZE {bsize}", savef)
    #depth(100000, path + f"blockmatch_depth_values{bsize}.txt", f"BLOCKMATCH DEPTHMAP BLOCKSIZE {bsize}", savef)

    #disparity(1400, path + f"opencv_disparity_values{bsize}.txt", f"OPENCV DISPARITY MAP BLOCKSIZE {bsize}", savef)
    #disparity(100, path + f"patchmatch_disparity_values{bsize}.txt", f"PATCHMATCH DISPARITY MAP BLOCKSIZE {bsize}", savef)
    #disparity(100, path + f"blockmatch_disparity_values{bsize}.txt", f"BLOCKMATCH DISPARITY MAP BLOCKSIZE {bsize}", savef)
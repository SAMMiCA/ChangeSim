from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

# Define Depth Range
MIN_DEPTH = 1 # Unit: meter
MAX_DEPTH = 50 # Unit: meter

def depth2disp(normalized_dep,min_depth=MIN_DEPTH,max_depth=MAX_DEPTH):
    assert normalized_dep.max() <= 1.0, 'depth array should be normalized into the range of 0~1'
    # min disparity = 0 when depth > MAX_DEPTH
    # max disparity = 1 when depth < MIN_DEPTH

    # Convert Depth to true-scale
    truescale_dep = max_depth * normalized_dep.astype('float32') # unit: meter
    clipped_truescale_dep = np.clip(truescale_dep,a_min=min_depth,a_max=max_depth)
    clipped_normalized_dep = clipped_truescale_dep/max_depth # range MIN_DEPTH/MAX_DEPTH ~ 1
    disp = 1/(clipped_normalized_dep) # range 1 ~ MAX_DEPTH/MIN_DEPTH
    norm_disp = (MIN_DEPTH/MAX_DEPTH) * disp # range MIN_DEPTH/MAX_DEPTH ~ 1

    return norm_disp

# Load PNG Depth image (Max depth=50M when val=255, Min depth=0M when val=0)
normalized_dep = Image.open('../../Dataset/Mapping/StorageHouse/Room_1/Seq_0/depth/326.png')
normalized_dep = np.array(normalized_dep) # (height,width)=(480,640), Val range 0~255 grayscale array (dtype=uint8)

normalized_dep = normalized_dep.astype('float32')/255 # Val range 0~1
# truescale_dep = 50 * normalized_dep.astype('float32') # unit: meter
# clipped_truescale_dep = np.clip(truescale_dep,a_min=MIN_DEPTH,a_max=MAX_DEPTH)
# disp = 1/(clipped_truescale_dep) #
norm_disp = depth2disp(normalized_dep,MIN_DEPTH,MAX_DEPTH)
print(norm_disp.max(),norm_disp.min())
plt.subplot(121)
plt.imshow(normalized_dep)
plt.subplot(122)
plt.imshow(norm_disp)
plt.show()
# import cv2
# cv2.imshow('fsad',norm_disp)
#
# cv2.waitKey(0)
# cv2.destroyAllWindows()
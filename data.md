### RGB Image

The color images are stored as 640x480 8-bit RGB images in PNG format.

* Load the image using OpenCV: 
```
import cv2
img = cv2.imread(FILENAME)
cv2.imshow('img', img)
```

* Load the image using Pillow:
```
from PIL import Image
img = Image.open(FILENAME)
img.show()
```

### Camera intrinsics 
```
fx = 320.0  # focal length x
fy = 320.0  # focal length y
cx = 320.0  # optical center x
cy = 240.0  # optical center y

fov = 90 deg # field of view    ??????????????

width = 640
height = 480
```

### Depth image

The depth images are stored as 640x480 8-bit RGB images in PNG format. 

The unit of the depth value is meter. 

* Load the depth image:
```
import numpy as np
img = cv2.imread(FILENAME)

# change to disparity image
disparity = 80.0 / depth
```

### Semantic segmentation image

The semantic segmentation images are stored as 640x480 8-bit RGB images in PNG format.

[More details](https://github.com/microsoft/AirSim/blob/master/docs/image_apis.md#segmentation)

* Load the semantic segmentation image
```
import numpy as np
depth = np.load(FILENAME)
```

### Change segmentation image

The semantic segmentation images are stored as 640x480 8-bit RGB images in PNG format.


* Load the change segmentation image
```
import numpy as np
depth = np.load(FILENAME)
```

### Pose file

The camera pose file is a text file containing the translation and orientation of the camera in a fixed coordinate frame. Note that our automatic evaluation tool expects both the ground truth trajectory and the estimated trajectory to be in this format. 

* Each line in the text file contains a single pose.

* The number of lines/poses is the same as the number of image frames in that trajectory. 

* The format of each line is '**tx ty tz qx qy qz qw**'. 

* **tx ty tz** (3 floats) give the position of the optical center of the color camera with respect to the world origin in the world frame.

* **qx qy qz qw** (4 floats) give the orientation of the optical center of the color camera in the form of a unit quaternion with respect to the world frame. 

* The camera motion is defined in the NED frame. That is to say, the x-axis is pointing to the camera's forward, the y-axis is pointing to the camera's right, the z-axis is pointing to the camera's downward. 

* Load the pose file:
```
import numpy as np
flow = np.loadtxt(FILENAME)
```

### trajectory file

All poses mentioned above are merged into a trajectory.txt file for each sequence.

### rtabmap.db file

All data from the mapping stage. We can view using rtabmap-databaseViewer.

### cloud_map.ply file

3D reconstruction map from RTABMAP.

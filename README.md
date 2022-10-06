# Real-time 360° 3D detection and tracking for Mobile Robot

##Welcome to the easy inference repository! 

Real-time multi-modal multi-pedestrian detection and tracking framework.
###Overview
The main goal of this repository is to provide a clean, simple and short way of setting up inference pipelines for 2D (and 3D) visual detection and tracking. 
Our proposed framework is depicted in fig.1 using  five RGB-D cameras.

 <!-- Fig.1. A schematic the perception framework. un selection, Blocks with dashed border will be implemented in future version -->

This framework uses a 2D object detector that processes the RGB images from five RGB cameras and outputs 2D bounding boxes.
We employed the state-of-the-art YOLOv7 architecture, as our
2D object detector. YOLOv7 is the powerful object detection algorithm and focuses on fast object detection in real-time applications.


<!-- Real-Time 2D detection  -->

Since this framework relies on tracking 3D bounding, and RGB images
do not contain depth information, so we use the Depth image to translate these 2D bounding boxes to 3D.
Obtaining accurate depth information is essential in order to translate 2D object detections into 3D. We utilize a standard feature of the RGB-D cameras that aligns the depth images with their corresponding RGB images. This alignment allows us to directly extract the region of interest (RoI) in the depth image by examining the bounding boxes provided by the 2D object detector

###Mount Hardware : 
5X intel realsense d455 
1X lidar velodyne VLP-16
1 X  3.1 USB Hub 5 ports 

 <!-- Fig.2. A LiDAR and Cameras mount  -->
Sensor Calibration:
In this version the sensor calibration was made manually. 

Ready to Hack!
The interfaces to camera drivers are abstracted away as python generators. A simple inference pipeline for a webcam based inference pipeline looks as follows:
from easy_inference.providers.webcam import Webcam

```python
provider = Webcam(source=0)

for frame in provider:

  # run my detection
```

See the examples directory for some yolov7 pipelines.


Current Version  1.0.0

Video 

https://youtu.be/kFF_nrtYIOM





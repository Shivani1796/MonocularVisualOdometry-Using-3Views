## Odometry using ORB, BRIEF Descriptors algorithm to show feature matching and detection in an image.

**Calibration Parameter T265 CAMERA:** 

```
Focal Length:  618.8560
Center or Principle point: (427, 394)
```
**MOIL LIBRARY PARAMETERS**

```
	md->Config("car", 3, 3,
			427,394, 1, //center
			848, 800, 1.68, // resolution
			0, 0, -24.964, 38.2, -16.956, 183.42 //calibration
	);
```

## HOW TO RUN ON ECLIPSE

Please follow the below url to understand how to clone poject on Eclipse

https://bit.ly/31ph49X

**NOTE: Remember to checkout the branch which you want to run**
	
**DESCRIPTION:**

Argument 1: Ground Truth Data (/home/shivani/Documents/mydataset/D5_17-04/poses/D5.txt)

Argument 2: Image sequence  (/home/shivani/Documents/fisheye-test-images/D5/%d.png)

Argument 3: Zoom Factor (3.4)

Argument 4: Front View angle 

Argument 5: Left View angle

Argument 6: Right View angle

 **NOTE**
- Change img_path and pose_path to correct image sequences and pose file paths according to your system.
- Ensure focal length and principal point information is correct .
---------------------------------------------------------------------------------------------------------------------------

## HOW TO RUN ON ECLIPSE

Please follow the below url to understand how to clone poject on Eclipse

https://bit.ly/31ph49X

---------------------------------------------------------------------------------------------------------------------------

## ORB Feature Points
<p align="center">
  <img src="https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/blob/ORB-Matching/img/orb.png">
</p>

## All matching points and good matched points from source fisheye image to *Front view image.*
<p align="center">
  <img src="https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/blob/ORB-Matching/img/front.png">
</p>

## All matching points and good matched points from source fisheye image to *Left view image.*
<p align="center">
  <img src="https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/blob/ORB-Matching/img/left.png">
</p>

## All matching points and good matched points from source fisheye image to *Right view image.*
<p align="center">
  <img src="https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/blob/ORB-Matching/img/right.png">
</p>

-----------------------------------------------------------------------------------------------------------------------------
## Scale Estimation

<p align="center">
  <img src="https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/blob/ORB-Matching/img/indoorscale.png">
</p>

> **Credit:** *Shivani Baldwa, Raghav Jethliya, under the guidance of Professor Chuang-Jan-Chang from Ming Chi University of Technology, New Taipei City*

## Odometry using MOIL LIBRARY genrating 3 Views 

We used approach designed in MOIL-Lab ,  “MOIL SDK- MCUT Omnidirectional Imaging Lab” to extricate the fisheye picture into 6 diverse view as: Front, Left, Right respectively. By utilizing this technique, we made a trajectory of 3 view by  the arrangement of fisheye pictures as a source document for odometry.

It means we use source fisheye images and integrate that with MOIL Library and generate 6-view odometry.
Before we start this project, we have referred the AVI SINGH blogs and github https://github.com/avisingh599/mono-vo. So, this can be a referenced code before starting monocular visual odometry.

**Watch this video for more understanding !!!! (*Click on below image*)**

[![Watch this video](https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/blob/master/images/Monocular-Visual-Odomertry-Using-Three-Views.jpg)](https://www.youtube.com/embed/dGPUH05CyqU)

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
## Download the MOIL dataset: https://bit.ly/3iq759W

If you want to know how to create dataset refer: https://github.com/Shivani1796/How-to-create-MOIL-Dataset 

## HOW TO RUN ON TERMINAL

Step 1.Open Terminal 

Step 2.
```
git clone https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views.git
```
*MOIL SDK DEPENDENCIES*

```
sudo apt update
sudo apt upgrade
sudo apt install build-essential cmake pkg-config
sudo apt install libjpeg-dev libpng-dev libtiff-dev
sudo apt install software-properties-common
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install libjasper1 libjasper-dev
sudo apt update
sudo apt install libgtk-3-dev
sudo apt install libatlas-base-dev gfortran
sudo apt install libopencv-dev python-opencv
```

Step 3.Compile command
 ```
     cd MonocularVisualOdometry-Using-3Views
     mkdir build 
     cd build 
     cmake .. 
     make 
 ```
Step 4.Run command
```
	./vo /home/shivani/Documents/mydataset/D5_17-04/poses/D5.txt /home/shivani/Documents/fisheye-test-images/D5/%d.png 3.4 0,0 -10,0 10,0
	
```
**DESCRIPTION:**
```
Argument 1: Ground Truth Data (/home/shivani/Documents/mydataset/D5_17-04/poses/D5.txt)

Argument 2: Image sequence  (/home/shivani/Documents/fisheye-test-images/D5/%d.png)

Argument 3: Zoom Factor (3.4)

Argument 4: Front View angle 

Argument 5: Left View angle

Argument 6: Right View angle
```

 **NOTE**
- Change img_path and pose_path to correct image sequences and pose file paths according to your system.
- Ensure focal length and principal point information is correct .
---------------------------------------------------------------------------------------------------------------------------
## HOW TO RUN ON ECLIPSE

Please follow the below url to understand how to clone poject on Eclipse

https://bit.ly/3eLsatG
---------------------------------------------------------------------------------------------------------------------------

## Odometry using Round Path

<p align="center">
  <img src="https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/blob/master/images/result1%20.png">
</p>


## Odometry using Long path
<p align="center">
  <img src="https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/blob/master/images/result2.png">
</p>


## Odometry using Straight Path
<p align="center">
  <img src="https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/blob/master/images/result3.png">
</p>
---------------------------------------------------------------------------------------------------------------------------------------------------------------


*To use ORB matching for feature points switch branch- "ORB-Matching" or https://github.com/Shivani1796/MonocularVisualOdometry-Using-3Views/tree/ORB-Matching*

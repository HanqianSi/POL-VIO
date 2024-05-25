# Point-Line Visual-Inertial Odometry with Optimized Line Feature

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**

Ubuntu 20.04. ROS Noetic, please google it.

1.2. **Dependency**

Eigen 3.3.4 + OpenCV 4+ Cere-solver: [Ceres Installation](http://ceres-solver.org/installation.html), remember to **sudo make install**.

1.3. **Pretrained model**

Download [pretrained model](https://drive.google.com/file/d/1VP0M7O-6ng461y_VWznMFuqKw2efH-aD/view?usp=sharing) the pretrained model and unzip the model to **"hawp/src/outputs/hawp"**


## 2. Build POL-VIO on ROS
Clone the repository and catkin_make (# note that you will create a new workspace named *catkin_polvio*):
```
mkdir -p ~/catkin_polvio/src    
cd ~/catkin_polvio/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH            
git clone https://github.com/HanqianSi/POL-VIO.git
catkin_make
source devel/setup.bash
```

## 3. Run on EuRoC dataset

Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).

run in the ~/catkin_polvio/
```
roslaunch polvio_estimator euroc_fix_extrinsic.launch
```

Now you should be able to run POL-VIO in the ROS RViZ. 

## 3. Run on corridor dataset

Download [corridor Dataset](https://drive.google.com/drive/folders/15YIKDlhFGrk1igapBLTYtZpDltXbmumm?usp=drive_link)

run in the ~/catkin_polvio/
```
roslaunch polvio_estimator corridor_fix_extrinsic.launch
```

**Note that**: Different CPU and GPU maybe yield different results. Therefore, we suggest you test or compare methods on your machine by yourself. 


## 4. Related Papers

**Point-Line Visual-Inertial Odometry with Optimized Line Feature**.


This paper is developed based on PL-VIO [1], VINS-Mono [2], PL-VINS[3] and EPLF-VINS[4].
```
[1] PL-VIO: Tightly-coupled monocular visual-inertial odometry using point and line features

[2] VINS-mono: A robust and versatile monocular visual-inertial state estimator

[3] PL-VINS: real-time monocular visual-inertial SLAM with point and line features

[4] EPLF-VINS: real-time monocular visual-inertial SLAM with efficient point-line flow features
```
*If you find aforementioned works helpful for your research, please cite them.*




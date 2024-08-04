# VIW-Fusion
## An visual-inertial-wheel fusion odometry
VIW-Fusion is an optimization-based viusla-inertial-wheel fusion odometry, which is developed as a part of my master thesis.
**Features:**
- multiple sensors support (stereo cameras+IMU+wheel / mono camera+IMU+wheel)
- wheel enhanced visual-inertial initialization
- online spatial calibration (transformation between camera, IMU and wheel)
- online wheel intrinsic calibration
- online temporal calibration (time offset between camera, IMU and wheel)
- plane constraint

## Updates
- Feb 25, 2023 - Released [my thesis in chinese](https://github.com/TouchDeeper/VIW-Fusion/blob/master/Thesis%20-%20TingdaZhuang%20-%20Chinese%20-%20Research%20on%20Multi-sensor%20Fusion%20Localization%20of%20Mobile%20Robot%20Based%20on%20ROS.pdf)

### Performance in the scene with challenge light
We tested Mono VIWO in scenes with drastic changes in light, and the parameters between different scenes remained unchanged. The video is below, if you can't access youtube, please try [bilibili](https://www.bilibili.com/video/BV1zg411N75H?share_source=copy_web):
[![visual-inertial-wheel fusion odometry](https://res.cloudinary.com/marcomontalbano/image/upload/v1638587991/video_to_markdown/images/youtube--HXNaLTJWea4-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/HXNaLTJWea4 "visual-inertial-wheel fusion odometry")
We compare the VIWO with Mono VIO. The trajectory estimated by Mono VIO is completely different from the real trajectory, while the trajectory estimated by Mono VIWO is in good agreement with the real trajectory.

[//]: # (<img src = "/home/td/slam/vins_fusion_ws/src/VINS-Fusion/support_files/image/mono_compare.png" width = 54% height = 30% />)

<img src = "https://github.com/TouchDeeper/VIW-Fusion/blob/master/support_files/image/mono_compare.png" width = 54% height = 30% />

We align the trajectory with the entrance door and exit door to further compare VIWO and Stereo VIO, the trajectory estimated by Mono VIWO is also more reasonable.

[//]: # (<img src = "/home/td/slam/vins_fusion_ws/src/VINS-Fusion/support_files/image/performance_compare_light.png" width = 54% height = 30% />)
[//]: # (<img src = "/home/td/slam/vins_fusion_ws/src/VINS-Fusion/support_files/image/performance_compare_dark.png" width = 54% height = 30% />)

<img src = "https://github.com/TouchDeeper/VIW-Fusion/blob/master/support_files/image/performance_compare_light.png" width = 54% height = 30% />
<img src = "https://github.com/TouchDeeper/VIW-Fusion/blob/master/support_files/image/performance_compare_dark.png" width = 54% height = 30% />

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

Version: 1.14.0

### 1.3 **Sophus**
```asm
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout a0fe89a323e20c42d3cecb590937eb7a06b8343a
mkdir build && cd ./build
cmake ..
make -j4
sudo make install
```

## 2. Build VIW-Fusion
Clone the repository and catkin_make:
```asm
cd ~/catkin_ws/src
git clone https://github.com/TouchDeeper/VIW-Fusion.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. Example

Download dataset [here](https://drive.google.com/drive/folders/1m2msbo3DRGhtINtDE47v-1blyJc0RK0E?usp=sharing).
```asm
roslaunch vins vins_rviz.launch
rosrun vins viwo_node ~/catkin_ws/src/VIW-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml 
(optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VIW-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
rosbag play YOUR_DATASET_FOLDER/ridgeback_dark.bag
```

## 4. Citation
If you use this package for your research, a footnote with the link to this repository is appreciated: `github.com/TouchDeeper/VIW-Fusion`, or for citation with BibTeX:
```
@misc{ztd2021viwo,
  title={VIW-Fusion: visual-inertial-wheel fusion odometry.},
  author={Tingda Zhuang},
  howpublished={\url{https://github.com/TouchDeeper/VIW-Fusion}},
  year={2021}
}
```

## Acknowledgment
I need to thank HKUST Aerial Robotics Group led by Prof. Shaojie Shen for their outstanding work [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion). VIW-Fusion is developed based on VINS-Fusion.

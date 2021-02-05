# ICP-Localization-Using-Lidar-Points
### This is the public repo for the project of Self-Driving Car at 2020 Fall.
National Chiao Tung University

### Main Objective
The goal of the project is to estimate the localization of self-driving car on the HD map and keep tracking its position while driving. The main localization algorithm is ICP and the sensors used in this project are GPS and lidar.
There are three indepedent experiments included in this project. Each experiment uses different dataset.

### Methods
For the first experiment, I used the dataset from ITRI. The progress are shown in the flow chart below.![](https://i.imgur.com/QYqu7MB.jpg)

The second and third experiments used the dataset from nuScenes. The progress are shown in the flow chart below.![](https://i.imgur.com/dH6VdEf.jpg)

### Result
The following videos are the localization performances of the first, second and third experiments in order. The colorful point cloud is the HD map, and the white point clouds are the data from lidar sensor.

ITRI:
![](https://i.imgur.com/Y0j74xp.gif)



nuScenes 1:
![](https://i.imgur.com/DrEMj5Z.gif)




nuScenes 2:
![](https://i.imgur.com/zmUrgz2.gif)


### Using this Code to Reproduce the Results
1. Before running the programs, make sure you download the dataset.
2. Place the data in the folder "test", and the all nuScenes map segmentations should be place in the "nuscenes_maps" folder under "test" folder.
3. Change the director path in four scripts in "src" folder to your path.
(mine is /home/weichin/catkin/src/localization/test)
4.Simply run the following three commands to do the localizations for three experimens respectively:
```
roslaunch localization itri_icp.launch
roslaunch localization nuScenes_icp.launch
roslaunch localization nuScenes2_icp.launch
```


---
### Dependencies
* Ubuntu 18.04
* ROS melodic
* PCL <https://pointclouds.org/>
* 
### Dataset
All data including maps could be found at <https://drive.google.com/drive/folders/1BNC1kmKMfWrkpLs97KrPVZdFnliMZgSq>

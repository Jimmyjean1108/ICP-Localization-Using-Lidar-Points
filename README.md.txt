This package is used for the localization competition in the course "Self-Driving Cars" in 2020 Fall.

The file "itri_map.pcd" is placed in the "test" folder.
The all files of nuScenes map segmentations are placed in the "nuscenes_maps" folder under "test" folder.
All bag files are placed in the "test" folder.

Simply run the three commnds below to do the localizations for three bags.
roslaunch localization_competition itri_icp.launch
roslaunch localization_competition nuScenes_icp.launch
roslaunch localization_competition nuScenes2_icp.launch
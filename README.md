# 3D Perception

The goal of this projec was to make an object detection using RGB_D camera that used Point Cloud Library (PCL) to filter the camera image using some algorithms, and then used the SVM model to build a classifier to identify target objects in the tabletop. Eventually, PR2-Robot picks and places objects in corresponding drop boxes.


![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

***Capture the images with different poses***

![training_objects](/images/training_objects.gif)


### This sensor `RGB_D camera` is a bit noisy, much like real sensors and may have some dust.
So, I have to filter and extract features to recognition them using some algorithms. See `pdf` for an extended discussion .

![Filter noisies](/images/filtered.png)

**Point Cloud after Statistical Outlier Filtering Algorithm**

**Next, Perform RandomSample (RANSAC) plane Algorithm fitting to Segment the table in the scene**

![RANSAC filter](/images/RANSAN.png)


**Then Used the Euclidean Clustering Algorithm to separate the objects into distinct clusters, thus completing the segmentation process.**

![cluster](/images/cluster.png)


**Finally Made Object Recognition using `SVM Model`.**

![object recognition](/images/object_recognition.png)


# Project Setup
For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Perception-Project.git
```
#### Note: If you have the Kinematics Pick and Place project in the same ROS Workspace as this project, please remove the 'gazebo_grasp_plugin' directory from the `RoboND-Perception-Project/` directory otherwise ignore this note. 

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```
Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/models:$GAZEBO_MODEL_PATH
```

If you haven’t already, following line can be added to your .bashrc to auto-source all new terminals
```
source ~/catkin_ws/devel/setup.bash
```

To run the demo:
```sh
$ cd ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts
$ chmod u+x pr2_safe_spawner.sh
$ ./pr2_safe_spawner.sh
```




Once Gazebo is up and running, make sure you see following in the gazebo world:
- Robot

- Table arrangement

- Three target objects on the table

- Dropboxes on either sides of the robot


#### If any of these items are missing, please report as an issue on [the waffle board](https://waffle.io/udacity/robotics-nanodegree-issues).

In your RViz window, you should see the robot and a partial collision map displayed:

![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)


Close all active terminal windows using **ctrl+c** before restarting the demo.

You can launch the project scenario like this:
```sh
$ roslaunch pr2_robot pick_place_project.launch
```

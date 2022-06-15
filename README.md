# MPGR

This repository gives an insight into the basics of Motion Planning using ROS. This project is performed using [Sahayak](https://github.com/IvLabs/Sahayak-v3).

## Python Requirements
``` shell
pip install -r requirement.txt
```

## Required ROS Packages
``` shell
xargs sudo apt install < ros_requirements.txt
```

## Results

### Sahayak Teleoperation
For teleoperation of Sahayak run the below command in terminal:

```shell
roslaunch sahayak_control teleop.launch
```

<p align="center">
  <img src= "https://user-images.githubusercontent.com/83055325/160807783-ba8521bf-5bdc-4819-b19d-33134c7983d4.gif">
</p>

### A* Search
To run A* search on a given map:

![A_Implementation](https://user-images.githubusercontent.com/83055325/160805615-a1b44fbf-f9b4-468e-9c4d-5b85fbababde.png) ![Astar](https://user-images.githubusercontent.com/83055325/160805640-bc946511-0748-44d0-88cb-9accef683e88.png)

### State Estimation using EKF

![image](https://user-images.githubusercontent.com/83055325/160806997-63f84ee2-a393-423b-a014-ad0fac8f8aff.png)  ![image](https://user-images.githubusercontent.com/83055325/160807088-46e4a05f-a57c-47b7-bb1e-c287b9cf73ac.png)


### Sahayak Control

To move the bot on the path planned by global planner, run the following:

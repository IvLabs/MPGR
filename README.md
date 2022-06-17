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

A* search is run on a given map to find the shortest path. RDP algorithm is used to remove the redundant nodes in the generated path. Through these nodes a smooth B-spline curve is fitted.

#### Without RDP
<p align="center">
<img src = "https://user-images.githubusercontent.com/83055325/160805615-a1b44fbf-f9b4-468e-9c4d-5b85fbababde.png" height = "400">
<!-- ![A_Implementation](https://user-images.githubusercontent.com/83055325/160805615-a1b44fbf-f9b4-468e-9c4d-5b85fbababde.png) 
![Astar](https://user-images.githubusercontent.com/83055325/160805640-bc946511-0748-44d0-88cb-9accef683e88.png) -->
<img src = "https://user-images.githubusercontent.com/83055325/160805640-bc946511-0748-44d0-88cb-9accef683e88.png" height = "400">
</p>

#### With RDP and B-spline
<p align="center">
  <img src="https://user-images.githubusercontent.com/83055325/174044900-e694205d-b143-4b3e-bf1e-d4a7a4e9ed8a.jpeg" height="350">
</p>

#### Comparing Both Paths

Green : Without RDP <br/>
Blue : RDP with $\epsilon$=0.1 <br/>
Red : RDP with $\epsilon$=0.3
<p align="center">
  <img src="https://user-images.githubusercontent.com/83055325/174045625-be2a9ddd-48a5-42bb-9f6f-5804a4f41f82.jpeg" height="450">
</p>

To run A* search on a given map:
```
```

### State Estimation using EKF
<p align="center">
<img src="https://user-images.githubusercontent.com/83055325/174037578-180c6e23-ce76-4a94-b7b1-9ea314098c9b.png">
</p>

<!-- <p align="center">
![image](https://user-images.githubusercontent.com/83055325/160806997-63f84ee2-a393-423b-a014-ad0fac8f8aff.png) 
![image](https://user-images.githubusercontent.com/83055325/160807088-46e4a05f-a57c-47b7-bb1e-c287b9cf73ac.png)
</p> -->


### Sahayak Control.

To move the bot on the path planned by global planner, run the following:
```
```
<p align="center">
  <img src= "https://user-images.githubusercontent.com/83055325/174037147-62fd6fdd-e50b-4b02-90b9-195f63e0f32b.gif" >
</p>

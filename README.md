# Sawyer's Travels
Team Members: Evan Chien, Miaoding Dai, Ana Pervan, William Wang, Michael Wiznitzer

Northwestern University ME 495: Embedded Systems in Robotics (Fall 2017, Jarvis Schultz)


## Introduction
###  Objective
The goal of this project is for Sawyer to navigate a ball through a maze using vision feedback.

### Maze Design and Fabrication
A Sawyer [gripper mount](https://github.com/anapervan/Sawyers-Travels/blob/master/CAD/stl/Gripper%20Mount.stl), [spacers](https://github.com/anapervan/Sawyers-Travels/blob/master/CAD/stl/Spacer.stl), and a [maze base](https://github.com/anapervan/Sawyers-Travels/blob/master/CAD/stl/Maze%20Base.stl) were designed using [OnShape](https://www.onshape.com/). These were laser cut and glued together so that they could be mounted on the Sawyer, and any maze with the same corner mounting holes could be attached or removed using screws.

Next, a [maze](https://github.com/anapervan/Sawyers-Travels/blob/master/CAD/stl/Maze%201.stl) was designed, cut, and painted blue. The maze base was painted black, so that the camera could clearly differentiate between the walls and the background. Once the maze is attached, they yellow ball is placed in the starting position, and a green sticker is added to indicate the desired ending position.

### Computer Vision
tripod mount, calibration, layers, OpenCV ...

### Path Planning
grid based planning, RRT, update/feedback ...

### Robot Control
inverse kinematics/motion planning, p (or pid?) control, joint states topic ...


## Implementation
### Launch
launch files and dependencies

### Services
services we create?

### Nodes
#### Computer Vision Node
Subscribed Topics:

Published Topics:

#### Path Planning Node
Subscribed Topics:

Published Topics:

#### Robot Control Node
Subscribed Topics:

Published Topics:

#### Loop
feedback loop / rqt graph

## Conclusion
could use any maze
### Further Improvements

### Video

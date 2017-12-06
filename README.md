# Sawyer's Travels
**Team Members: Evan Chien, Miaoding Dai, Ana Pervan, William Wang, Michael Wiznitzer**

Northwestern University ME 495: Embedded Systems in Robotics (Fall 2017)


## Introduction
####  Objective
The goal of this project is for Sawyer to navigate a ball through a maze using vision feedback.

#### Maze Design and Fabrication
A Sawyer [gripper mount](https://github.com/anapervan/Sawyers-Travels/blob/master/CAD/stl/Gripper%20Mount.stl), [spacers](https://github.com/anapervan/Sawyers-Travels/blob/master/CAD/stl/Spacer.stl), and a [maze base](https://github.com/anapervan/Sawyers-Travels/blob/master/CAD/stl/Maze%20Base.stl) were designed using [OnShape](https://www.onshape.com/). These were laser cut and glued together so that they could be mounted on the Sawyer, and any maze with the same corner mounting holes could be attached or removed using screws.

Next, a [maze](https://github.com/anapervan/Sawyers-Travels/blob/master/CAD/stl/Maze%201.stl) was designed, cut, and painted blue. The maze base was painted black, so that the camera could clearly differentiate between the walls and the background. Once the maze was mounted, a white ball was placed in the starting position and a green sticker was placed at the desired ending position.

#### Computer Vision
Sawyer's head camera is oriented at an angle that would not work for this project, so a separate tripod and a 720p webcam were used to record the maze and ball. A node identified the blue outline of the maze by transforming the corners that it detected onto a 30cm square (the dimensions of the maze were known ahead of time). So no matter the orientation of the maze, a top-down view could always be extrapolated. Next, the position of the rest of the blue walls in the maze were identified, as well as the position of the white ball and the green "final position" sticker. This information was all passed along to the Path Planning node.

#### Path Planning
A grid-based planning algorithm was used to solve the maze. The maze was divided into a 7x7 grid, so that the sampling process moved very quickly. A global plan, from the initial ball location to the desired final location at the green sticker, was constructed and then a local plan, from the current position to the next grid point, was sent to the Robot Control node. The local path was updated with new ball position information at approximately 20Hz.

#### Robot Control
Sawyer was programmed to begin at the same starting position each time, holding the maze level and low, so that the camera could see it. After moving to the initial position, only two joints were used (joint 4 and joint 5) to move the ball in the positive and negative x and y directions. The error between the current ball position and the next desired position (from the local path) was detected, and then two PID controllers (one for joint position and one for joint velocity) were used to get the ball to the next point in the local path. The final values for the PID controllers were:
- joint position: k<sub>p</sub> = 0.01, k<sub>i</sub> = 0.005, k<sub>d</sub> = 0.0.
- joint velocity: k<sub>p</sub> = 0.001, k<sub>i</sub> = 0.0, k<sub>d</sub> = 0.0.

These values were found experimentally, while testing Sawyer with the ball and maze. Having both controllers allowed Sawyer to swing the ball into turns where there was no "backboard" (a wall for the ball to stop at before turning).


## Implementation
#### Launch
launch files and dependencies

#### Services
[`ctr_pos.srv`]()

#### Nodes
##### Computer Vision Node
[`get_start.py`]()

[`get_dest.py`]()

[`ball_pos_pub.py`]()

Subscribed Topics:

Published Topics:

##### Path Planning Node
[`.py`]()

Subscribed Topics:

Published Topics:

##### Robot Control Node
[`labyrinth_manipulation.py`]()

Subscribed Topics:

Published Topics:

##### Loop
feedback loop / rqt graph

## Conclusion
#### Further Improvements
We had already started building a new, circular maze and revising the path planning algorithm to use a grid with polar coordinates, but in the end we did not have time to alter the computer vision code to identify the circular shape or to experiment the dynamics of the circular maze. But solving the square maze for different starting and ending points lead to a lot of different possible maze paths -- almost as if we had made many mazes.


#### Video

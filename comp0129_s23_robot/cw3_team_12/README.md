# cw3_team_12
## Authors: 
### Heshui He*, Yicheng Gong*, Hongzhan Yu*

*These authors contributed to the work equllly and should be regarded as co-first authors.

## Pre-Requisites
```bash
> git clone git clone https://github.com/COMP0129-UCL/comp0129_s23_robot.git --recurse-
submodules
> cd comp0129_s23_robot
> git pull
> cd comp0129_s23_robot/src
> git submodule add https://github.com/COMP0129-UCL/cw3_world_spawner.git
```

## Installation
```bash
> git clone https://github.com/TroubleNaming/cw3_team_12.git
> cd cw3_team_12
> git pull
```

## Run CW3
### Run Gazebo and rviz
```bash
> cd comp0129_s23_robot
> catkin build
> source devel/setup.bash
> roslaunch roslaunch cw3_team_12 run_solution.launch
```

### Run Task 1
Open a new terminal.
```bash
> source devel/setup.bash
> rosservice call /task 1
```

### Run Task 2
Open a new terminal.
```bash
> source devel/setup.bash
> rosservice call /task 2
```

### Run Task 3
Open a new terminal.
```bash
> source devel/setup.bash
> rosservice call /task 3
```

## Time Cost
### Task 1
36 hours
### Task 2
24 hours
### Task 3
24 hours

## Code Information
### Task 1
1. Initial with adding collision of ground, clearing vector of object.
2. Recieve the coordinate and type of basket and object, instantiate object and basket class then push thenm to vectors.
3. Compute the object orientation, and align the gripper with the object.
4. Add collisions by looping through content of object and basket.
5. Perform pick & place function
6. Clear all collision and home the arm. 

### Task 2
1. Loop through the given location list.
2. For each location, judge the type by checking the centre of object.

### Task 3
1. scan 6 positions in the scene, generate a point cloud map.
2. cluster the map to recognize all objects using 2 different algorithm.
3. count the object and store them in vector.
4. perform pick and place than follows the task1 procedure.

## Contribution
### Yicheng Gong (33.33%)
Participated in the method design and code implementation of Task 1 and Task 2, and organized the text files.
### Hongzhan Yu (33.33%)
Participated in the method design and code implementation of Task 2 and Task 3, and reorganized part of the code format and part of code comments.
### Heshui He (33.33%)
Participated in the method design and code implementation of Task 1 and Task 3, and reorganized part of the code format and part of code comments.

## License
LICENSE: MIT.  See LICENSE

DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2023 COMP0129 Team 12 except where specified

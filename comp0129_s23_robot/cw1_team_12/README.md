## Authors: 
### Heshui He*, Yicheng Gong*, Hongzhan Yu*

*These authors contributed to the work equllly and should be regarded as co-first authors.

## Pre-Requisites
```bash
> git clone https://github.com/COMP0129-UCL/comp0129_s23_robot.git --recurse-submodules
> cd comp0129_s23_robot
> git pull
> cd comp0129_s23_robot/src
> git submodule add https://github.com/COMP0129-UCL/cw1_world_spawner.git
```

## Installation
```bash
> git clone https://github.com/TroubleNaming/cw1_team_12.git
> cd cw1_team_12
> git pull
```

## Run CW3
### Run Gazebo and rviz
```bash
> cd comp0129_s23_robot
> catkin build
> source devel/setup.bash
> roslaunch roslaunch cw1_team_12 run_solution.launch
```
#### note above all, because we add collisions by read files using: 
#### "package://cw3_team_12/models/nought_30mm_40H.STL", please treat the cw3_team_12 as a package
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
24 hours
### Task 2
24 hours
### Task 3
48 hours

## Code Information
### Task 1
1. Initial with adding collision of ground, clearing vector of basket and cube.
2. Recieve the coordinate of basket and cube, instantiate cube and basket class then push thenm to vectors.
3. Add collisions by looping through content of basketand cube vectors.
4. perform pick & place function
5. clear all collision and home the arm. 

### Task 2
1. loop through the given location list.
2. for each location, judge the color by calculating the number of specific color.

### Task 3
1. perform color judgement, store each point in 3 different color vector.
2. For each color vector, cluster individual object and judge the type(basket and cube) using euclidien distance.
3. store these locations in cube and basket vectors.
4. loop through the vectors and reuse the functions in task 1 to perform pick and place.

## Contribution
### Heshui He (33.33%)
Participated in the method design and code implementation of Task 1 and Task 2, and reorganized part of the code format and part of code comments.
### Yicheng Gong (33.33%)
Participated in the method design and code implementation of Task 2 and Task 3, and organized the text files.
### Hongzhan Yu (33.33%)
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



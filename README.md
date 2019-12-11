## Sensor Placement

This repository serves for simulation environment of the optimization algorithms w.r.t the sensor placement. 

In one word, this optimization problem could be defined as how to use the minimal number or 
cheapest sensor sets to ensure a best safety criteria. In the current version, we define the safety situation, when
As


![](sensor_class/out/scenario/sketch_problem.png | width=100)
## Sensor Type.
1. Laser
    - Position, angle resolution, frame rate, detection range (distance and angle range)
    - Detect worker's position when the worker moves in its' range  
 <!---
 ![laser](sensor_class/out/gif/lidar.gif)
 -->

2. Fence
    - Length, width, position
    - Reflect the human motion's direction
 <!---
    ![Fence](sensor_class/out/gif/fence.gif)
-->

3. Mate
    - Position, length, width
    - Detect worker's position when the worker directly on its' surface  
 <!---
    ![Mate](sensor_class/out/gif/mat.gif)
-->

## Working Configuration
1. One robot in the centre
2. Worker walks randomly in the environment.

## How to run
1. run *simulator_with_factory* for randomly generate the sensors in working area.
2. run *simulator* only for demo

## Next step
2. Define the Fence coverage area
3. Optimize the human motion
4. Combined with cost function

## Project notes:
1. Use @staticmethod for a clean class structure
2. Use 'raise NotImplementedError' in the Base class.
3. Use Factory method for multiple object.
4. **dict or *list to get single respondent value
5. Use *.gitignore* for a clean git. Tricky for git:
    - git rm --cached for modified and tracked files
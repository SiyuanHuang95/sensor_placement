## Sensor Placement

This repos is for the optimization algorithms of the sensor placesment.

## Sensor Type
1. Laser
    - Position, angle resolution, frame rate, detection range (distance and angle range)
    - detect worker's position when in its' range  
2. Fence
    - Length, width, position
    - reflect the human motion's direction
3. Mate
    - Position, length, width
    - detect worker's position when directly on its' surface  

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
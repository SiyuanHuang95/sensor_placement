## Sensor Placement

This repos is for the optimization algorithms of the sensor placesment.

## Sensor Type
1. Laser
2. Fence
3. Mate

## Working Configuration
1. One robot in the centre
2. Worker walks randomly in the environment.

## How to run
1. run *simulator_with_factory* for randomly generate the sensors in working area.
2. run *simulator* only for demo

## Next step
1. No overlapping sensor generation 
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
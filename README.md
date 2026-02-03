# Dead Reckoning
## Usage
1. Create a new ros2 workspace with a src folder.
2. Clone this repository into the src folder.
3. Run `colcon build` to build both nodes.
4. Source the setup file from the install folder `source install/setup.bash`
5. Commands to run each node are below.
   ```
   ros2 run dead_reckoning dead_reckoner_node
   ```
   ```
   ros2 run dead_reckoning imu_estimator_node
   ```

## Results
Neither path was completley accurate to the ground truth. The dead reckoning (in purple) implentation provided a choppy yet approximate representation of the path. The drift and choppiness of this implementation of this method was likely caused by the linear velocity model used which does not account for the curvature caused by acceleration. The IMU implementation did not follow the path at all. It immediately began drifing which only compounded as bag went on. This is due to the inverse 2nd order model used. Slight errors in the acceleration are compounded in the posistion due to the double integration. This causes the extreme overshoots seen on the path. 
Additional work needs to be done to correct the IMU implementation. However, with a more functional implementation, both implmentations could be fused together. This can be done with naive ways like weighted averageing. More robust implementations can correct for noise using covariance but this data would have to be calculated from the sensors used in the robot.

![result](https://github.com/MisReid/EE5531_DeadReckoning/blob/master/imgs/Screenshot%202026-02-02%20152154.png)

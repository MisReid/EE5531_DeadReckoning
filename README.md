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

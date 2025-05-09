# Optical flow-backed velocity controller for Aqua2 AUV

This ROS2 package provides a velocity controller for AUVs that estimates velocity from optical flow. This velocity is affected by environmental disturbances such as current, turbulence etc. 
Once this velocity is estimated, the controller adjusts the robot's intended velocity accordingly. Note that this controller only adjusts the xy planar velocity and does not affect vertical velocity (i.e., depth/altitude).

## Usage

1. Clone the repo in ros2 workspace.   
   ```sh
   cd ~/ros2_ws/src
   git clone https://github.com:aduvai22/aqua_velocity_control.git
   ```
2. Build the package
   ```sh
   cd ~/ros2_ws
   colcon build --packages-select aqua_velocity_control

3. Run the velocity estimator node.
   ```sh
   ros2 run aqua_velocity_control velocity_estimator
   ```
   
 This node subscribe to down-facing camera image and calculates optical flow from two consecutive frames. A 2D velocity vector (v_x, v_y) is calculated for each pair of frames.
 Optionally, if altitude data is available (such as via DVL), the node calculates and publishes metric velocity (in m/s). Other parameter required for metric velocity calculation are:
 - Camera intrinsic (fx, fy)
 - Time difference between two frames (frame rate) 

### TODO
- Add an active velocity controller.
- Investigate how current influences lateral drag. Mitigate it with sway control.

Group 3

To run:
```
ros2 launch comp3 comp3.launch.py 
```
after launching the world

We modified the world files in the multi_robot_challenge_23 repo to add:
```
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
  <ros>
    <namespace>/gazebo</namespace>
    <argument>model_states:=model_states_demo</argument>
  </ros>
  <update_rate>1.0</update_rate>
</plugin>
```
Because the service was missing for scoring.

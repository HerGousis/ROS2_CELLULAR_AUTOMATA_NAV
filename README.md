## ROS2 with CA (Cellular Automata) for Robot Navigation

Cellular automata are used to process the local obstacle map generated from LIDAR data.

Based on the updated map, the robot makes navigation decisions: if there is an obstacle ahead, it changes course or activates an "escape mode" to escape if it gets stuck.


 <div style="text-align:center;">
    <img src="image/1.png" alt="1" width="800">
</div>

creating a new folder ``ca_robot.py `` inside which I put the folder ```src```

in the terminal inside ```ca_robot``` I write:

```shell
colcon build
colcon build --symlink-install
```
after
```shell 
source install/setup.bash
```

finally
```shell
ros2 launch ca_robot_bringup ca_gazebo.launch.xml  
```
and in another terminal

```shell
ros2 run ca_robot_controller ca
```

![Image](https://github.com/user-attachments/assets/7b62e6c5-008c-4950-9b83-9f6f976a12d9)
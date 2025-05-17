## ROS2 �� CA (�������� ��������) ��� �������� ������ 

������� �����  ��������� ��������� ```(cellular automata)``` ��� ����������� ��� ������� ����� �������� ��� ������������� ��� �� �������� ��� LIDAR.

�� ���� ��� ����������� �����,�� ������ ������� ��������� ���������: �� ������� ������� �������, ������� ������ � ����������� ��� "escape mode" ��� �� ������� �� ��������.


 <div style="text-align:center;">
    <img src="image/1.png" alt="1" width="800">
</div>

���������� ���� ����������� ������� ``ca_robot.py `` ���� ���� ���� ��� ������ ```src```

��� �������� ���� ���� ```ca_robot``` ����� :

```shell
colcon build
colcon build --symlink-install
```
����   
```shell 
source install/setup.bash
```

��� ����� 
```shell
ros2 launch ca_robot_bringup ca_gazebo.launch.xml  
```
��� �� ���� ��������� 

```shell
ros2 run ca_robot_controller ca
```

![Image](https://github.com/user-attachments/assets/7b62e6c5-008c-4950-9b83-9f6f976a12d9)
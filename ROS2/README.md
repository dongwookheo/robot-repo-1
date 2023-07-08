# 프로젝트에서 사용한 명령어 정리

# minibot_navigation2
 - Mapping
 - navigation
 - 4개의 시나리오
   1. 버튼(pyqt)을 눌러 특정 테이블로 이동
   2. 아루코 마커 감지하면 일정 거리 전진 & 전자석 off를 통한 테이블 셋팅
   3. 매장 내 CCTV에서 Yolov8 모델을 통해 사람을 인식하고 그 좌표를 토픽으로 발행. 토픽을 구독해서 좌표를 장애물로 실시간 처리를 해서 효율적인 global path planning 한다.
   4. 로봇에 부착된 뎁스카메라에서 Yolov8 모델을 통해 사람을 인식하고 그 거리 값을 토픽으로 발행. 토픽을 구독해서 특정 거리 이하가 되면 회피하거나 정지한다.   

## Map Building

### For Gazebo simulation
```shell
$ ros2 launch minibot_bringup bringup_robot_gazebo.launch.py world_name:=simple_building.world
```
```shell
$ ros2 launch minibot_navigation2 map_building.launch.py use_sim_time:=true
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/map_building.rviz
```
```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=base_controller/cmd_vel_unstamped
```

### For Real Robot Mapping
```shell
$ ros2 launch minibot_bringup bringup_robot.launch.py
```
```shell
$ ros2 launch minibot_navigation2 map_building.launch.py
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/map_building.rviz
```
```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=base_controller/cmd_vel_unstamped
```


### For Navigation (Gazebo)
```shell
$ ros2 launch minibot_bringup bringup_robot_gazebo.launch.py world_name:=simple_building.world
```
```shell
$ ros2 launch minibot_navigation2 bringup_launch.py use_sim_time:=true map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/simple_building.yaml
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```


### For Navigation (Real Robot)
```shell
$ ros2 launch minibot_bringup bringup_robot.launch.py
```
```shell
$ ros2 launch minibot_navigation2 bringup_launch.py map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/<map-name>.yaml
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```

### 1st scenario
#### move to qt/src
```shell
$ ros2 launch minibot_bringup bringup_robot.launch.py
```
```shell
$ ros2 launch minibot_navigation2 bringup_launch.py map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/<map-name>.yaml
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```
```shell
$ python3 button.py
```
#### move to navi_test  / source install/setup.bash
```shell
$ ros2 run navi_test_package navi_test_node
```

### 2nd scenario
#### move to minibot_navigation2/solver
```shell
$ ros2 launch minibot_bringup bringup_robot.launch.py
```
```shell
$ ros2 launch minibot_navigation2 bringup_launch.py map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/<map-name>.yaml
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```
```shell
$ python3 aruco.py
```
#### move to navi_test  / source install/setup.bash
```shell
$ ros2 run navi_test_package navi_test_node
```

### 3nd scenario
```shell
$ ros2 launch minibot_bringup bringup_robot.launch.py
```
```shell
$ ros2 launch minibot_navigation2 bringup_launch.py map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/<map-name>.yaml
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```
#### check cctv_dev
```shell
$ ros2 run cctv_person_detect webcam
```
```shell
$ ros2 run cctv_person_detect coordinate_publisher
```
#### move to navi_test  / source install/setup.bash
```shell
$ ros2 run navi_test_package navi_test_node
```

### 4nd scenario
```shell
$ ros2 launch minibot_bringup bringup_robot.launch.py
```
```shell
$ ros2 launch minibot_navigation2 bringup_launch.py map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/<map-name>.yaml
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```
#### check realsense_dev
```shell
$ ros2 run depth_package realsense_publisher
```
```shell
$ ros2 run depth_package center_publisher
```
```shell
$ ros2 run depth_package depth_subscriber
```
#### move to navi_test  / source install/setup.bash
```shell
$ ros2 run navi_test_package navi_test_node
```





# Local Camera System
### 패키지 설명
- 로봇에 장착된 Local Camera(Realsense D435)를 통해 급작스럽게 튀어나오는 사람을 인식하고 depth를 측정
- depth를 기반으로 로봇이 정지 혹은 회피 주행에 활용해 안정적인 서빙을 도모
- 라즈베리파이의 컴퓨터 리소스의 부족으로, Jetson 보드 및 서버 컴퓨터를 활용
  
#### depth_package
- realsense_publisher.py: Jetson에서 D435를 통해 color image를 publish. 이후, 객체의 중심 좌표 list를 subscribe하여 depth frame으로부터 depth를 얻고 가장 작은 값을 publish.
- center_publisher.py: 서버 컴퓨터에서 color image를 subscribe하고, YOLO를 통해 객체를 인식 후 중심 좌표를 list의 형태로 publish.
- depth_subscriber.py: 라즈베리파이에서 depth를 subscribe.
  

#### 명령어
Jetson에서 실행
```
$ ros2 run depth_package realsense_publisher
```
서버 컴퓨터에서 실행
```
$ ros2 run depth_package center_publisher
```
라즈베리파이에서 실행
```
$ ros2 run depth_package depth_subscriber
```

#### 시연영상
<img src=https://github.com/addinedu-amr-2th/robot-repo-1/assets/124948998/9a3d5dd9-85cf-4442-8046-30dd9686f229>



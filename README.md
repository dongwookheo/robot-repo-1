## 프로젝트 기획 및 컨셉
<img src="./image/Image01.jpg" alt="이미지" width="640" height="320"> <br>
<span style='background-color:#dcffe4'>사람이 도와주지 않으면 서빙이 안돼는 현세대 서빙로봇을 보완한 차세대 서빙 로봇</span> <br>

## 데모영상
<img src="./image/demo.gif" alt="이미지" width="640" height="240"> <br>
Planning part : 임용재 <br>
System part : 노태형 <br>
DL part: 신재성 <br>
Computer Vision part : 허동욱 <br>
Firmware part : 성용호 <br>
SLAM & Navigation : 김정현 <br>

## 시스템
### 프로토타입

<b align="left">
<img src="./image/ProtoType.png" alt="이미지" width="320" height="320">
<b align="right">
<img src="./image/Diagram01.png" alt="이미지" width="320" height="320">
<b align="left"> <br>

#### 로봇 컴퓨터
Model : Jetson NX Develop Kit <br>
RAM : 8 GB <br>
OS : Linux 20.04 LTS <br>
※JetPack = 5.1.1

#### 구동부
MCU : STM32 Controller Board <br>
Motor : 12V DC Motor X 4 <br>
Wheel : Mecanum Wheel

### 실물 기체
<b align="left">
<img src="./image/Real_Machine.png" alt="이미지" width="320" height="320">
<b align="right">
<img src="./image/Diagram02.png" alt="이미지" width="320" height="320">
<b align="left"> <br>

#### 로봇 컴퓨터
Model : Raspberry pi 4 <br>
CPU : 6-core NVIDIA Carmel ARM v8.2 64-bit CPU 6MB L2 + 4MB L3<br>
GPU : NVIDIA Volta with 384 CUDA and 48 Tensor Cores <br>
RAM : 4 GB <br>
OS : Linux 22.04 LTS <br>
※JetPack = 5.1.1

#### 로컬 캠
Model : Jetson NX Develop Kit <br>
RAM : 8 GB <br>
OS : Linux 20.04 LTS <br>
※JetPack = 5.1.1

#### 구동부
MCU : Arduino Mega Controller Board <br>
Motor : 12V DC Motor X 2 <br>
Wheel : differential steering

<위 2개 너무 긴거 같아서 따로 시스템 dev로 뺄까 고민 중>

### 사용된 기술

#### **YOLO**
돌발상황시 사람의 전체가 아닌 팔과 다리가 우선적으로 보이므로 <br>
팔과 다리만을 인식하는 모델 학습 및 라벨링 적용 <br>
<img src="./image/YOLO_Custom.jpg" alt="이미지" width="320" height="320">
<b align="left"> <br>

 Model | Trained |
|-------|---------|
| recognize Arm and Leg | [download (245MB)](https://drive.google.com/file/d/1DHGC-n0PIB-iuSVyvKAjuUM-vJ_csmHB/view?usp=sharing) |


#### **컴퓨터 비전**
<img src="./image/camera_system.png" alt="이미지" width="649" height="320"> <br>

**[글로벌 카메라처리](https://github.com/addinedu-amr-2th/robot-repo-1/blob/master/ROS2/src/cctv_person_detect/README.md)**  <br>

**[로컬 카메라처리](https://github.com/addinedu-amr-2th/robot-repo-1/blob/master/ROS2/src/depth_package/README.md)** <br>


### 하드웨어
메카넘 휠 로봇을 teleop_twist_keyboard를 이용해 주행<br>
```shell
$ ros2 run serial_py2stm serial_py2stm
```
```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
<img src="./image/image45.gif" alt="이미지" width="320" height="300"> <br>
#### S-Curve Profile
음식물을 안전하게 서빙하기 위해 급격한 가속 변화를 줄여 동작을 부드럽게 한다.<br>
<img src="./image/image48.gif" alt="이미지" width="320" height="300"/> <br>
적용X   
<img src="./image/image49.gif" alt="이미지" width="320" height="300"> <br>
적용O   

### Navigation & SLAM
#### [주행 시나리오 시연 때 사용한 명령어 정리](https://github.com/addinedu-amr-2th/robot-repo-1/blob/master/ROS2/README.md)
#### 1번, 2번 시나리오 
<img src="./image/Untitled (1).gif" alt="이미지" width="640" height="240"> <br>
#### 3번 시나리오
<img src="./image/Untitled (2).gif" alt="이미지" width="640" height="240"> <br>
#### 4번 시나리오
<img src="./image/Untitled (3).gif" alt="이미지" width="640" height="240"> <br>

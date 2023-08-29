### 하드웨어
메카넘 휠 로봇을 teleop_twist_keyboard를 이용해 주행<br>
```shell
$ ros2 run serial_py2stm serial_py2stm
```
```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
<img src="../../../image/image45.gif" alt="이미지" width="320" height="300"> <br>
#### S-Curve Profile
음식물을 안전하게 서빙하기 위해 급격한 가속 변화를 줄여 동작을 부드럽게 한다.<br>
<img src="../../../image/image48.gif" alt="이미지" width="320" height="300"/> <br>
적용X   
<img src="../../../image/image49.gif" alt="이미지" width="320" height="300"> <br>
적용O   

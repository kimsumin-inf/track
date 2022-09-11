# track
220911 - 점심

Track Control developed and then, this commit is beta 1

## Lidar

- roslaunch track_lidar track_lidar.launch

-> 런치 파일 내부에 shutdown 후  true 시 한 바퀴 주행 후 shutdown

## Track_Path

- roslaunch track_path track_path.launch

-> 맵 바퀴 수

-> Global Path ->  속도 4.5 KPH 이후 Local Pose의 Kalman Filtering 된 위치로 맵 생성 

-> 초기 UTM

-> 맵 제작 완료 state

-> global path smoothing (start point & end point smoothing)

## Track_Localization

- roslaunch track_localizer track_localizer.launch

-> 한 바퀴 돈 이후 Kalman Filter를 통해 GPS, IMU, Encoder를 융합한 위치 측위 완료

-> 투명 Axes : gps raw

-> 불투명 Axes : fillterd_gps

## track control

- roslaunch track_control track_control.launch 



## 해야 할 일

- 실차 oscillation 해결 

- map의 직선 구간과 회전 구간을 State로 구분한 후 구간 별로 속도를 제어함 

- 실차 검증 필요 
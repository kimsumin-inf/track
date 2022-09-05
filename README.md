# track
220905 - 저녁 

Track Map Generate & Kalman Filter based ERP-42 Localization


Lidar

roslaunch track_lidar track_lidar.launch

-> 런치 파일 내부에 shutdown가 true 시 한 바퀴 주행 후 shutdown

Track_Path

roslaunch track_path track_path.launch

-> 맵 바퀴 수

-> Global Path

-> 초기 UTM

-> 맵 제작 완료 state

Track_Localization

roslaunch track_localizer track_localizer.launch

-> 한 바퀴 돈 이후 Kalman Filter를 통해 GPS, IMU, Encode를 융합한 위치 측위 완료

-> 투명 Axes : gps raw

-> 불투명 Axes : fillterd_gps

해야 할 일

track control

global path smoothing (start point & end point smoothing)

실차 테스트
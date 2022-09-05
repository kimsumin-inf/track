# track
220905

Track Map Generator module and Global Path module are fusioned

Lidar
roslaunch track_lidar track_lidar.launch

-> 런치 파일 내부에 shutdown가 true 시 한 바퀴 주행 후 shutdown

Track_Path
roslaunch track_path track_path.launch
-> 맵 바퀴 수
-> Global Path
-> 초기 UTM
-> 맵 제작 완료 state

해야 할 일
track control
track localizer
global path smoothing (start point & end point smoothing)


# first terminal 
source devel/setup.bash 
roslaunch lv_slam dlo_lfa_ggo_szu.launch calib_file:='/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/config/szu_calib/calib.txt'     odom_file:='/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/dlo_lfa_global/data/szu_01_odom.txt' seq:=01  lfa_output_path:='/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo' label_path:="/media/chenshoubin/HD_CHEN_4T/02data/szu_data/726data/new2_2021-07-31-17-06-25_label.txt"

# 2nd terminal

rosbag play --clock  xx.bag rslidar_points:=/velodyne_points   -r 0.5

# 3rd terminal
source devel/setup.bash 
rosservice call /global_graph/dump "destination: '/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/data/'  "

rosservice call /global_graph/save_map "{resolution: 0.05, destination: '/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/data/map.pcd'}"

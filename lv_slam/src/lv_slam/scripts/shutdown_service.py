#!/usr/bin/env python

import rospy
import subprocess

def call_service(service_name, args):
    rospy.wait_for_service(service_name)
    command = f"rosservice call {service_name} {args}"
    
    try:
        result = subprocess.call(command, shell=True)
        if result == 0:
            rospy.loginfo(f"Service call to {service_name} succeeded")
        else:
            rospy.logerr(f"Service call to {service_name} failed with return code: {result}")
    except Exception as e:
        rospy.logerr(f"Service call to {service_name} failed: {e}")

if __name__ == "__main__":
    rospy.init_node('shutdown_services_node', anonymous=True)

    # First service call
    service_name_1 = '/global_graph/dump'
    args_1 = "\"destination: '/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/data/'\""
    call_service(service_name_1, args_1)
    
    # Second service call
    service_name_2 = '/global_graph/save_map'
    args_2 = "\"{resolution: 0.05, destination: '/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/data/map.pcd'}\""
    call_service(service_name_2, args_2)
# 在launch结束时运行两个service：
# rosservice call /global_graph/dump "destination: '/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/data/'  "
# rosservice call /global_graph/save_map "{resolution: 0.05, destination: '/home/jq/jq/GML-WORKSPACE/chun/slamspace/lv_slam/src/lv_slam/data/szu_lv_dlo_lfa_ggo/data/map.pcd'}"
#!/bin/bash

if [ "$#" -le 0 ]; then
	echo "usage: $0 [drone_namespace] "
	exit 1
fi

# Arguments
drone_namespace=$1
init_x=$2
init_y=$3
init_z=$4

pushd $AEROSTACK2_PROJECTS/mbzirc/
source ./launch/launch_tools.bash

# declare -r COMPRESSED_IMAGE_TOPIC='stream/compressed_image'
declare -r COMPRESSED_IMAGE_TOPIC='/image'
declare -r REPORT_TOPIC='report'

new_session $drone_namespace  

new_window 'yolo_detector' " ros2 launch yolo_object_detector yolo_object_detector_launch.py \
  drone_id:=$drone_namespace \
  use_sim_time:=true \
  config:=./config/darknet_params_objects.yaml \
  camera_topic:=slot6/image_raw"

new_window 'depthtection' " ros2 launch depthtection depthtection_launch.py \
  namespace:=$drone_namespace \
  camera_topic:=slot6 \
  base_frame:=$drone_namespace \
  show_detection:=true \
  use_sim_time:=true"

echo -e "Launched drone $drone_namespace. For attaching to the session, run: \n  \t $ tmux a -t $drone_namespace"
popd 
popd 

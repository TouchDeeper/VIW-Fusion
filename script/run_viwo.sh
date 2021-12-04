#!/bin/bash
source /home/td/slam/vins_fusion_ws/devel/setup.bash
roslaunch vins ridgeback_viwo.launch&
VIWO_PID=$!
echo "VIWO_PID = "$VIWO_PID
sleep 1s
rosbag play /media/td/00360FE3360FD894/dataset/vins_fusion_ridgeback/ridgeback_vins3.bag -q&
#rosbag play /media/td/00360FE3360FD894/dataset/vins_fusion_ridgeback/ridgeback_vins4.bag -q&
#rosbag play /media/td/00360FE3360FD894/dataset/circle_ridgeback/scale1.bag -q&
BAG_PID=$!
echo "BAG_PID = "$BAG_PID
#isGlobalPipelineExist=`ps -ef|grep data_gen_VIW|grep -v "grep"|wc -l`

#while [ "$isGlobalPipelineExist" -ne "0" ]
#do
#  echo "sleep 1s"
#  sleep 1s
#  isGlobalPipelineExist=`ps -ef|grep data_gen_VIW|grep -v "grep"|wc -l`
#done
while [ -d /proc/$BAG_PID ]
do
  echo "sleep 1s"
  sleep 1s
#  isGlobalPipelineExist=`ps -ef|grep data_gen_VIW|grep -v "grep"|wc -l`
done

rosnode kill -a
echo "kill VIWO_PID"

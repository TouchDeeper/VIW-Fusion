#!/bin/bash

#power=1
#power=$(echo "($run_num-1)/2"|bc)

#param="$1"

trap 'onCtrlC' INT
keep=1
function onCtrlC () {
    echo 'Ctrl+C is captured'
    keep=0
    kill -9 $VIWO_PID
    echo "kill VIWO_PID"
    kill -9 $BAG_PID
    echo "kill BAG_PID"
    rosnode kill -a
    sed -i "s:^wheel_gyro_noise_sigma\:.*[0-9]$:wheel_gyro_noise_sigma\: 0.003:g" /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
    sed -i "s:^wheel_velocity_noise_sigma\:.*[0-9]$:wheel_velocity_noise_sigma\: 0.003:g" /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
    sed -i "s:^output_path.*\"$:output_path\: \"/home/td/slam/vins_fusion_ws/src/VINS-Fusion/output\":g" /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
    exit
}
#sed -i "s/^acc_n: [0-9]*\.[0-9]*/acc_n: $acc_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^acc_w: [0-9]*\.[0-9]*/acc_w: $acc_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^gyr_n: [0-9]*\.[0-9]*/gyr_n: $gyr_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#sed -i "s/^gyr_w: .[0-9]*\.[^[:space:]]*\|^gyr_w: \.[^[:space:]]*/gyr_w: $gyr_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml

#let power=(run_num-1)/2
#eval $(awk -v power_awk="$power" 'BEGIN {scale_awk=0.5^power_awk; printf "scale=%.15f", scale_awk}')
#scale=$(awk -v power_awk="$power" 'BEGIN {scale_awk=0.5^power_awk; printf "%.15f", scale_awk}')

#echo $scale

#scale=$(echo "0.5**$power"|bc)
#sed -i "s:^verbose=.*[a-zA-Z]$:verbose=false:g" ~/imta_project/pose_estimation/config.ini
#sed -i "s:^visual=.*[a-zA-Z]$:visual=false:g" ~/imta_project/pose_estimation/config.ini
#sed -i "s:^mutation_method=.*[a-zA-Z]$:mutation_method=mutate_by_chromosomes:g" ~/imta_project/pose_estimation/config.ini
rosnode kill -a
cd /home/td/slam/vins_fusion_ws/src/VINS-Fusion/script

echo "------running noise loop------"
for((j=0;j<10;j++))
do
  gyro_noise=$(echo "0.001+$j*0.001"|bc)
  sed -i "s:^wheel_gyro_noise_sigma\:.*[0-9]$:wheel_gyro_noise_sigma\: $gyro_noise:g" /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
  for((k=0;k<10;k++))
  do
    velocity_noise=$(echo "0.001+$k*0.001"|bc)
    sed -i "s:^wheel_velocity_noise_sigma\:.*[0-9]$:wheel_velocity_noise_sigma\: $velocity_noise:g" /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
    echo "wheel_gyro_noise_sigma:$gyro_noise       wheel_velocity_noise_sigma:$velocity_noise"
    sed -i "s:^output_path.*\"$:output_path\: \"/home/td/slam/vins_fusion_ws/src/VINS-Fusion/output/viwo\_$j\_$k\":g" /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
    mkdir /home/td/slam/vins_fusion_ws/src/VINS-Fusion/output/viwo_$j\_$k
    ./run_viwo.sh
  done
#  for((i=0;i<$[run_num];i++))
#  do
    #power_=$(echo "i-$power"|bc)
    #let power_=i-power
    #scale=$(awk -v power_awk="$power_" 'BEGIN {scale_awk=2^power_awk; printf "%.13f", scale_awk}')
    # modify the vio_path
    #sed -i "s:^vio_path.*\.txt\"$:vio_path\: \"/home/wang/vins_ws/src/VINS-Mono/output/euroc_result/$loop/vio$i\.txt\":g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
    #if [ "$loop" == "ACC_N" ];then
    #  acc_n_scaled=$(echo "$acc_n*$scale"|bc)
      # modify the acc_n's value
    #  sed -i "s/^acc_n: [0-9]*\.[0-9]*/acc_n: $acc_n_scaled/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
      #echo "scale="$scale "i="$i
     # echo "acc_n="$acc_n_scaled "i="$i
    #fi
    #if [ "$loop" == "ACC_W" ];then
    #  acc_w_scaled=$(echo "$acc_w*$scale"|bc)
    #  # modify the acc_w's value
    #  sed -i "s/^acc_w: [0-9]*\.[0-9]*/acc_w: $acc_w_scaled/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
    #  #echo "scale="$scale "i="$i
    #  echo "acc_w="$acc_w_scaled "i="$i
    #fi


#  done

  #restore the noise parameters and vio path
  #sed -i "s/^acc_n: [0-9]*\.[0-9]*/acc_n: $acc_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
  #sed -i "s/^acc_w: [0-9]*\.[0-9]*/acc_w: $acc_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
  #sed -i "s/^gyr_n: [0-9]*\.[0-9]*/gyr_n: $gyr_n/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
#    sed -i "s/^gyr_w: .[0-9]*\.[^[:space:]]*\|^gyr_w: \.[^[:space:]]*/gyr_w: $gyr_w/g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
  #sed -i "s:^vio_path.*\.txt\"$:vio_path\: \"/home/wang/vins_ws/src/VINS-Mono/output/euroc_result/vio\.txt\":g" ~/vins_ws/src/VINS-Mono/config/euroc/euroc_config.yaml
done
kill -9 $VIWO_PID
echo "kill VIWO_PID"
kill -9 $BAG_PID
echo "kill BAG_PID"
rosnode kill -a

sed -i "s:^wheel_gyro_noise_sigma\:.*[0-9]$:wheel_gyro_noise_sigma\: 0.003:g" /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
sed -i "s:^wheel_velocity_noise_sigma\:.*[0-9]$:wheel_velocity_noise_sigma\: 0.003:g" /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml
sed -i "s:^output_path.*\"$:output_path\: \"/home/td/slam/vins_fusion_ws/src/VINS-Fusion/output\":g" /home/td/slam/vins_fusion_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config_ridgeback.yaml


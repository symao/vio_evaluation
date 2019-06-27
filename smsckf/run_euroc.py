import os
import sys
import time
import subprocess,signal

def single_run_ros(roslaunch_cmd, bag_file, output_file, res_dir):
    data_dir = os.path.basename(bag_file)[:-4]
    print('Run EuRoC data: %s'%data_dir)
    time.sleep(5)

    p = subprocess.Popen(roslaunch_cmd.split(' '))
    time.sleep(5)

    cmd = 'rosbag play %s'%bag_file
    os.system(cmd)
    time.sleep(15)

    if not os.path.exists(res_dir):
        os.makedirs(res_dir)

    cmd = 'mv %s %s/out_%s.txt'%(output_file, res_dir, data_dir)
    os.system(cmd)

    p.kill()
    print('\n'*5)


def batch_run_ros(ros_cmd, bag_dir, output_file, res_dir):
    bag_list = [os.path.join(bag_dir, x) for x in os.listdir(bag_dir) if x[-4:]=='.bag']
    for bag_file in bag_list:
        single_run_ros(ros_cmd, bag_file, output_file, res_dir)

if __name__ == '__main__':
    bag_dir = '/home/symao/data/euroc/rosbag'
    output_file = 'vio.txt'

    # # run VINS_MONO
    # batch_run_ros('roslaunch vins_estimator euroc.launch', bag_dir, output_file, './results')

    # run S-MSCKF
    batch_run_ros('roslaunch msckf_vio msckf_vio_euroc.launch', bag_dir, output_file, './results')
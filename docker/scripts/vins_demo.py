#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import subprocess as sbp
import argparse
import rosbags
import wget
import time
import sys
import os

BASE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../..')

def argparser(argv):
    parser = argparse.ArgumentParser(description='VINS-Fusion Docker')
    parser.add_argument('--dataset_url', type=str, default='http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag', help='dataset url')
    parser.add_argument('--config', type=str, default='%s/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml' % os.path.join(BASE_DIR, '..'), help='config file')
    parser.add_argument('--rviz', action='store_true', help='launch rviz')
    parser.add_argument('--debug', action='store_true', help='debug mode')
    parser.add_argument('--pack', action='store_true', help='pack binary')
    parser.add_argument('--demo', action='store_true', help='run demo')
    parser.add_argument('--d435i', action='store_true', help='run with d435i')
    parser.add_argument('--bridge', action='store_true', help='launch ros1_bridge')
    args = parser.parse_args(argv)
    return args

def compile_vins():
    CMD = 'bash -c \"source /opt/ros/foxy/setup.bash && cd %s && colcon build --symlink-install --allow-overriding cv_bridge --packages-skip ros1_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release\"' % os.path.join(BASE_DIR, '..', '..')
    os.system(CMD)

def pack_vins():
    CMD = 'bash -c \"cd %s && tar -zcvhf src/install.tar.gz install && chmod 777 src/install.tar.gz\"' % os.path.join(BASE_DIR, '..', '..')
    os.system(CMD)

def download_dataset(dataset_url):
    filename = wget.detect_filename(dataset_url)
    rosbag_path = os.path.join(BASE_DIR, 'dataset', filename)
    ros2bag_path = os.path.join(BASE_DIR, 'dataset', filename[:-4])

    os.makedirs(os.path.join(BASE_DIR, 'dataset'), exist_ok=True)
    if os.path.exists(ros2bag_path):
        print('Dataset %s already exists' % filename)
    else:
        print('Downloading dataset %s' % filename)
        wget.download(dataset_url, out=os.path.join(BASE_DIR, 'dataset', filename))

    if not os.path.exists(ros2bag_path):
        print('Converting rosbag to ros2bag')
        sbp.run('rosbags-convert %s --dst %s' % (rosbag_path, ros2bag_path), shell=True, capture_output=True)
        os.remove(rosbag_path)

    return ros2bag_path

def play_rosbag(rosbag_path):
    CMD = 'bash -c \"source /opt/ros/foxy/setup.bash && ros2 bag play -r 2 %s\"' % rosbag_path
    sbp.Popen(CMD, shell=True)

def run_vins(config):
    CMD = 'bash -c \"source /opt/ros/foxy/setup.bash && source %s/../../install/local_setup.bash && ros2 run vins vins_node %s\"' % (BASE_DIR, config)
    os.system(CMD)

def realsense2_camera():
    os.system('cp %s/config/realsense_d435i/rs_launch_vins.py %s/../../install/realsense2_camera/share/realsense2_camera/launch' % (BASE_DIR, BASE_DIR))
    CMD = 'bash -c \"source /opt/ros/foxy/setup.bash && source %s/../../install/local_setup.bash && ros2 launch realsense2_camera rs_launch_vins.py\"' % (BASE_DIR)
    sbp.Popen(CMD, shell=True)

def run_loop_fusion(config):
    CMD = 'bash -c \"source /opt/ros/foxy/setup.bash && source %s/../../install/local_setup.bash && ros2 run loop_fusion loop_fusion_node %s\"' % (BASE_DIR, config)
    sbp.Popen(CMD, shell=True)

def launch_rviz():
    CMD = 'bash -c \"source /opt/ros/foxy/setup.bash && source %s/../../install/local_setup.bash && ros2 launch vins vins_rviz.launch.xml\"' % BASE_DIR
    sbp.Popen(CMD, shell=True)

def compile_ros1_bridge():
    CMD = '''
    bash -c "
    export ROS1_INSTALL_PATH=/opt/ros/noetic && \\
    export ROS2_INSTALL_PATH=/opt/ros/foxy && \\
    source /opt/ros/noetic/setup.bash && \\
    source /opt/ros/foxy/setup.bash && \\
    cd %s && \\
    colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
    "
    ''' % os.path.join(BASE_DIR, '..', '..')
    os.system(CMD)

def launch_ros1_bridge():
    compile_ros1_bridge()
    CMD = '''
    bash -c "
    source /opt/ros/noetic/setup.bash && \\
    roscore &
    "
    '''
    sbp.Popen(CMD, shell=True)
    time.sleep(3)

    CMD = '''
    bash -c "
    source /opt/ros/noetic/setup.bash && \\
    source /opt/ros/foxy/setup.bash && \\
    export ROS_MASTER_URI=http://localhost:11311 && \\
    source %s/install/setup.bash && \\
    ros2 run ros1_bridge dynamic_bridge
    "
    ''' % os.path.join(BASE_DIR, '..', '..')
    sbp.Popen(CMD, shell=True)
    time.sleep(3)

def main(argv):
    args = argparser(argv)
    compile_vins()
    if args.pack:
        pack_vins()
    elif args.demo:
        rosbag_path = download_dataset(args.dataset_url)
        if args.rviz:
            launch_rviz()
        play_rosbag(rosbag_path)
        if args.bridge:
            launch_ros1_bridge()
        run_vins(args.config)
    elif args.d435i:
        if args.rviz:
            launch_rviz()
        realsense2_camera()
        run_vins(os.path.join(BASE_DIR, 'config', 'realsense_d435i', 'realsense_stereo_imu_config.yaml'))
        run_loop_fusion(os.path.join(BASE_DIR, 'config', 'realsense_d435i', 'realsense_stereo_imu_config.yaml'))

if __name__ == '__main__':
    main(sys.argv[1:])

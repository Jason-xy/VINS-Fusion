#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import subprocess as sbp
import argparse
import rosbags
import wget
import sys
import os

BASE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../..')

def argparser(argv):
    parser = argparse.ArgumentParser(description='VINS-Fusion Docker')
    parser.add_argument('--dataset_url', type=str, default='http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag', help='dataset url')
    parser.add_argument('--config', type=str, default='%s/config/euroc/euroc_stereo_imu_config.yaml' % BASE_DIR, help='config file')
    args = parser.parse_args(argv)
    return args

def download_dataset(dataset_url):
    filename = wget.detect_filename(dataset_url)
    os.makedirs(os.path.join(BASE_DIR, 'dataset'), exist_ok=True)
    if os.path.exists(os.path.join(BASE_DIR, 'dataset', filename)):
        print('Dataset %s already exists' % filename)
    else:
        print('Downloading dataset %s' % filename)
        wget.download(dataset_url, out=os.path.join(BASE_DIR, 'dataset', filename))
    rosbag_path = os.path.join(BASE_DIR, 'dataset', filename)
    ros2bag_path = os.path.join(BASE_DIR, 'dataset', filename[:-4])
    if not os.path.exists(ros2bag_path):
        print('Converting rosbag to ros2bag')
        sbp.run('rosbags-convert %s --dst %s' % (rosbag_path, ros2bag_path), shell=True, capture_output=True)
    return ros2bag_path

def play_rosbag(rosbag_path):
    CMD = 'bash -c \"source /opt/ros/foxy/setup.bash && ros2 bag play %s\"' % rosbag_path
    sbp.Popen(CMD, shell=True)

def run_vins(config):
    CMD = 'bash -c \"source /opt/ros/foxy/setup.bash && source %s/../install/local_setup.bash && ros2 run vins vins_node %s\"' % (BASE_DIR, config)
    sbp.Popen(CMD, shell=True)
    CMD = 'bash -c \"source /opt/ros/foxy/setup.bash && source %s/../install/local_setup.bash && ros2 launch vins vins_rviz.launch.xml\"' % BASE_DIR
    os.system(CMD)
    

def main(argv):
    args = argparser(argv)
    rosbag_path = download_dataset(args.dataset_url)
    play_rosbag(rosbag_path)
    run_vins(args.config)

if __name__ == '__main__':
    main(sys.argv[1:])

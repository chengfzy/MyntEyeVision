"""
Analysis the Recorded Data Quality
"""

import os
from os import path
import numpy as np
import matplotlib.pyplot as plt
import argparse


def analysis_img_fps(root_folder: str, camera_name: str):
    img_folder = path.join(root_folder, camera_name)
    times = []
    for f in os.listdir(img_folder):
        if f.endswith('.png') or f.endswith('.jpg'):
            times.append(float(f.split('.')[0].split('_')[-1]) / 100.)  # to ms
    times = np.array(sorted(times))
    delta_time = times[1:] - times[:-1]
    print(f'mean = {delta_time.mean():.5f} ms, min = {delta_time.min():.5f} ms, max = {delta_time.max():.5f} ms',
          f', freq = {1000 / delta_time.mean():.5f} Hz')

    fig = plt.figure(f'Delta Time of {camera_name} Camera')
    ax = fig.add_subplot(1, 1, 1)
    ax.set_title(f'Delta Time of {camera_name} Camera')
    ax.plot(range(len(delta_time)), delta_time, '.b')
    ax.set_xlabel('Index')
    ax.set_ylabel('Delta Time (ms)')
    ax.grid(True)


def analysis_imu_fps(imu_file: str):
    imu = np.loadtxt(imu_file, delimiter=',')

    # acc
    acc_times = imu[imu[:, 1] != 0, 0] / 100.  # to ms
    acc_delta = acc_times[1:] - acc_times[:-1]
    print(f'acc, mean = {acc_delta.mean():.5f} ms, min = {acc_delta.min():.5f} ms, max = {acc_delta.max():.5f} ms',
          f', freq = {1000 / acc_delta.mean():.5f} Hz')
    # gyro
    gyro_times = imu[imu[:, -1] != 0, 0] / 100.  # to ms
    gyro_delta = gyro_times[1:] - gyro_times[:-1]
    print(f'gyro, mean = {gyro_delta.mean():.5f} ms, min = {gyro_delta.min():.5f} ms, max = {gyro_delta.max():.5f} ms',
          f', freq = {1000 / gyro_delta.mean():.5f} Hz')

    # plot
    fig = plt.figure('Delta Time of IMU')
    ax = fig.add_subplot(2, 1, 1)
    ax.set_title('Delta Time of Accelerator')
    ax.plot(range(len(acc_delta)), acc_delta, '.b')
    ax.set_xlabel('Index')
    ax.set_ylabel('Delta Time (ms)')
    ax.grid(True)
    ax = fig.add_subplot(2, 1, 2)
    ax.set_title('Delta Time of Gyroscope')
    ax.plot(range(len(gyro_delta)), gyro_delta, '.b')
    ax.set_xlabel('Index')
    ax.set_ylabel('Delta Time (ms)')
    ax.grid(True)


def main():
    # argument parser
    parser = argparse.ArgumentParser(description='Analysis the recorded data quality of MYNT EYE camera')
    parser.add_argument('--folder', type=str, required=True, help='data folder')
    args = parser.parse_args()
    print(args)
    data_path = args.folder

    # check the image timestamp
    analysis_img_fps(data_path, "left")
    analysis_img_fps(data_path, "right")
    analysis_imu_fps(path.join(data_path, 'imu.txt'))

    plt.show(block=True)


if __name__ == "__main__":
    main()
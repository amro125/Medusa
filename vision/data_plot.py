from pprint import pprint
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def plot_data(processed_data):
    algorithms = ['raw', 'smoothed']
    coords = ['head_x', 'head_y', 'head_z', 'shoulder_x', 'shoulder_y', 'shoulder_z']

    fig, axs = plt.subplots(3, 2, figsize=(12, 12), sharex=True)
    axs = axs.ravel()

    for idx, coord in enumerate(coords):
        for algo in algorithms:
            axs[idx].plot(processed_data['timestamp'], processed_data[f'{algo}_{coord}'], label=f'{algo}')
        axs[idx].set_title(f'{coord} over time')
        axs[idx].set_ylabel(f'{coord} value')
        axs[idx].legend()

    plt.xlabel('Timestamp')
    plt.tight_layout()
    plt.show()


def plot_joint_angles(jointdata):
    joints = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7']

    for joint in joints:
        plt.figure(figsize=(12, 6))
        plt.title(f'{joint} joint angles over time')

        plt.plot(jointdata['timestamp'], jointdata[f'{joint}'], label=f'{joint}')

        plt.xlabel('Time (sec)')
        plt.ylabel('Joint Angle (deg)')
        plt.legend()
        plt.show()


def process_data(filename):
    data = pd.read_csv(filename, header=0)
    data['timestamp'] = data['timestamp'] - data['timestamp'][0]
    return data


if __name__ == '__main__':
    data = process_data('joint_data.csv')
    plot_joint_angles(data)
